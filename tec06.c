/**
 * Code by Russell Graves, https://syonyk.blogspot.com/, 2018 AD
 * 
 * This code is free for anyone to use for anything.  It comes with no
 * warrantee, and if you use this in production without rewriting it and
 * understanding it, you're being stupid.  It's a proof of concept demonstrating
 * how to configure a FTDI device for the bizarre baud rate, and how to decode
 * the data.  That's all.  Nothing more, nothing else.  I don't even promise it
 * will work for you!
 * 
 * However, if you find it helpful and want to toss beer money my way, there's
 * a donate button on my blog.
 * 
 * ============================================================================
 * 
 * Serial analysis for the TEC06 battery tester gizmo.  This is a bizarre little
 * device that runs at 128,000 baud, even parity.
 * 
 * However, it works on Windows, so I reverse engineered the protocol.  Partly
 * because it was useful, mostly because it was fun.
 * 
 * This code works on Linux with a FTDI 232R based serial adapter.  I cannot
 * promise it works on anything else - you may have to find your own way to set
 * 128,000 baud, even parity, 8 data bits.  It's a really weird rate.
 * 
 * The data frame coming across the wire (once you get the baud and parity
 * correct) is a 15 byte frame, 8 bit bytes.  Each segment is sent with the
 * highest byte first, if it is a multi-byte value.
 * 
 * 0-1: aa 6a: Magic.  This is the start-of-frame marker that will be found in
 *             all frames.  If you're getting a 3a5a prefix, you probably have
 *             your baud set to 115200.  That's not right.
 * 2-3: 00 75: Current set, in mA.  The current set is calculated by taking this
 *             value, subtracting 17 (0x11), then multiplying by 10.  The lowest
 *             set value is 00 16, corresponding to 50mA.  The highest set value
 *             is 01 6f, corresponding to 3500mA.
 * 4-5: 10 12: Battery voltage.  This is the current battery voltage in mV,
 *             offset by 0x200.  A value of 0x200 corresponds to 0V, a value of
 *             0x0db8 corresponds to 3.000V, 0x11a0 corresponds to 4.000V.
 * 6-7: 0b b8: Termination voltage in mV.  0b b8 corresponds to 3.000V.
 * 8-10: 00 00 00: mAh through the device.  This corresponds directly to the
 *                 reported mAh on the display, with byte 10 being the LSB.
 * 11-12: 00 14: Internal resistance, in mΩ, offset by 20 (0x14).  A value of
 *               00 14 corresponds to 0 (not yet calculated), 00 3c corresponds
 *               to 40mΩ, etc.
 * 13: 01: State.  01: Running.  02: Stopped.  03: Finished (terminated due to
 *         termination voltage cutoff being reached).
 * 14: ac: End-of-frame marker.  Always 0xac.
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <termios.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

/**
 * Magic bytes.  The first two bytes are always 0xaa, 0x6a.  The last byte is
 * always 0xac.  And, the proper length is 15 bytes.
 * 
 * If you're randomly missing bytes, you probably don't have your hardware and
 * kernel properly configured to pass raw binary data.  XON/XOFF get swallowed
 * by most kernels, and you can have missing CR/NL values (or, worse, mutated
 * values - you can set the port to convert newlines to CRs or the other way
 * around).  If you get weird values, check all that stuff.
 */
#define MAGIC0 0xaa
#define MAGIC1 0x6a
#define MAGIC_TERM 0xac
#define EXPECTED_LENGTH 15

// Define this to enable the hex output with reverse text on changed bytes.
//#define DEBUG_HEX_OUTPUT

/**
 * Each element of the data field that is extracted as the proper integer.  This
 * means mA for current, mV for voltage, mAh for capacity, and mΩ for internal
 * resistance.  State is the raw value - 1, 2, or 3.
 */
typedef struct {
    uint32_t current_ma;
    uint32_t battery_mv;
    uint32_t termination_mv;
    uint32_t mah_total;
    uint32_t ir_mohms;
    uint8_t state_raw;
    uint8_t frame_is_valid;
} tec06_frame;

// The three states as string values for printing.
char state_str[4][10] = {
    "UNKNOWN",
    "Running",
    "Stopped",
    "Finished",
};

/**
 * Decode read bytes from the serial port into a TEC06 frame.  This function
 * simply takes a buffer and length to allow flexibility with how it is fed.
 * 
 * frame.frame_is_valid will be set to 1 (true) if the frame has been
 * successfully decoded.  If not, this will be 0 and the frame is empty.  This
 * is common on the first data fed in, but if it happens consistently, you are
 * probably swallowing data bytes with your port configuration.
 * 
 * If print_errors is true, this function will print out error messages.  If
 * false, it will simply return without populating the frame.
 */
tec06_frame decode_buffer(const uint8_t *buf, const size_t bytes_read, const
    uint8_t print_errors) {
    tec06_frame frame;
    uint32_t temp;
    
    memset(&frame, 0, sizeof(frame));
    
    // Check the 
    if (bytes_read != EXPECTED_LENGTH) {
        if (print_errors) {
            printf("ERROR: Line length invalid, got %lu, expected %u\n", 
                    bytes_read, EXPECTED_LENGTH);
        }
        return frame;
    }

    /**
     * If the buffer length is not 15, this code will not execute in the normal
     * architectural realm.  It will probably speculatively execute and touch
     * cache lines, but, hey, nobody cares about that, right?  No, really, this
     * code is probably not safe to actually run.  Just like all other code.
     */
    if ((buf[0] != MAGIC0) || (buf[1]) != MAGIC1 || (buf[14] != MAGIC_TERM)) {
        if (print_errors) {
            printf("ERROR: Unexpected magic values %02x %02x %02x\n",
                    buf[0], buf[1], buf[14]);
        }
        return frame;
    }

    /**
     * Current set value, in mA, is (10 * (value - 17))mA.  No idea why it's
     * done like this, but that's how you do it.  The current is stored in two
     * of the bytes - 2 and 3.
     */
    temp = ((uint32_t)buf[2] << 8) | buf[3];
    temp = temp - 17;
    frame.current_ma = temp * 10;

    /**
     * Battery voltage, in mV, is bytes 4 and 5, offset by 0x200.
     */
    temp = ((uint32_t)buf[4] << 8) | buf[5];
    temp -= 0x200;
    frame.battery_mv = temp;

    /**
     * Termination voltage is easy - this is just the actual setting in mV!
     */
    frame.termination_mv = ((uint32_t)buf[6] << 8) | buf[7];

    /**
     * mAh is three bytes, to allow for readings above 65Ah.  It's also quite
     * straightforward and requires no translation.
     */
    frame.mah_total =
            ((uint32_t)buf[8] << 16) | ((uint32_t)buf[9] << 8) | buf[10];

    /**
     * mOhm is offset by 20.
     */
    temp = ((uint32_t)buf[11] << 8) | buf[12];
    frame.ir_mohms = temp - 20;
    

    frame.state_raw = buf[13];
   
    // Force state to "UNKNOWN" if it is not in {1, 2, 3}.
    if ((frame.state_raw == 0) || (frame.state_raw > 3)) {
        frame.state_raw = 0;
    }
    
    // Mark the frame as valid.
    frame.frame_is_valid = 1;

    return frame;
}

void print_frame_readable(const tec06_frame frame) {
    if (frame.frame_is_valid) {
        printf("%0.3f V -> %0.3fV, %u mA, %u mAh, %u mΩ  %s\n",
                (float)frame.battery_mv / 1000.0,
                (float)frame.termination_mv / 1000.0,
                frame.current_ma,
                frame.mah_total,
                frame.ir_mohms,
                state_str[frame.state_raw]);
    }
}

/**
 * Prints the current timestamp in seconds, then:
 * battery_mv, termination_mv, current_ma, mah_total, ir_mohms, state_raw
 */ 
void print_frame_csv(const tec06_frame frame) {
    if (frame.frame_is_valid) {
        printf("%lu, %u, %u, %u, %u, %u, %u\n",
                (uint64_t)time(NULL), 
                frame.battery_mv,
                frame.termination_mv,
                frame.current_ma,
                frame.mah_total,
                frame.ir_mohms,
                frame.state_raw);
    }
}

/**
 * Read this: https://www.cmrr.umn.edu/~strupp/serial.html
 * 
 * That covers a lot of serial port configuration in far greater detail than
 * I'm about to.
 */
int main(int argc, char *argv[]) {
    int serial;
    struct serial_struct port_info;
    struct termios options;
    int use_csv = 0;
    uint8_t new_buffer[32];
#ifdef DEBUG_HEX_OUTPUT
    uint8_t old_buffer[32] = {0};
#endif
    
    if ((argc != 2) && (argc != 3)) {
        printf("Use: %s [serial port] [1 to use CSV]\n", argv[0]);
        exit(1);
    }
    
    serial = open(argv[1], O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial < 0) {
        printf("Cannot open serial port.\n");
        exit(1);
    }
    
    if ((argc == 3) && (atoi(argv[2]) == 1)) {
        use_csv = 1;
    }

    /**
     * Setting a custom baud rate, at least on the FTDI devices, is done by
     * configuring the custom divisor based on the baud rate, setting a few
     * flags, and then configuring the port to run at 38400 baud.  We do that
     * here.
     * 
     * https://sourceforge.net/p/ftdi-usb-sio/mailman/message/6501625/
     */
    
    // Load port_info with the port configuration.
    if (ioctl(serial, TIOCGSERIAL, &port_info) < 0) {
        printf("Cannot get serial port info.\n");
        exit(1);
    }

    // Toggle the proper flags, and calculate the divisor based on the existing
    // baud_base rate.  It'll be close enough.
    port_info.flags &= ~ASYNC_SPD_MASK;
    port_info.flags |= ASYNC_SPD_CUST;
    port_info.custom_divisor = port_info.baud_base / 128000;

    // Write back the new port configuration.
    if (ioctl(serial, TIOCSSERIAL, &port_info) < 0) {
        printf("Cannot set serial port info.\n");
        exit(1);
    }

    
    // Ensure the port is configured for blocking reads (only return after
    // there is data present).  We configure this more accurately later.
    fcntl(serial, F_SETFL, 0);
    

    // Get the serial port options for further configuration.
    tcgetattr(serial, &options);
    /**
     * "The c_cflag member contains two options that should always be enabled, 
     * CLOCAL and CREAD. These will ensure that your program does not become the
     * 'owner' of the port subject to sporatic job control and hangup signals, 
     * and also that the serial interface driver will read incoming data bytes."
     */
    options.c_cflag |= (CLOCAL | CREAD);
    // Enable parity.
    options.c_cflag |= PARENB;
    // Parity is not odd (even).
    options.c_cflag &= ~PARODD;
    // 8 data bits.
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    /**
     * Normal serial ports are configured for ASCII data, and make use of some
     * of the various control bytes to control the port.  The port may also
     * remap CR/NL as requested.  This is NO GOOD when you want the raw binary
     * data coming across the port.  The next few lines configure the port to
     * simply pass the raw binary data.
     */

    // Disable software flow control (this swallows DC1 and DC3 by default,
    // or 0x17 and 0x19)
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    // Don't remap newline to CR or CR to newline.  This is byte mutation!
    options.c_iflag &= ~(INLCR | ICRNL);
    
    // Raw mode - I'm dealing with binary data, not newline terminated ASCII.
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // Configure a half second timeout on port reads, with a minimum delivery
    // of 15 bytes.  This works well with how the TEC06 transmits and means that
    // I can receive a whole line of data at once.
    options.c_cc[VMIN]  = 15;
    options.c_cc[VTIME] = 5;   

    // To use the above-configured baud rate, set the port to 38400 baud.
    cfsetispeed(&options, B38400);
    cfsetospeed(&options, B38400);
    
    // Write everything back.
    tcsetattr(serial, TCSANOW, &options);
    
    /**
     * Main loop.  Read bytes from the serial port.  Once it finds sync, it
     * should consistently return 15 bytes at a time (the first read may have
     * random offsets).  The VMIN/VTIME settings above make this work nicely,
     * so don't change those.  Then work with the data in a reasonable manner.
     */
    while (1) {
        int bytes_read;
        
        bytes_read = read(serial, new_buffer, sizeof(new_buffer));
        if (bytes_read != -1) {
#ifdef DEBUG_HEX_OUTPUT
            for (int i = 0; i < bytes_read; i++) {
                if (new_buffer[i] == old_buffer[i]) {
                    printf("%02x ", new_buffer[i]);
                } else {
                    printf("\033[7m%02x\033[m ", new_buffer[i]);
                }
                old_buffer[i] = new_buffer[i];
            }
            printf("\n");
#endif
            if (use_csv) {
                // CSV won't print frame errors.
                print_frame_csv(decode_buffer(new_buffer, bytes_read, 0));
            } else {
                print_frame_readable(decode_buffer(new_buffer, bytes_read, 1));
            }
        }
    }
    
    // Close the serial port before termination - this should happen anyway.
    close(serial);
    
    return 0;
}