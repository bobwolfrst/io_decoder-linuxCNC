/********************************************************************
 * Description:  io_decoder.c
 * This file, 'io_decoder.c', is a HAL component that
 * connect an ARDUINO based board with USB communication to drive an input/output system
 * based on NPIC6C596 ICs, encoder, DAC and ADC to menage a CNC operator panel.
 *
 * Author: bobwolf
 * License: GPL Version 2
 *
 * Copyright (c) 2024 All rights reserved.
 *
 * 29/12/2025 tested on MICRO
 ********************************************************************/
/*
 * This HAL component for LINUXCNC is designed to interface with an Arduino-based board
 * via USB serial communication. The primary goal is to manage a CNC operator panel,
 * delegating to the Arduino all sequences for reading and writing registers, encoders and converters,
 * thus keeping the HAL component "light" and focused on communication.
 *
 * Main Features:
 *
 * Delegated I/O Management: The component only sends commands and receives data from the Arduino.
 *     The scanning logic, encoder handling, and the read/write sequences for DAC/ADC
 *     are entirely managed by the firmware on the Arduino, freeing LinuxCNC from these low-latency operations.
 *
 * Communication Protocol: The system relies on a USB serial protocol that efficiently
 *     handles packet exchange between LinuxCNC and the Arduino.
 *
 * output: through the component's output pins it transfers the signal to the physical outputs on the USB board,
 *     with the ability to blink each output independently at different frequencies.
 *
 * input: the physical inputs of the USB board are sent to the HAL component and, using a semi-debounce period
 *     equal to one USB communication period (typically 20ms), the signal is reflected on the HAL pin.
 *
 * Encoder Reading: Receives quadrature encoder data; counts and directions are handled
 *     directly by the Arduino firmware and it reports the delta since the last communication (typically 20ms).
 *     If the delta is 0 it sends 0 on the serial. Besides inversion of the count, the component generates
 *     up/down pulse sequences to drive HAL components such as MULTISWITCH or UPDOWN.
 *
 * ADC Handling: receives data from ADCs. Provides a HAL pin with the raw value received, a pin with the filtered float value,
 *     and pins to invert or scale the value.
 *
 * DAC Handling: Sends values to DACs. Transmits the value transformed by invert and scale pins.
 *
 * Handshake and State: The component implements an initial handshake protocol to verify firmware compatibility
 *     and to exchange the USB board capability data. Then it starts ordinary communication.
 *     It also maintains a communication state (Handshake, Communicating, Error) exposed as a HAL pin
 *     for diagnostics, useful to identify cable or disconnection issues.
 *     In case of communication timeout with the USB board an error is reported and the system returns to handshake.
 *     This error can be caused by a USB board malfunction, a physical USB disconnection, or an incorrect port setting.
 *     If the cable is reconnected the system reports the situation through an error message (used anomalously but useful for troubleshooting).
 *     If the USB cable suffers too much interference or is of poor quality the system emits error messages
 *     indicating the percentage of faulty messages; if this value is between 1% and 5% a warning is printed.
 *     If it exceeds 5% an error is printed and the system goes to error state and restarts serial communication from zero.
 *     All the error messages described above are visible in the LINUXCNC GUI if enabled in the component's
 *     initial configuration in the machine's .hal file.
 *
 * Overview of Operation:
 *
 * Initial Handshake: On startup the component connects to the Arduino and exchanges data to confirm that
 *     the firmware version (specified as a module parameter) matches the expected one.
 *
 * Communication Cycle: Once connected, the component sends a data packet to the Arduino
 *     containing output values and receives a response packet with input data
 *     (encoders, switches, ADCs).
 *
 * Communication Thread: A separate non-realtime thread, created internally by the component,
 *     handles USB communication to avoid blocking LinuxCNC's realtime loop,
 *     improving system reliability.
 *
 * Module Parameters:
 * input: number of inputs installed on the USB board. These can be expanded up to
 *     a maximum of 128 inputs (16 expansion boards). default 8.
 *
 * output: number of outputs installed on the USB board. These can be expanded up to
 *     a maximum of 128 outputs (16 expansion boards). default 8.
 *
 * firmware: Firmware version on the Arduino hardware (must match). default 101.
 *
 * usb_port_name: Name of the serial USB port default "/dev/io_decoder".
 *
 * verbose: to enable the level of error messages on the GUI. the number activates the indicated message type
 *     and all lower ones. default 1.
 *         0=none.
 *         1=only USB. Sends message on USB disconnection and on communication restart.
 *         2=minimal. Parsing error percentage messages.
 *         3=all.
 *
 *
 */

/* Standard Linux Includes */
#include "rtapi.h"     // RTAPI realtime OS API
#include "rtapi_app.h" // RTAPI realtime module decls
#include "hal.h"       // HAL public API decls
// #include "rtapi_print.h"
// #include "rtapi_thread.h" // ADDED: For rtapi_id_t, rtapi_thread_new, etc.
#include <float.h>
#include <rtapi_math.h>
#include <rtapi_string.h>
#include <stdbool.h>
#include <termios.h>   // For serial port configuration
#include <fcntl.h>     // For open(), close()
#include <unistd.h>    // For read(), write()
#include <errno.h>     // For error handling
#include <sys/ioctl.h> // For ioctl (optional, for flow control)
#include <pthread.h>   // For thread management (if not using rtapi_start_thread)
#include <stdio.h>     // For snprintf
#include <string.h>    // For memset
#include <ctype.h>     // For isprint
#include <time.h>      // for printing time

#include <linux/uinput.h>
#include <stdlib.h>   // For atoi, malloc, free, etc.
#include <sys/stat.h> // For chmod()

// IDE compatibility stubs: if IntelliSense non trova gli header RTAPI/LinuxCNC
// definiamo macro vuote per evitare falsi errori nel parser dell'editor.
#ifndef MODULE_AUTHOR
#define MODULE_AUTHOR(x)
#define MODULE_DESCRIPTION(x)
#define MODULE_LICENSE(x)
#endif

#ifndef RTAPI_MP_INT
#define RTAPI_MP_INT(name, desc)
#endif
#ifndef RTAPI_MP_STRING
#define RTAPI_MP_STRING(name, desc)
#endif

#ifndef RTAPI_NO_FP
#define RTAPI_NO_FP 0
#endif

// void rtapi_delay_ns(long ns);

// module information
MODULE_AUTHOR("bobwolf");
MODULE_DESCRIPTION("NPIC6C596 input output USB driver Component for EMC HAL");
MODULE_LICENSE("GPL");

static int input = 8;                                     // Default value
static int output = 8;                                    // Default value
static char *usb_port_name = "/dev/io_decoder";           // Default value for 'usb_port_name'
static int firmware = 101;                                // Default value
static int verbose = 1;                                   // Default value
static char *keymap_file = "io_decoder-keymap.cfg";       // Default value for the keyboard configuration file
static char *uinput_chmod_cmd = "chmod 0666 /dev/uinput"; // Default value to open UINPUT permissions to everyone

RTAPI_MP_INT(input, "number input channels");
RTAPI_MP_INT(output, "number output channels");
RTAPI_MP_STRING(usb_port_name, "USB serial port name (e.g., /dev/io_decoder)");
RTAPI_MP_INT(firmware, "Firmware of the hardware; must match to go on"); // to be compared with handshake
RTAPI_MP_INT(verbose, "type of error messeges");                         // 0=none. 1=all. 3=minimal. 4=only USB
RTAPI_MP_STRING(keymap_file, "description file to set the inputs versus keyboard commands");
RTAPI_MP_STRING(uinput_chmod_cmd, "Command used to set permissions on /dev/uinput (e.g., 'chmod 0660 /dev/uinput')");

#define DEBUG 0                      // 0=nothing 1=call usb 2=usb and communication 3=
static int printed_debug[100] = {0}; // used to limit repeated debug prints

#define MAX_CHAN 256              // maximum total npic expected
#define MIN_CHAN 16               // minimum total npic expected
#define MAX_INPUT 128             // maximum input npic expected
#define MAX_OUTPUT 128            // maximum output npic expected
#define MAX_N_ENCODER 6           // maximum expected
#define MAX_BIT_ENCODER 16        // maximum bits per encoder
#define MAX_ADC 6                 // maximum expected
#define MAX_BIT_ADC 16            // maximum bits per ADC
#define MAX_DAC 6                 // maximum expected
#define MAX_BIT_DAC 16            // maximum bits per DAC
#define MAX_IN_BIT_EXPANSION 128  // maximum bits per IN EXPANSION
#define MAX_OUT_BIT_EXPANSION 128 // maximum bits per OUT EXPANSION

// Payload sizes will use these maxima to allocate buffers
// These are calculations for maximum sizes of communication buffers
#define MAX_TX_PAYLOAD_SIZE ((MAX_OUTPUT / 8) + (MAX_DAC * ((MAX_BIT_DAC + 7) / 8)) + ((MAX_OUT_BIT_EXPANSION + 7) / 8)) + 3
#define MAX_RX_PAYLOAD_SIZE ((MAX_INPUT / 8) + (MAX_N_ENCODER * ((MAX_BIT_ENCODER + 7) / 8)) + (MAX_ADC * ((MAX_BIT_ADC + 7) / 8)) + ((MAX_OUT_BIT_EXPANSION + 7) / 8)) + 3

// USB control bytes
#define COMM_INIT 0x02
#define COMM_END 0x03
#define HAND_INIT 0x04
#define HAND_END 0x05
#define ERROR_MSG 0x06

// HAL USB protocol states
typedef enum
{
    HAL_STATE_HANDSHAKE = 0,
    HAL_STATE_HANDSHAKE_WAIT_RESPONSE = 1,
    HAL_STATE_COMMUNICATING = 2,
    HAL_STATE_COMMUNICATING_WAIT_RESPONSE = 3,
    HAL_STATE_ERROR = 4,

    VERBOSE_NULL = 0,
    VERBOSE_USB = 1,
    VERBOSE_MIN = 2,
    VERBOSE_ALL = 3,

} hal_comm_state_t;

// ADDED: Definitions for USB thread priority and stack
// #define USB_THREAD_PRIORITY rtapi_prio_lowest() // You can choose an appropriate priority
#define USB_THREAD_PRIORITY 50     // You can choose an appropriate priority
#define USB_THREAD_STACKSIZE 32768 // Stack size (may need adjustment)

// to filter ADC value and stabilize the output. smaller = more smoothing
#define ALPHA 0.1

// Helper functions for serial communication
static int serial_write(int fd, const unsigned char *buf, int len);
static int serial_read(int fd, unsigned char *buf, int len);

#define MAX_INPUT_2 (MAX_INPUT * 2) // for keyboard functionality with 2 keys
static int in_bit_d[MAX_INPUT];
static int in_bit_t_state[MAX_INPUT];
static uint32_t in_bit_keycode_value[MAX_INPUT_2];
static int keycode_number = 0;
static int hal_shutdown_flag = 0;
static int state_flag = 0;
volatile int error_message;
static long next_key_attempt_time_ns = 0;

static int keyboard_initialized_ok = 0;
static int shift_count = 0;
static int ctrl_count = 0;
static int l_alt_count = 0;
static int r_alt_count = 0;

/***********************************************************************
 * STRUCTURES AND GLOBAL VARIABLES                       *
 ************************************************************************/

//* This structure contains the runtime data for a single io_decoder.

// Keyboard functionality definitions *******************************************
// Type definitions for key_type (very useful!)
#define KEY_TYPE_UNUSED 0
#define KEY_TYPE_MODIFIER 1
#define KEY_TYPE_BOTH 2
#define KEY_TYPE_NORMAL 3

// #define KEY_DOWN 1
// #define KEY_UP 0

#define KEY_SCAN_DELAY_NS 100000000 // 100 ms
// ...

// static char keymap_filepath[512] = {0};

typedef struct
{
    // 1. Current State (Read from HAL pin)
    uint8_t current_state;

    // 2. Previous State (To detect transitions)
    uint8_t previous_state;

    // 3. Key Type (Modifier or Normal or Both)
    uint8_t key_type;

    // 4. Keysym Value (Numeric keyboard code, requires 32 bit)
    uint32_t keycode_value;

    uint32_t modifier_value;

    // 6. to delay the synchronized modified command
    uint8_t both_loop;

} PinData_t;

// One-dimensional array of structures
PinData_t key_input_map[MAX_INPUT];

//****************************************************************************

typedef struct
{

    hal_bit_t *usb_connected;
    hal_float_t *debug_f;
    hal_float_t *debug_f2;

    hal_s32_t *servo_thread_time; // input for debug
    hal_s32_t *base_thread_time;  // input for debug

    hal_s32_t *servo_jitter_pin;
    hal_s32_t *base_jitter_pin;

    hal_s32_t *jitter_usb_comm; // input for debug
    hal_s32_t *jitter_usb_loop; // input for debug

    hal_bit_t *in_bit[MAX_INPUT];
    hal_bit_t *in_bit_t[MAX_INPUT];

    hal_bit_t *out_bit[MAX_OUTPUT];
    hal_bit_t *out_bit_blink_en[MAX_OUTPUT];
    hal_float_t *out_bit_blink_freq[MAX_OUTPUT];
    hal_float_t *out_bit_blink_width[MAX_OUTPUT];
    long out_blink_time_ns[MAX_OUTPUT];

    hal_s32_t *enc[MAX_N_ENCODER];         // For encoder counters
    hal_bit_t *enc_invert[MAX_N_ENCODER];  // For inversion
    hal_bit_t *enc_up[MAX_N_ENCODER];      // For up pins (if using pulses)
    hal_bit_t *enc_down[MAX_N_ENCODER];    // FIXED: Typo MAX_N_ENCODERS -> MAX_N_ENCODER
    uint8_t enc_delta[MAX_N_ENCODER];      // to manage enc_up enc_down pulses
    uint8_t enc_delta_sign[MAX_N_ENCODER]; // to manage enc_up enc_down pulses

    hal_float_t *adc[MAX_ADC];              // For ADC values
    hal_float_t *adc_scale[MAX_ADC];        // To scale each ADC value
    hal_bit_t *adc_invert[MAX_ADC];         // For inversion
    hal_u32_t *adc_raw[MAX_ADC];            // For raw data
    uint32_t old_adc_raw[MAX_ADC];          // For raw data
    hal_float_t *adc_joy_center[MAX_ADC];   // For joystick center offset
    hal_float_t *adc_joy_deadb[MAX_ADC];    // For joystick deadband
    hal_float_t *adc_joy_factor[MAX_ADC];   // For joystick sensitivity
    hal_bit_t *adc_joy_pulse_up[MAX_ADC];   // For up pulse driven by potentiometer
    hal_bit_t *adc_joy_pulse_down[MAX_ADC]; // For down pulse driven by potentiometer
    hal_s32_t *adc_joy_count[MAX_ADC];      // For pulse counting
    uint32_t adc_raw_joy[MAX_ADC];          // To provide raw pulse data also inverted
    long next_adc_joy_ns[MAX_ADC];
    uint8_t adc_joy_valid[MAX_ADC]; //

    hal_float_t *dac[MAX_DAC];       // For DAC values
    hal_float_t *dac_scale[MAX_DAC]; // To scale each DAC value
    hal_bit_t *dac_invert[MAX_DAC];  // For inversion

    hal_bit_t *out_expansion[MAX_OUT_BIT_EXPANSION]; // For expansion output bits
    hal_bit_t *in_expansion[MAX_IN_BIT_EXPANSION];   // For expansion input bits

    // New variables for USB communication
    char usb_port_name[HAL_NAME_LEN]; // Serial USB port name (e.g. /dev/io_decoder)
    int serial_fd;                    // Serial port file descriptor
    int baud_rate;                    // Serial port baud rate
    int usb_thread_id;                // MODIFIED: rtapi_id_t doesn't exist, use int

    /* runtime: 1 while usb thread running, 0 when exiting */
    volatile int usb_thread_running;

    // Buffers for data exchange between servo-thread and usb-thread
    // These will contain only the *payload* without header/footer/checksum
    uint8_t usb_tx_payload[MAX_TX_PAYLOAD_SIZE];
    uint8_t usb_rx_payload[MAX_RX_PAYLOAD_SIZE];

    uint32_t usb_tx_payload_len; // Actual payload length in tx buffer
    uint32_t usb_rx_payload_len; // Actual payload length in rx buffer

    unsigned char serial_rx_buffer[MAX_RX_PAYLOAD_SIZE];
    int serial_rx_buffer_len;
    unsigned char tx_packet_buffer[MAX_TX_PAYLOAD_SIZE];

    // definitions to set firmware 101
    /*
    #define FIRMWARE    101
    #define N_ENCODER   2     //then 4
    #define N_DAC       2
    #define N_ADC       3
    #define IN_BIT_EXPANSION    8
    #define OUT_BIT_EXPANSION   8*/
    uint8_t firmware_n_encoder;
    uint8_t firmware_n_dac;
    uint8_t firmware_n_adc;
    uint8_t firmware_in_bit_expansion;
    uint8_t firmware_out_bit_expansion;
    uint8_t firmware_firmware;

    // Data received from Arduino handshake (for payload sizes)
    uint8_t arduino_byte_npic_input;  // helper variable
    uint8_t arduino_byte_npic_output; // helper variable
    uint8_t arduino_n_encoder;
    uint8_t arduino_bit_encoder;
    uint8_t arduino_byte_encoder;  // helper variable
    uint8_t arduino_bytes_encoder; // helper variable
    uint8_t arduino_n_dac;
    uint8_t arduino_bit_dac;
    uint8_t arduino_byte_dac;  // helper variable
    uint8_t arduino_bytes_dac; // helper variable
    uint8_t arduino_n_adc;
    uint8_t arduino_bit_adc;
    uint8_t arduino_byte_adc;  // helper variable
    uint8_t arduino_bytes_adc; // helper variable
    uint8_t arduino_in_bit_expansion;
    uint8_t arduino_in_byte_expansion; // helper variable
    uint8_t arduino_out_bit_expansion;
    uint8_t arduino_out_byte_expansion; // helper variable
    uint8_t arduino_loop_time;
    uint8_t arduino_firmware;

    // MODULE PARAMETERS (now members of the struct for access via 'io->')
    int input;    // Number of NPIC inputs requested (copied from module parameter)
    int output;   // Number of NPIC outputs requested (copied from module parameter)
    int firmware; // Expected firmware version (copied from module parameter)
    int verbose;  // Error message verbosity level
    // char keymap_file[512];

    // State variables for the protocol
    hal_comm_state_t comm_state;         // Current communication state (handshake/communicating)
    int comm_error_count;                // Consecutive error counter
    hal_s32_t *hal_comm_state;           // HAL pin for communication state (for GUI)
    hal_float_t *hal_error_count;        // HAL pin for error counter (for GUI)
    hal_float_t *parse_error_percentile; // HAL pin for parse error percentage (for GUI)
    hal_bit_t *led;                      // pin: output led for checks

    // Variables for the timing of the non-realtime USB thread
    long usb_last_read_time_ns; // Timestamp of last USB read/write
    long usb_mini_timeout_ns;   // Timestamp for mini-timeout
    long usb_timeout_ns;        // Timeout for USB communication
    long loop_time_ns;
    volatile int usb_data_busy; // semaphore to avoid simultaneous data reads between threads

    // variables to block undesired refreshes of values inside functions
    volatile int encoder_refresh;
    volatile int in_npic_refresh;
    volatile int out_npic_refresh;
    volatile int adc_refresh;
    volatile int dac_refresh;
    volatile int in_exp_refresh;
    volatile int out_exp_refresh;

    // to implement the simulated keyboard functionality
    int uinput_fd;

} hal_io_decoder_t;

// pointer to array of io_decoder_t structs in shared memory, 1 per gen
static hal_io_decoder_t *io_decoder_array;

// other globals
static int comp_id; // component ID

/*******************************************************************************************************************
 * LOCAL FUNCTION DECLARATIONS                         *
 ********************************************************************************************************************/

static int export_io_decoder(int num, hal_io_decoder_t *addr, char *prefix);
static void calc_io_decoder(void *arg, long period);
/* forward declaration: usb thread calls this non-RT helper */
static void key_simulation_loop(void *arg);

/* ensure hal_free prototype is visible to avoid implicit declaration warning */
extern void hal_free(void *ptr);

//***********************************************************************
// Helper function to configure the serial port
// Helper function to configure the serial port (Version for 32u4/Leonardo)
int configure_serial_port(int fd, int baud_rate)
{
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Error tcgetattr\n");
        return -1;
    }

    // Set the speed (even if on Leonardo/USB it's virtual, it's better to set it)
    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);

    // --- FULL RAW MODE (Required for Leonardo) ---
    // Disable parity, stop bits and force 8 data bits
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    // CLOCAL: ignore modem control lines, CREAD: enable receiver
    tty.c_cflag |= (CLOCAL | CREAD);

    // Disable all input processing (no translation of \r or \n)
    // This prevents bytes 0x00, 0x0A, 0x0D from being discarded or modified
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);

    // Disable all output processing (essential for sending binary payloads)
    tty.c_oflag &= ~OPOST;

    // Disable canonical mode, echo and signals
    tty.c_lflag &= ~(ICANON | ECHO | ECHONL | ISIG | IEXTEN);

    // Timeout settings: non-blocking read if using O_NONBLOCK afterwards
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10; // 1 second timeout for tcsetattr

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Error tcsetattr\n");
        return -1;
    }

    // --- ACTIVATE DTR/RTS (Opens Leonardo data channel) ---
    int status;
    if (ioctl(fd, TIOCMGET, &status) == -1)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Error TIOCMGET\n");
    }
    else
    {
        status |= TIOCM_DTR;
        status |= TIOCM_RTS;
        if (ioctl(fd, TIOCMSET, &status) == -1)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Error TIOCMSET\n");
        }
    }

    // Short wait to allow firmware (without bootloader) to stabilize USB
    usleep(100000);

    // Flush buffers from any startup residues
    tcflush(fd, TCIOFLUSH);

    rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: Serial port configured in RAW mode (DTR/RTS ON)\n");
    return 0;
}

//***********************************************************************
// XOR Checksum Calculation Function
// Returns the checksum byte.
uint8_t calculate_checksum(const uint8_t *payload, int payload_len)
{
    uint8_t checksum = 0;
    for (int i = 0; i < payload_len; i++)
    {
        checksum ^= payload[i]; // Cumulative XOR of all payload bytes
    }
    return checksum;
}

// Packet Building Function for Transmission
// Builds the complete packet into the supplied 'buffer'.
// Returns the total length of the constructed packet, or a negative value on error.
int build_packet(uint8_t *buffer, int max_buffer_len, uint8_t start_byte, uint8_t end_byte, const uint8_t *payload, int payload_len)
{
    // Calculate minimum required length for the complete packet:
    // 1 (start_byte) + payload_len + 1 (checksum) + 1 (end_byte)
    int required_len = 1 + payload_len + 1 + 1;

    // Check if provided buffer is large enough
    if (max_buffer_len < required_len)
    {
        rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: ERROR: build_packet: Buffer too small (%d bytes needed, %d available).\n", required_len, max_buffer_len);
        return -1; // Error: buffer too small
    }

    // Calculate payload checksum
    uint8_t checksum = calculate_checksum(payload, payload_len);

    int current_idx = 0;

    // Add start byte
    buffer[current_idx++] = start_byte;

    // Copy payload into buffer
    memcpy(&buffer[current_idx], payload, payload_len);
    current_idx += payload_len;

    // Add checksum byte
    buffer[current_idx++] = checksum;

    // Add end byte
    buffer[current_idx++] = end_byte;

    return current_idx; // Return total length of constructed packet
}

int parse_packet(const uint8_t *buffer, int buffer_len, uint8_t expected_start_byte, uint8_t expected_end_byte, uint8_t *parsed_payload_buffer, int max_parsed_payload_len)
{

    int start_idx = -1;
    int end_idx = -1;
    uint8_t received_checksum;
    uint8_t calculated_checksum;
    error_message = 0;

    // 1. Find the end byte after the start byte
    for (int i = 0; i < buffer_len; i++)
    {
        if (buffer[i] == expected_end_byte)
        {
            end_idx = i;
            // break;
        }
    }

    if (end_idx == -1)
    {
        rtapi_print_msg(RTAPI_MSG_DBG, "io_decoder: DBG: parse_packet: End byte 0x%02X not found after start byte 0x%02X.\n", expected_end_byte, expected_start_byte);
        return 0; // End byte not found, incomplete packet
    }
    // 2. Find the start byte
    for (int i = 0; i < buffer_len; i++)
    {
        if (buffer[i] == expected_start_byte)
        {
            start_idx = i;
            break;
        }
        else if (buffer[i] == ERROR_MSG)
        {
            // i++;
            start_idx = i;
            error_message = 1;
            break;
        }
    }

    if (start_idx == -1)
    {
        rtapi_print_msg(RTAPI_MSG_DBG, "io_decoder: DBG: parse_packet: Start byte 0x%02X not found.\n", expected_start_byte);
        return 0; // Start byte not found, packet not complete or not present
    }

    // 3. Verify minimum packet length: start + payload + checksum + end (minimum 3 bytes if payload_len is 0)
    // i.e.: start_idx < (checksum_idx) < end_idx
    // end_idx - start_idx must be at least 3 (start_byte + checksum + end_byte)
    if (((end_idx - start_idx) < 3) && !error_message)
    {
        rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: ERROR: parse_packet: Packet too short. Start: %d, End: %d.\n", start_idx, end_idx);
        return -2; // Error: Packet too short
    }

    // 4. Extract received checksum (it's the byte before the end_byte)
    received_checksum = buffer[end_idx - 1];

    // 5. Compute payload length
    // The payload is between start_byte (excluded) and checksum (excluded)
    // int payload_len = (end_idx - 1) - (start_idx + 1);
    int payload_len = (end_idx - start_idx) - 2;

    // 6. Check if the output payload buffer is large enough
    if ((max_parsed_payload_len < payload_len) && !error_message)
    {
        rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: ERROR: parse_packet: Parsed payload buffer too small (%d needed, %d available) end_idx=%d. start_idx=%d.\n", payload_len, max_parsed_payload_len, end_idx, start_idx);
        return -3; // Error: output payload buffer too small
    }

    // 7. Calculate checksum of extracted payload
    // Payload starts immediately after start_byte
    calculated_checksum = calculate_checksum(&buffer[start_idx + 1], payload_len);

    // 8. Compare checksums
    if ((received_checksum != calculated_checksum) && !error_message)
    {
        rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: ERROR: parse_packet: Checksum mismatch! Received 0x%02X, Calculated 0x%02X.\n", received_checksum, calculated_checksum);
        return -4; // Error: Checksum mismatch
    }

    // 9. If everything is valid, copy payload to output buffer
    memcpy(parsed_payload_buffer, &buffer[start_idx + 1], payload_len);

    return payload_len; // Returns length of valid extracted payload
}

// Implementation of serial communication utility functions
static int serial_write(int fd, const unsigned char *buf, int len)
{
    int bytes_written = write(fd, buf, len);
    tcdrain(fd); // <--- THIS forces Linux to flush data to the USB cable now!
    if (bytes_written < 0)
    {
        rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: ERROR serial_write %s\n", strerror(errno));
    }
    return bytes_written;
}

static int serial_read(int fd, unsigned char *buf, int len)
{
    int bytes_read = read(fd, buf, len);
    if (bytes_read < 0)
    {
        // EAGAIN/EWOULDBLOCK are normal if no data is available and port is non-blocking
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
            rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: ERROR serial_read: %s\n", strerror(errno));
        }
    }
    return bytes_read;
}

// Helper to format a byte buffer into a hexadecimal string
// Returns a pointer to a static string. Use immediately,
// because the next call will overwrite the contents.
static char *format_buffer_for_debug(const unsigned char *buffer, int len)
{
// We use a static buffer. This limits output string length
// but avoids dynamic allocations and memory issues in RT environment.
// Buffer is 3 times the byte length + 1 for null terminator
// (e.g. "0xAA " uses ~5 chars per byte, so 5*len + 1. To be safe, 4*len + 1)
// If rx_buffer max is 64, then 64 * 5 = 320 + 1
#define MAX_DEBUG_STR_LEN (64 * 5 + 1) // Assuming rx_buffer max 64 bytes
    static char debug_str[MAX_DEBUG_STR_LEN];
    int current_pos = 0;

    debug_str[0] = '\0'; // Initialize empty string

    for (int i = 0; i < len; i++)
    {
        // Compute remaining space
        int remaining_space = MAX_DEBUG_STR_LEN - current_pos;
        if (remaining_space <= 5)
        { // 5 is minimum space for "0xXX "
            // No more space for next byte + space
            break;
        }
        // snprintf is safer than sprintf to avoid buffer overflow
        current_pos += snprintf(&debug_str[current_pos], remaining_space, "0x%02X ", buffer[i]);
    }
    return debug_str;
}

// Helper to format a byte buffer as ASCII string.
// Non-printable characters are replaced with a dot (.).
// Returns pointer to a static string. Use immediately,
// because the next call will overwrite it.
static char *format_as_ascii_for_debug(const unsigned char *buffer, int len)
{
// Use a static buffer. Must be as big as max byte length + 1
#define MAX_ASCII_STR_LEN (64 + 1) // Assuming rx_buffer max 64 bytes
    static char ascii_str[MAX_ASCII_STR_LEN];
    int current_pos = 0;

    // Ensure we don't exceed buffer size
    if (len >= MAX_ASCII_STR_LEN)
    {
        len = MAX_ASCII_STR_LEN - 1; // Truncate if needed
    }

    for (int i = 0; i < len; i++)
    {
        // isprint() checks if character is printable (ASCII 32-126)
        if (isprint(buffer[i]))
        {
            ascii_str[current_pos++] = (char)buffer[i];
        }
        else
        {
            ascii_str[current_pos++] = '.'; // Replace non-printables with dot
        }
    }
    ascii_str[current_pos] = '\0'; // Null-terminate

    return ascii_str;
}

// usb_thread_function************************************************************************************************
void usb_thread_function(void *arg)
{
    hal_io_decoder_t *io = (hal_io_decoder_t *)arg;

    io->usb_thread_running = 1;
    // Initial communication state setup
    *(io->hal_comm_state) = HAL_STATE_HANDSHAKE; // Always start in handshake state
    state_flag = HAL_STATE_HANDSHAKE;
    *(io->hal_error_count) = 0.0f;        // Zero the error counter at startup
    *(io->parse_error_percentile) = 0.0f; // Zero the parse error percentile at startup
    io->loop_time_ns = 20 * 1000 * 1000;  // Default to 20ms

    io->serial_fd = -1; // Initialize file descriptor to invalid
    io->encoder_refresh = 0;
    io->in_npic_refresh = 0;
    io->out_npic_refresh = 0;
    io->adc_refresh = 0;
    io->dac_refresh = 0;
    io->in_exp_refresh = 0;
    io->out_exp_refresh = 0;

    long current_time_ns;
    long next_handshake_attempt_time_ns = 0;                            // Next handshake attempt time
    long next_comm_attempt_time_ns = 0;                                 // Next communication attempt time
    long response_timeout_ns = 1000 * 1000 * 1000;                      // 1000ms response timeout
    long handshake_period_ns = 20 * 1000 * 1000;                        // 20ms handshake period
    static long last_read_time_ns;                                      // Initialize to current time
    long long parse_error_time_ns = (long long)30 * 1000 * 1000 * 1000; // 60s for parse error statistics
    long next_parse_error_time_ns;
    long loop_current_time_ns;
    long usb_current_time_ns;
    long usb2_current_time_ns;
    long usb_jitter_time_ns;

    static int timer_overflow_0 = 0;
    static int timer_overflow_1 = 0;
    static int timer_overflow_2 = 0;
    static int timer_overflow_3 = 0;
    static int timer_overflow_4 = 0;
    static int timer_overflow_5 = 0;
    // Declare all static variables at function start
    static int handshake_init_dbg_printed = 0;     // For HAL_STATE_HANDSHAKE_INIT
    static int handshake_response_dbg_printed = 0; // For HAL_STATE_HANDSHAKE_WAIT_RESPONSE
    static int comm_state_dbg_printed = 0;         // For HAL_STATE_COMMUNICATING
    static int comm_state_reenter_printed = 1;     // For HAL_STATE_COMMUNICATING
    static int error_state_dbg_printed = 0;        // For HAL_STATE_ERROR (first entry)
    static int error_state_serial_failed = 0;      // For serial not present
    static int error_state_serial_configure = 0;   // For serial not configured
    static int error_state_hand_packet = 0;        // For handshake packet length mismatch
    static int error_state_firmware = 0;           // For wrong firmware
    static int error_state_firmware_2 = 0;         // For wrong firmware function values
    static int error_state_payload = 0;            // For wrong handshake payload
    static int error_state_hand_serial_read = 0;
    ; // For serial read error during handshake
    static int error_state_comm_timeout = 0;

    // New flags to control packet processing during COMMUNICATING
    static int comm_packet_sent = 0;
    static int comm_packet_received_and_validated = 0; // To check loop inside COMMUNICATING
    static int comm_parse_error = 0;
    static int old_comm_parse_error = 0;
    static int max_parse_diff = 0;
    static int print_parse_error = 0;
    static int comm_valid_send = 0;
    static int old_comm_valid_send = 0;
    static int comm_valid_received = 0;
    static int old_comm_valid_received = 0;
    static int comm_invalid_send = 0;
    static int old_comm_invalid_send = 0;
    static int comm_invalid_received = 0;
    static int old_comm_invalid_received = 0;
    static int hand_valid_send = 0;
    static int old_hand_valid_send = 0;
    static int hand_valid_received = 0;
    static int old_hand_valid_received = 0;
    static int hand_invalid_send = 0;
    static int old_hand_invalid_send = 0;
    static int hand_invalid_received = 0;
    static int old_hand_invalid_received = 0;
    static int old_pin_debug2 = 0;

    static long max_servo_thread_time; // input for debug
    static long max_base_thread_time;  // input for debug
    static long min_servo_thread_time; // input for debug
    static long min_base_thread_time;

    static long old_usb_loop;
    static long old_usb_time;
    static long min_usb_loop;
    static long min_usb_time;
    static long max_usb_loop;
    static long max_usb_time;
    static uint8_t usb_time_ok;

    max_servo_thread_time = 0;
    min_servo_thread_time = 2147483647;
    max_base_thread_time = 0;
    min_base_thread_time = 2147483647;
    min_usb_loop = 2147483647;
    min_usb_time = 2147483647;
    max_usb_loop = 0;
    max_usb_time = 0;

    // Buffer for reading raw data from serial
    // Should be large enough for the largest packet including start/end/checksum
    uint8_t serial_rx_buffer[MAX_RX_PAYLOAD_SIZE + 32]; // Add some margin
    int serial_rx_buffer_len = 0;                       // Amount of data currently in buffer
    int serial_verify;
    static uint8_t tx_packet_buffer[MAX_TX_PAYLOAD_SIZE + 4]; // Buffer for complete packet
    static int packet_len;

    // variables to print the time
    time_t rawtime;
    struct tm *info_time;
    char time_buffer[80];

    // Main loop of the USB thread
    while (!hal_shutdown_flag)
    { // Use !hal_exit for a clean component shutdown

        current_time_ns = rtapi_get_time(); // Update current time at start of each cycle

        if (current_time_ns < 0)
        {
            // If overflow occurs, re-normalize to positive to avoid breaking calculations
            current_time_ns = current_time_ns & (~0x80000000); // Mask 32nd bit
            if (current_time_ns < next_comm_attempt_time_ns - handshake_period_ns)
            {                                                                      // first wrap
                next_comm_attempt_time_ns = current_time_ns + handshake_period_ns; // Schedule next attempt
            }
        }

        // --- Section 1: Open and Configure Serial Port ---
        // This logic runs only if the port is not yet open or was closed due to an error
        if (io->serial_fd < 0)
        {
            io->serial_fd = open(io->usb_port_name, O_RDWR | O_NOCTTY | O_NONBLOCK); // O_NONBLOCK crucial for non-blocking reads
            if (io->serial_fd < 0)
            {
                // Error opening port
                if (!error_state_serial_failed)
                {
                    if (verbose >= VERBOSE_USB)
                    {
                        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ERROR: \nFailed to open serial port %s: %s \nOR \ncable disconnected\n", io->usb_port_name, strerror(errno));
                    }
                    *(io->usb_connected) = 0;
                    error_state_serial_failed = 1;
                    *(io->hal_comm_state) = HAL_STATE_ERROR;
                }
                *(io->hal_comm_state) = HAL_STATE_ERROR; // Report error state in HAL
                usleep(500000);                          // Wait 500ms before retrying to open
                continue;                                // Skip to next loop to retry opening
            }

            // If opened successfully, proceed to configure
            // Use default baud_rate or module parameter if provided
            if (configure_serial_port(io->serial_fd, B115200) != 0)
            {                                            // Use B115200 as example
                *(io->hal_comm_state) = HAL_STATE_ERROR; // Report error state
                usleep(500000);                          // Wait 500ms
                comm_state_reenter_printed = 0;
                continue;
            }

            // Port opened and configured successfully. Reset state to HANDSHAKE.
            *(io->hal_comm_state) = HAL_STATE_HANDSHAKE;
            serial_rx_buffer_len = 0;                    // Clear receive buffer
            next_comm_attempt_time_ns = current_time_ns; // Prepare immediate handshake attempt
        }
        else
        {
            if (usb_time_ok == 1)
            {
                loop_current_time_ns = current_time_ns - old_usb_time;
                old_usb_time = current_time_ns;
                if (loop_current_time_ns > 1000000)
                {
                    if (loop_current_time_ns > max_usb_loop)
                    {
                        max_usb_loop = loop_current_time_ns;
                    }
                    if (loop_current_time_ns < min_usb_loop)
                    {
                        min_usb_loop = loop_current_time_ns;
                    }
                }
            }
        }

        if (*(io->base_thread_time) > 0)
        {
            if (*(io->base_thread_time) > max_base_thread_time)
            {
                max_base_thread_time = *(io->base_thread_time);
            }
            if (*(io->base_thread_time) < min_base_thread_time)
            {
                min_base_thread_time = *(io->base_thread_time);
            }
        }
        if (*(io->servo_thread_time) > 0)
        {
            if (*(io->servo_thread_time) > max_servo_thread_time)
            {
                max_servo_thread_time = *(io->servo_thread_time);
            }
            if (*(io->servo_thread_time) < min_servo_thread_time)
            {
                min_servo_thread_time = *(io->servo_thread_time);
            }
        }

        // --- Section 2: State Machine Logic ---
        switch (*(io->hal_comm_state))
        {
        case HAL_STATE_HANDSHAKE:
        {
            state_flag = HAL_STATE_HANDSHAKE;
            // Handle timeout for handshake attempt
            if (current_time_ns >= next_comm_attempt_time_ns)
            {
                usb_current_time_ns = rtapi_get_time();
                // Build handshake payload (HAL sends requested input/output NPIC)
                uint8_t handshake_payload_tx[2];
                handshake_payload_tx[0] = (uint8_t)io->output; // Number of output NPIC requested
                handshake_payload_tx[1] = (uint8_t)io->input;  // Number of input NPIC requested

                packet_len = build_packet(tx_packet_buffer, sizeof(tx_packet_buffer),
                                          HAND_INIT, HAND_END, handshake_payload_tx, sizeof(handshake_payload_tx));

                //					rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: MESSAGE IN HANDSHAKE (ASCII):\n '%s'\n", format_buffer_for_debug(tx_packet_buffer, packet_len));

                if (packet_len > 0)
                {
                    int bytes_written = serial_write(io->serial_fd, tx_packet_buffer, packet_len);
                    // tcdrain(io->serial_fd);          // <--- THIS forces Linux to flush data to the USB cable now!

                    if (bytes_written != packet_len)
                    {
                        *(io->hal_comm_state) = HAL_STATE_ERROR; // Or HAL_STATE_HANDSHAKE, depending on desired restart
                        hand_invalid_send++;
                    }
                    else
                    {
                        *(io->hal_comm_state) = HAL_STATE_HANDSHAKE_WAIT_RESPONSE;
                        io->usb_last_read_time_ns = current_time_ns; // Start response timeout
                        io->usb_mini_timeout_ns = current_time_ns;   // Start mini-timeout
                        hand_valid_send++;
                    }
                }
                else
                {
                }
                next_comm_attempt_time_ns = current_time_ns + handshake_period_ns; // Schedule next attempt
            }
            break;
        } // end case HAL_STATE_HANDSHAKE

        case HAL_STATE_HANDSHAKE_WAIT_RESPONSE:
        {
            state_flag = HAL_STATE_HANDSHAKE_WAIT_RESPONSE;
            // Read response from Arduino
            // Try to read data from serial (non-blocking)
            int bytes_read = read(io->serial_fd, serial_rx_buffer + serial_rx_buffer_len, sizeof(serial_rx_buffer) - serial_rx_buffer_len);
            if (bytes_read > 0)
            {
                serial_rx_buffer_len += bytes_read;

                // Try to parse a handshake response packet
                uint8_t handshake_payload_rx[10]; // Arduino responds with 9 bytes payload + 1 firmware byte
                int parsed_len = parse_packet(serial_rx_buffer, serial_rx_buffer_len,
                                              HAND_INIT, HAND_END, handshake_payload_rx, sizeof(handshake_payload_rx));
                if (error_message == 1)
                {
                    *(io->hal_comm_state) = HAL_STATE_HANDSHAKE;
                    if (verbose >= VERBOSE_ALL)
                    {
                        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ERROR MESSAGE IN HANDSHAKE (ASCII): '%s'\n", format_as_ascii_for_debug(serial_rx_buffer, serial_rx_buffer_len));
                    }
                }
                else
                {
                    if (parsed_len > 0)
                    {
                        // Handshake packet received and valid!
                        // Parse payload received from Arduino
                        if (parsed_len == 10)
                        { // Arduino should send 10 bytes payload (9 for config + 1 for firmware)
                            switch (firmware)
                            {
                            case 255:
                                // Set values specific to firmware 255
                                io->arduino_byte_npic_output = 1; // helper variable
                                io->arduino_byte_npic_input = 1;  // helper variable
                                break;

                            default:
                                io->arduino_byte_npic_output = output / 8; // helper variable
                                io->arduino_byte_npic_input = input / 8;   // helper variable
                            }
                            io->arduino_n_encoder = handshake_payload_rx[0];                              // payload
                            io->arduino_bit_encoder = handshake_payload_rx[1];                            // payload
                            io->arduino_byte_encoder = (io->arduino_bit_encoder + 7) / 8;                 // helper calculation
                            io->arduino_bytes_encoder = io->arduino_n_encoder * io->arduino_byte_encoder; // helper calculation
                            io->arduino_n_dac = handshake_payload_rx[2];                                  // payload
                            io->arduino_bit_dac = handshake_payload_rx[3];                                // payload
                            io->arduino_byte_dac = (io->arduino_bit_dac + 7) / 8;                         // helper calculation
                            io->arduino_bytes_dac = io->arduino_n_dac * io->arduino_byte_dac;             // helper calculation
                            io->arduino_n_adc = handshake_payload_rx[4];                                  // payload
                            io->arduino_bit_adc = handshake_payload_rx[5];                                // payload
                            io->arduino_byte_adc = (io->arduino_bit_adc + 7) / 8;                         // helper calculation
                            io->arduino_bytes_adc = io->arduino_n_adc * io->arduino_byte_adc;             // helper calculation
                            io->arduino_in_bit_expansion = handshake_payload_rx[6];                       // payload
                            io->arduino_in_byte_expansion = (io->arduino_in_bit_expansion + 7) / 8;       // helper calculation
                            io->arduino_out_bit_expansion = handshake_payload_rx[7];                      // payload
                            io->arduino_out_byte_expansion = (io->arduino_out_bit_expansion + 7) / 8;     // helper calculation
                            io->arduino_loop_time = handshake_payload_rx[8];                              // payload
                            io->arduino_firmware = handshake_payload_rx[9];                               // payload

                            io->usb_tx_payload_len = (io->arduino_byte_npic_output) +
                                                     (io->arduino_n_dac * io->arduino_byte_dac) + // DAC bytes
                                                     (io->arduino_out_byte_expansion);

                            io->usb_rx_payload_len = (io->arduino_byte_npic_input) +
                                                     (io->arduino_n_encoder * io->arduino_byte_encoder) + // Encoder bytes
                                                     (io->arduino_n_adc * io->arduino_byte_adc) +         // ADC bytes
                                                     (io->arduino_in_byte_expansion);

                            io->loop_time_ns = io->arduino_loop_time * 1000 * 1000;
                            if (io->loop_time_ns == 0)
                            {                                           // Avoid division by zero or infinite loop if Arduino sends 0
                                io->loop_time_ns = handshake_period_ns; // Default to 20ms
                            }

                            // Check firmware match
                            if (io->arduino_firmware != io->firmware)
                            {
                                *(io->hal_comm_state) = HAL_STATE_HANDSHAKE; // Go to error state
                                if (!comm_state_dbg_printed)
                                {
                                    comm_state_dbg_printed = 1;
                                    rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ERROR: firmware mismatch\nsoftware setup = %d\nhardware setup = %d\nchange the software value\ncomponent stopped\n", io->firmware, io->arduino_firmware);
                                }
                            }
                            else
                            {
                                if ((io->arduino_n_encoder != io->firmware_n_encoder) ||
                                    (io->arduino_n_dac != io->firmware_n_dac) ||
                                    (io->arduino_n_adc != io->firmware_n_adc) ||
                                    (io->arduino_in_bit_expansion != io->firmware_in_bit_expansion) ||
                                    (io->arduino_out_bit_expansion != io->firmware_out_bit_expansion))
                                {
                                    *(io->hal_comm_state) = HAL_STATE_HANDSHAKE; // Go to error state
                                }
                                else
                                {
                                    *(io->hal_comm_state) = HAL_STATE_COMMUNICATING; // Go to communicating state
                                    // comm_parse_error = 0;
                                    comm_state_dbg_printed = 0;
                                    io->usb_last_read_time_ns = current_time_ns; // Start response timeout
                                    hand_valid_received++;
                                    usb2_current_time_ns = rtapi_get_time();
                                    usb_time_ok = 1;
                                    rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: INFO: hand_valid_send=%d. hand_valid_received=%d. hand_invalid_send=%d, hand_invalid_received=%d.\n", hand_valid_send, hand_valid_received, hand_invalid_send, hand_invalid_received);
                                }
                            }
                        }
                        else
                        {
                            *(io->hal_comm_state) = HAL_STATE_HANDSHAKE; // Could be severe error, go to ERROR
                        }
                        // Clear buffer after parsing a valid packet
                        serial_rx_buffer_len = 0;
                    }
                    else if (parsed_len < 0)
                    {
                        // Parsing error (checksum mismatch, buffer too small, etc.)
                        // Handle buffer: may discard problematic data
                        serial_rx_buffer_len = 0; // Discard full buffer on parse error
                        hand_invalid_received++;
                        // Could return to HAL_STATE_ERROR or retry handshake more aggressively
                        *(io->hal_comm_state) = HAL_STATE_HANDSHAKE;
                    }
                    // If parsed_len == 0, packet not yet complete or not found.
                    // Continue accumulating data in serial buffer.
                }
            }
            else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
            {
                // Non-temporary read error
                *(io->hal_comm_state) = HAL_STATE_ERROR;
                continue; // Skip rest of cycle and try to re-open port
            }

            // Check timeout for handshake response
            if (*(io->hal_comm_state) == HAL_STATE_HANDSHAKE_WAIT_RESPONSE && (current_time_ns - io->usb_mini_timeout_ns) > handshake_period_ns)
            {
                *(io->hal_comm_state) = HAL_STATE_HANDSHAKE;
                serial_rx_buffer_len = 0; // Clear buffer on timeout
            }
            break;
        } // end case HAL_STATE_HANDSHAKE_WAIT_RESPONSE

        case HAL_STATE_COMMUNICATING:
        {

            state_flag = HAL_STATE_COMMUNICATING;
            error_state_serial_failed = 0;
            error_state_serial_configure = 0;
            error_state_hand_packet = 0;
            error_state_firmware = 0;
            error_state_firmware_2 = 0;
            error_state_payload = 0;
            error_state_hand_serial_read = 0;
            error_state_comm_timeout = 0;

            *(io->usb_connected) = 1;

            if (!comm_state_dbg_printed)
            {
                rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: Entering COMMUNICATING state. Normal operation.\n");
                comm_state_dbg_printed = 1;
                comm_packet_received_and_validated = 0; // Reset for a new cycle
                last_read_time_ns = rtapi_get_time();
                if (!comm_state_reenter_printed)
                {
                    comm_state_reenter_printed = 1;
                    if (verbose >= VERBOSE_USB)
                    {
                        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Communication restored. \nNormal operation resumed.\n");
                    }
                }
            }

            if (current_time_ns >= next_comm_attempt_time_ns)
            {

                next_comm_attempt_time_ns = current_time_ns + io->loop_time_ns; // Schedule next communication attempt
                //*(io->led) = 1;
                io->usb_data_busy = 1;
                //					io->encoder_refresh = 0;
                //					io->in_npic_refresh = 0;
                io->out_npic_refresh = 0;
                //					io->adc_refresh = 0;
                io->dac_refresh = 0;
                //					io->in_exp_refresh = 0;
                io->out_exp_refresh = 0;
                if (usb_time_ok == 1)
                {
                    usb_current_time_ns = rtapi_get_time();
                    usb_time_ok = 0;
                }

                // --- PREPARATION and SENDING of OUTPUT Packet (HAL -> Arduino) ---
                // The payload was prepared in usb_tx_payload by the realtime thread and its length is io->usb_tx_payload_len.

                int hal_output_payload_len = io->usb_tx_payload_len;

                // Calculate total TX packet size (payload + header/footer/checksum)
                int tx_buffer_size = hal_output_payload_len + 3; // Payload + COMM_INIT (1) + Checksum (1) + COMM_END (1)

                // Safety checks (prevent overflow and negative values)
                // Ensure MAX_TX_PAYLOAD_SIZE is defined correctly.
                if (hal_output_payload_len < 0 || hal_output_payload_len > MAX_TX_PAYLOAD_SIZE)
                {
                    *(io->hal_comm_state) = HAL_STATE_HANDSHAKE;

                    break;
                }
                if (tx_buffer_size > sizeof(io->tx_packet_buffer))
                {
                    *(io->hal_comm_state) = HAL_STATE_HANDSHAKE;

                    break;
                }

                // Prepare full TX packet (encapsulating usb_tx_payload)
                io->tx_packet_buffer[0] = COMM_INIT;
                unsigned char calculated_checksum = 0;
                int current_tx_buffer_idx = 1; // Current index in tx_packet_buffer for payload

                // Copy pre-generated payload from usb_tx_payload
                memcpy(io->tx_packet_buffer + current_tx_buffer_idx, io->usb_tx_payload, hal_output_payload_len);
                current_tx_buffer_idx += hal_output_payload_len;

                // Calculate checksum over copied payload
                for (int i = 1; i < current_tx_buffer_idx; i++)
                {
                    calculated_checksum ^= io->tx_packet_buffer[i];
                }
                io->tx_packet_buffer[current_tx_buffer_idx] = calculated_checksum; // Place checksum
                io->tx_packet_buffer[current_tx_buffer_idx + 1] = COMM_END;        // Place end byte

                // Send HAL OUTPUT packet
                int bytes_written = serial_write(io->serial_fd, io->tx_packet_buffer, tx_buffer_size);
                // tcdrain(fd);          // <--- THIS forces Linux to flush data to the USB cable now!

                if (bytes_written != tx_buffer_size)
                {
                    *(io->hal_comm_state) = HAL_STATE_COMMUNICATING;
                    comm_invalid_send++;
                }
                else
                {
                    //*(io->hal_comm_state) = HAL_STATE_COMMUNICATING;
                    *(io->hal_comm_state) = HAL_STATE_COMMUNICATING_WAIT_RESPONSE;
                    comm_valid_send++;
                    //						io->encoder_refresh = 1;
                    //						io->in_npic_refresh = 1;
                    io->out_npic_refresh = 1;
                    //						io->adc_refresh = 1;
                    io->dac_refresh = 1;
                    //						io->in_exp_refresh = 1;
                    io->out_exp_refresh = 1;
                }
            }
            io->usb_data_busy = 0;

            break;
        } // End case HAL_STATE_COMMUNICATING

        case HAL_STATE_COMMUNICATING_WAIT_RESPONSE:
        {
            state_flag = HAL_STATE_COMMUNICATING_WAIT_RESPONSE;
            //*(io->led) = 0;
            if (current_time_ns < next_comm_attempt_time_ns)
            {
                io->usb_data_busy = 1;
                io->encoder_refresh = 0;
                io->in_npic_refresh = 0;
                //					io->out_npic_refresh = 0;
                io->adc_refresh = 0;
                //					io->dac_refresh = 0;
                io->in_exp_refresh = 0;
                //					io->out_exp_refresh = 0;

                // --- RECEIVING and PARSING of INPUT Packet (Arduino -> HAL) ---
                // The received payload will be written into usb_rx_payload for the other thread.

                // The expected input payload length is based on handshake parameters
                // Apply user's method for all sections.

                // Read from serial and accumulate in serial_rx_buffer
                int bytes_read = serial_read(io->serial_fd, serial_rx_buffer + serial_rx_buffer_len, sizeof(serial_rx_buffer) - serial_rx_buffer_len);
                if (bytes_read > 0)
                {
                    serial_rx_buffer_len += bytes_read;
                } /* else {
                     break;
                 }*/

                // Try to parse packets from buffer
                int parsed_len;
                do
                {
                    // Try to parse a packet and put payload directly into io->usb_rx_payload
                    parsed_len = parse_packet(serial_rx_buffer, serial_rx_buffer_len,
                                              COMM_INIT, COMM_END, io->usb_rx_payload, MAX_RX_PAYLOAD_SIZE);
                    if (error_message == 1)
                    {
                        *(io->hal_comm_state) = HAL_STATE_COMMUNICATING;
                        if (verbose >= VERBOSE_ALL)
                        {
                            rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ERROR MESSAGE IN COMMUNICATING (ASCII): '%s'\n", format_as_ascii_for_debug(serial_rx_buffer, serial_rx_buffer_len));
                        }
                    }
                    else
                    {
                        if (parsed_len < 0)
                        {                                                    // Parsing or checksum error
                            serial_rx_buffer_len = 0;                        // Discard invalid data
                            *(io->hal_comm_state) = HAL_STATE_COMMUNICATING; // Return to handshake to re-establish
                            comm_parse_error++;
                            break; // Exit parsing loop
                        }
                        else if (parsed_len > 0)
                        { // Valid packet found
                            // Payload is already in io->usb_rx_payload
                            if (parsed_len == io->usb_rx_payload_len)
                            {                                                //
                                                                             //
                                io->usb_last_read_time_ns = current_time_ns; // Update timestamp of last valid reception
                                io->usb_mini_timeout_ns = current_time_ns;   // Update mini-timeout timestamp
                                // Remove processed packet from serial buffer
                                int packet_total_len = parsed_len + 3; // Payload + COMM_INIT + Checksum + COMM_END
                                if (serial_rx_buffer_len >= packet_total_len)
                                {
                                    memmove(serial_rx_buffer, serial_rx_buffer + packet_total_len, serial_rx_buffer_len - packet_total_len);
                                    serial_rx_buffer_len -= packet_total_len;
                                }
                                else
                                {
                                    serial_rx_buffer_len = 0; // Consumed entire buffer
                                }
                                *(io->hal_comm_state) = HAL_STATE_COMMUNICATING; //
                                *(io->led) = 1;
                                comm_valid_received++;
                                io->encoder_refresh = 1;
                                io->in_npic_refresh = 1;
                                //									io->out_npic_refresh = 1;
                                io->adc_refresh = 1;
                                //									io->dac_refresh = 1;
                                io->in_exp_refresh = 1;
                                //									o->out_exp_refresh = 1;
                                usb2_current_time_ns = rtapi_get_time();
                                usb_time_ok = 1;
                                /* Run keyboard emulation in non-realtime thread:
                                  safe because we're already in usb_thread_function (non-RT) */
                                if (keyboard_initialized_ok == 1 && io->uinput_fd >= 0)
                                {
                                    key_simulation_loop(io);
                                }
                                break;
                            }
                            else
                            {
                                // Unexpected payload length (may be an error or corrupted packet)
                                serial_rx_buffer_len = 0;                        // Discard buffer
                                *(io->hal_comm_state) = HAL_STATE_COMMUNICATING; // Return to handshake
                                comm_invalid_received++;
                                break; // Exit parsing loop
                            }
                        }
                        else
                        {          // parsed_len == 0, incomplete packet, wait for more data
                            break; // Exit parsing loop, wait for next read
                        }
                    }

                } while (serial_rx_buffer_len > 0); // Continue while there's data in buffer

                io->usb_data_busy = 0;
            }
            else
            {
                *(io->hal_comm_state) = HAL_STATE_COMMUNICATING;
            }
            break; // End case HAL_STATE_COMMUNICATING_WAIT_RESPONSE
        }

        case HAL_STATE_ERROR:
        {
            state_flag = HAL_STATE_ERROR;
            // In error state the component waits and retries opening the port (handled in Section 1)
            // Alternatively implement specific recovery logic here.
            *(io->hal_comm_state) = HAL_STATE_HANDSHAKE;                       // Or HAL_STATE_HANDSHAKE
            next_comm_attempt_time_ns = current_time_ns + handshake_period_ns; // Schedule next handshake attempt
            io->usb_last_read_time_ns = current_time_ns;                       // Start response timeout
            close(io->serial_fd);                                              // Close current file descriptor
            io->serial_fd = -1;                                                // Invalidate fd to force re-open
            // error_state_serial_failed = 0;
            comm_state_dbg_printed = 0;
            comm_state_reenter_printed = 0;
            usleep(100000); // Wait 0.1s before retry
            break;
        }
        } // end switch
        // timeout handling
        if ((current_time_ns - io->usb_last_read_time_ns) > response_timeout_ns)
        {
            *(io->hal_comm_state) = HAL_STATE_HANDSHAKE;
            io->usb_last_read_time_ns = current_time_ns;                       // Reset timeout for next attempt
            serial_rx_buffer_len = 0;                                          // Clear buffer on timeout
            next_comm_attempt_time_ns = current_time_ns + handshake_period_ns; // Schedule next handshake attempt
            if (verbose >= VERBOSE_MIN)
            {
                rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Communication timeout. \nReturn to handshake.\n");
            }
        }
        if ((current_time_ns >= (next_comm_attempt_time_ns - (io->loop_time_ns / 2))) || (timer_overflow_4 == 1))
        {
            timer_overflow_4 = 0;
            *(io->led) = 0;
        }

        usb_jitter_time_ns = usb_current_time_ns - usb2_current_time_ns;
        if (usb_jitter_time_ns > 0)
        {
            if (usb_jitter_time_ns > max_usb_time)
            {
                max_usb_time = usb_jitter_time_ns;
            }
            if (usb_jitter_time_ns < min_usb_time)
            {
                min_usb_time = usb_jitter_time_ns;
            }
        }

        // error statistics function for communication
        if ((current_time_ns - next_parse_error_time_ns) > parse_error_time_ns)
        {
            next_parse_error_time_ns = current_time_ns + parse_error_time_ns; // Schedule next stats calculation
            float tx_total = 0;
            float tx_success = 0;
            float rx_total = 0;
            float rx_success = 0;
            float error_total = 0;
            float tx_total_delta = 0;
            float tx_success_delta = 0;
            float rx_total_delta = 0;
            float rx_success_delta = 0;
            float error_total_delta = 0;

            float servo_jitter_f = 0;
            float base_jitter_f = 0;
            float jitter_time = 0;  // usb_jitter_time_ns
            float loop_current = 0; // loop_current_time_ns

            tx_total = (float)comm_valid_send + comm_invalid_send;
            if (tx_total > 0)
            {
                tx_success = ((float)comm_valid_send / tx_total) * 100.0;
            }

            rx_total = (float)comm_valid_received + comm_invalid_received + comm_parse_error;
            if (rx_total > 0)
            {
                rx_success = ((float)comm_valid_received / rx_total) * 100.0;
            }

            if ((rx_total > 0) && (tx_total > 0))
            {
                error_total = ((float)(comm_parse_error + comm_invalid_send) / (tx_total + rx_total)) * 100.0;
            }

            tx_total_delta = (float)(comm_valid_send - old_comm_valid_send) + (comm_invalid_send - old_comm_invalid_send);
            if (tx_total_delta > 0)
            {
                tx_success_delta = ((float)(comm_valid_send - old_comm_valid_send) / tx_total_delta) * 100.0;
            }

            rx_total_delta = (float)(comm_valid_received - old_comm_valid_received) + (comm_invalid_received - old_comm_invalid_received) + (comm_parse_error - old_comm_parse_error);
            if (rx_total_delta > 0)
            {
                rx_success_delta = ((float)(comm_valid_received - old_comm_valid_received) / rx_total_delta) * 100.0;
            }

            if ((rx_total_delta > 0) && (tx_total_delta > 0))
            {
                error_total_delta = ((float)((comm_parse_error - old_comm_parse_error) + (comm_invalid_send - old_comm_invalid_send)) / (tx_total_delta + rx_total_delta)) * 100.0;
            }

            int parse_diff = comm_parse_error - old_comm_parse_error;
            if (comm_parse_error <= old_comm_parse_error)
            {
                parse_diff = 0;
            }
            old_comm_valid_send = comm_valid_send;
            old_comm_invalid_send = comm_invalid_send;
            old_comm_valid_received = comm_valid_received;
            old_comm_invalid_received = comm_invalid_received;
            old_comm_parse_error = comm_parse_error;

            if (max_parse_diff < parse_diff)
            {
                max_parse_diff = parse_diff;
            }

            int32_t servo_jitter = max_servo_thread_time - min_servo_thread_time;
            int32_t base_jitter = max_base_thread_time - min_base_thread_time;
            usb_jitter_time_ns = max_usb_time - min_usb_time;
            loop_current_time_ns = max_usb_loop - min_usb_loop;
            // Format time string
            time(&rawtime);
            info_time = localtime(&rawtime);
            strftime(time_buffer, sizeof(time_buffer), "Time_%H:%M:%S\nDate_%Y-%m-%d", info_time);
            *(io->hal_error_count) = error_total;
            *(io->parse_error_percentile) = error_total_delta;
            if ((print_parse_error > 1) && (verbose >= VERBOSE_MIN))
            {
                *(io->servo_jitter_pin) = servo_jitter;
                *(io->base_jitter_pin) = base_jitter;
                *(io->jitter_usb_comm) = usb_jitter_time_ns;
                *(io->jitter_usb_loop) = loop_current_time_ns;
                servo_jitter_f = (float)(servo_jitter / 1000000.0f);
                base_jitter_f = (float)(base_jitter / 1000000.0f);
                jitter_time = (float)(usb_jitter_time_ns / 1000000.0f);
                loop_current = (float)(loop_current_time_ns / 1000000.0f);
                if (error_total_delta >= 5)
                { // 5% period threshold
                    rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ERROR:\nBad USB cable communication\nOr too much Jitter\nparse error=%.1f/100\n%s\njitter.base = %.3fms\njitter.servo = %.3fms\njitter.usb.comm. =%.3fms\njitter.usb.thread =%.3fms\n",
                                    error_total_delta, time_buffer, base_jitter_f, servo_jitter_f,
                                    jitter_time, loop_current);
                }
                else if (error_total_delta >= 1)
                { // 1% period threshold
                    rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: WARNING:\nBad USB cable communication\nparse error=%.1f/100\n%s\njitter.base = %.3fms\njitter.servo = %.3fms\njitter.usb.comm. =%.3fms\njitter.usb.thread =%.3fms\n",
                                    error_total_delta, time_buffer, base_jitter_f, servo_jitter_f,
                                    jitter_time, loop_current);
                }
                else
                {
                    rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: STATISTIC OK: parse error=%.1f/100\n%s\njitter.base = %.3fms\njitter.servo = %.3fms\njitter.usb.comm. =%.3fms\njitter.usb.thread =%.3fms\n",
                                    error_total_delta, time_buffer, base_jitter_f, servo_jitter_f,
                                    jitter_time, loop_current);
                }
            }
            print_parse_error++; // first two times it enters it does not send the message
            if (print_parse_error > 2)
            {
                print_parse_error = 2;
            }
            max_servo_thread_time = 0;
            min_servo_thread_time = 2147483647;
            max_base_thread_time = 0;
            min_base_thread_time = 2147483647;
            min_usb_loop = 2147483647;
            min_usb_time = 2147483647;
            max_usb_loop = 0;
            max_usb_time = 0;

            //			rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder:STATISTIC: 		tx_success_ratio=%.2f. 		rx_success_ratio=%.2f. 		error_total_ratio=%.2f.\n", tx_success, rx_success, error_total);
            //			rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder:STATISTIC: 		tx_success_ratio_delta=%.2f. 		rx_success_ratio_delta=%.2f. 		error_total_ratio_delta=%.2f.\n", tx_success_delta, rx_success_delta, e[...]
            //			rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: STATISTIC:	 comm_valid_send=%d. 	comm_valid_received=%d. 	comm_invalid_send=%d. 	comm_invalid_received=%d. 	comm_parse_error=%d.	parse_diff=%d. 	[...]
        }

        // Small delay to avoid CPU hogging in this non-realtime loop
        // This delay is outside the communication timeout logic
        // Gives OS time to schedule other tasks and not lock CPU core
        rtapi_delay(100000); // 100 microseconds minimum delay between USB thread cycles
    } // end while (!hal_exit)

    // If loop exits (hal_exit true), close serial port if open
    if (io->serial_fd >= 0)
    {
        close(io->serial_fd);
        io->serial_fd = -1;
        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: USB thread exiting. Serial port closed.\n"); // INFO -> ERR
    }

    rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: USB thread finished.\n"); // INFO -> ERR
    io->usb_thread_running = 0;
    return;
}

/***********************************************************************
 * REALTIME LOOP CALCULATIONS                     *
 ************************************************************************/

// npic inputs from buttons
uint8_t npic_in_routine(hal_io_decoder_t *io, uint8_t *buffer, uint8_t length)
{

    uint8_t i;
    uint8_t bytesProcessed = 0; // Count of bytes read from buffer
    uint8_t currentByte = 0;
    uint8_t current_bit = 0;
    uint8_t bitIndex;
    uint8_t bitIndex_app;
    uint8_t bitValue;

    if (io->in_npic_refresh == 1)
    {                            // this provides a kind of debounce. only on the second pass the HAL pin is
        io->in_npic_refresh = 0; // validated if equal to previous pass. pin updated after a USB loop (20ms)
        switch (firmware)
        {
        case 255:
            // Set values specific to firmware 255
            bitIndex_app = 4;
            break;

        default:
            bitIndex_app = 8;
        }
        for (i = 0; i < io->arduino_byte_npic_input; i++)
        {
            currentByte = buffer[i];
            for (bitIndex = 0; bitIndex < bitIndex_app; bitIndex++)
            {
                bitValue = (currentByte >> bitIndex) & 0x01;
                if (bitValue == in_bit_d[current_bit])
                {
                    *(io->in_bit[current_bit]) = bitValue;
                    if (io->in_bit_t[current_bit] != NULL)
                    {
                        if ((bitValue == 1) && (in_bit_t_state[current_bit] == 0))
                        {
                            *(io->in_bit_t[current_bit]) = !(*(io->in_bit_t[current_bit]));
                            in_bit_t_state[current_bit] = 1;
                        }
                        if (bitValue == 0)
                        {
                            in_bit_t_state[current_bit] = 0;
                        }
                    }
                }
                else
                {
                    in_bit_d[current_bit] = bitValue;
                }

                current_bit++;
            }
            bytesProcessed++;
        }
        if (current_bit != input)
        {
            rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: input extraction: mismatch.\n"); // INFO -> ERR
        }
    }
}

// npic outputs to indicator lights
void npic_out_routine(hal_io_decoder_t *io, uint8_t *data, uint8_t length)
{

    uint8_t i;
    uint8_t currentByte = 0;
    uint8_t current_bit = 0;
    uint8_t bitIndex;
    uint8_t bitIndex_app;
    uint8_t bitValue;
    long current_time_ns;
    long freq_conversion;
    long freq_conversion2;
    float freq_command;
    float freq_width;

    if (io->out_npic_refresh == 1)
    { // to avoid flooding cycles values update each USB loop. transmitted data is delayed by one loop
        io->out_npic_refresh = 0;
        current_time_ns = rtapi_get_time(); // Update current time at the start of each cycle
        if (current_time_ns < 0)
        {
            // If overflow, re-normalize to positive to avoid breaking calculations
            current_time_ns = current_time_ns & (~0x80000000); // Mask 32nd bit
        }
        switch (firmware)
        {
        case 255:
            // Set values specific to firmware 255
            bitIndex_app = 4;
            break;

        default:
            bitIndex_app = 8;
        }
        for (i = 0; i < io->arduino_byte_npic_output; i++)
        {
            if (current_time_ns < io->out_blink_time_ns[current_bit])
            {
                io->out_blink_time_ns[current_bit] = current_time_ns;
            }
            currentByte = 0;
            for (bitIndex = 0; bitIndex < bitIndex_app; bitIndex++)
            {
                bitValue = *(io->out_bit[current_bit]);

                if (io->out_bit_blink_en[current_bit] && *(io->out_bit_blink_en[current_bit]) == 1)
                {
                    freq_command = 0.25; // default and minimum 0.25Hz = 4 seconds
                    freq_width = 0.5;    // default
                    if (*(io->out_bit_blink_freq[current_bit]) >= freq_command)
                    {
                        freq_command = *(io->out_bit_blink_freq[current_bit]);
                        if (freq_command > 16)
                        {
                            freq_command = 16;
                            ;
                        }
                        freq_conversion = (long)((1.0f / freq_command) * 1000 * 1000 * 1000);
                    }
                    if (bitValue == 1)
                    { // io->out_blink_time_ns[current_bit]
                        if ((*(io->out_bit_blink_width[current_bit]) > 0) && (*(io->out_bit_blink_width[current_bit]) < 1))
                        {
                            freq_width = *(io->out_bit_blink_width[current_bit]);
                        }
                        freq_conversion2 = (long)(((1.0f / freq_command) * freq_width) * 1000 * 1000 * 1000);
                        if (current_time_ns > (io->out_blink_time_ns[current_bit] + freq_conversion))
                        {
                            bitValue = 1;
                            io->out_blink_time_ns[current_bit] = current_time_ns;
                        }
                        else if (current_time_ns > (io->out_blink_time_ns[current_bit] + (freq_conversion2)))
                        {
                            bitValue = 0;
                        }
                    }
                }

                if (bitValue == 1)
                {
                    currentByte |= (1 << bitIndex); // Set current bit (LSB first)
                }
                else
                {
                    currentByte &= ~(1 << bitIndex);
                }
                current_bit++;
            }
            data[i] = currentByte;
        }
        if (current_bit != output)
        {
            rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: output insection: mismatch.\n"); // INFO -> ERR
        }
    }
}

uint8_t encoder_routine(hal_io_decoder_t *io, uint8_t *buffer, uint8_t length)
{

    int encoder_idx;
    int encoder_delta_from_arduino; // This will be the decoded "delta"
    int i;
    uint8_t bytesProcessed = 0; // Count of bytes read from buffer

    if (io->encoder_refresh == 1)
    {
        io->encoder_refresh = 0;

        for (i = 0; i < io->firmware_n_encoder; i++)
        {

            if (bytesProcessed + io->arduino_byte_encoder > length)
            {
                break; // No more data space in buffer, exit
            }

            int32_t currentDelta = 0; // Use int32_t to manage delta value

            if (io->arduino_bit_encoder <= 8)
            {
                // Case: 8-bit encoder (1 byte)
                uint8_t inputByte = buffer[bytesProcessed];
                bytesProcessed += 1;

                if (inputByte & 0x80)
                { // Check MSB (sign bit)
                    // If MSB is 1, number is negative
                    io->enc_delta_sign[i] = 1;
                    currentDelta = (int32_t)(inputByte & 0x7F);
                    io->enc_delta[i] = currentDelta * 2;
                    currentDelta = currentDelta * -1;
                }
                else
                {
                    // If MSB is 0, number is positive
                    io->enc_delta_sign[i] = 0;
                    currentDelta = (int32_t)(inputByte & 0x7F);
                    io->enc_delta[i] = currentDelta * 2;
                }
            }
            else
            { // Assume 16-bit if > 8 (2 bytes)
                // Case: 16-bit encoder
                // Arduino sends MSB-first, rebuild 16-bit value
                uint16_t inputWord = ((uint16_t)buffer[bytesProcessed] << 8) | buffer[bytesProcessed + 1];
                bytesProcessed += 2;

                if (inputWord & 0x8000)
                { // Check MSB (sign bit)
                    // If MSB is 1, number is negative
                    io->enc_delta_sign[i] = 1;
                    currentDelta = (int32_t)(inputWord & 0x7FFF);
                    io->enc_delta[i] = currentDelta * 2;
                    currentDelta = currentDelta * -1;
                }
                else
                {
                    // If MSB is 0, number is positive
                    io->enc_delta_sign[i] = 0;
                    currentDelta = (int32_t)(inputWord & 0x7FFF);
                    io->enc_delta[i] = currentDelta * 2;
                }
            }

            // Apply inversion if HAL pin enc_invert is TRUE
            if (io->enc_invert[i] && *(io->enc_invert[i]) == 1)
            {
                currentDelta = -currentDelta;
            }

            // Update cumulative s32 pin value
            *(io->enc[i]) += currentDelta;
        }
    }
    // generate pulses for corresponding enc_up and enc_down HAL pins
    for (encoder_idx = 0; encoder_idx < io->firmware_n_encoder; encoder_idx++)
    {
        if (io->enc_delta[encoder_idx] > 0)
        {
            if (io->enc_delta[encoder_idx] % 2 == 0)
            {
                // even value
                if (io->enc_delta_sign[encoder_idx] == 0)
                { // If sign bit is 0 (corresponds to negative)
                    *(io->enc_down[encoder_idx]) = 1;
                }
                else
                { // If sign bit is 1 (corresponds to positive)
                    *(io->enc_up[encoder_idx]) = 1;
                }
            }
            else
            {
                // odd value
                *(io->enc_up[encoder_idx]) = 0;
                *(io->enc_down[encoder_idx]) = 0;
            }
            io->enc_delta[encoder_idx]--;
        }
        else
        {
            io->enc_delta[encoder_idx] = 0;
            *(io->enc_up[encoder_idx]) = 0;
            *(io->enc_down[encoder_idx]) = 0;
        }
    }

    return bytesProcessed; // Returns total number of bytes that were read
}

int32_t map(int32_t value, int32_t min_old, int32_t max_old, int32_t min_new, int32_t max_new)
{
    /* Protect against division by zero: if input range is zero,
       return the lower bound of the target range (safe fallback). */
    if (max_old == min_old)
    {
        rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: map(): avoided division by zero (min_old==max_old)\n");
        return min_new;
    }
    return (value - min_old) * (max_new - min_new) / (max_old - min_old) + min_new;
}

uint8_t adc_routine(hal_io_decoder_t *io, uint8_t *buffer, uint8_t length)
{

    int i;
    uint8_t bytesProcessed = 0; // Count of bytes read from buffer
    float_t current_value = 0;  // Use int32_t-like variable to hold ADC value
    uint32_t input_value = 0;
    long current_time_ns;
    long overflow_period_ns = 20 * 1000 * 1000;
    int32_t joy_deadb;
    int32_t joy_offset;
    // uint32_t joy_log;
    /* Safely compute ADC max value avoiding UB with shifts and avoid division by zero.
       If arduino_bit_adc is 0 or unreasonable, fallback to a safe value. */
    uint32_t bit_count = (uint32_t)io->arduino_bit_adc;
    if (bit_count == 0)
    {
        rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: adc_routine: arduino_bit_adc==0, defaulting to 8 bits\n");
        bit_count = 8;
    }
    if (bit_count > 31)
    {
        rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: adc_routine: arduino_bit_adc too large (%u), capping to 31\n", bit_count);
        bit_count = 31;
    }
    uint32_t max_val_u = (bit_count >= 32) ? 0xFFFFFFFFu : ((1u << bit_count) - 1u);
    float_t max_value = (float_t)max_val_u;
    if (max_value <= 0.0f)
    {
        rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: adc_routine: corrected max_value to 1 to avoid division by zero\n");
        max_value = 1.0f;
    }
    int32_t max_value_2 = (int32_t)(max_value / 2.0f);
    int32_t joy_center;
    int32_t joy_center_plus;
    int32_t joy_center_minus;
    int32_t max_joy_ms;
    int32_t min_joy_ms;
    long time_plus_ns;
    long time_minus_ns;
    int joy_plus_ok = 0;
    int joy_minus_ok = 0;

    if (io->adc_refresh == 1)
    {
        io->adc_refresh = 0;
        for (i = 0; i < io->firmware_n_adc; i++)
        {
            if (bytesProcessed + io->arduino_byte_adc > length)
            {
                break; // No more space in buffer, exit
            }

            current_time_ns = rtapi_get_time(); // Update current time at start of cycle

            if (current_time_ns < 0)
            {
                // If overflow, re-normalize to positive
                current_time_ns = current_time_ns & (~0x80000000); // Mask 32nd bit
                if (current_time_ns < io->next_adc_joy_ns[i])
                {                                             // first wrap
                    io->next_adc_joy_ns[i] = current_time_ns; // Schedule next attempt
                }
            }

            if (io->arduino_bit_adc <= 8)
            {
                // Case: 8-bit ADC (1 byte)
                uint8_t inputByte = buffer[bytesProcessed];
                bytesProcessed += 1;
                *(io->adc_raw[i]) = (uint32_t)inputByte;
                input_value = (uint32_t)inputByte;
            }
            else
            { // Assume 16-bit if > 8 (2 bytes)
                // Case: 16-bit ADC
                // Arduino sends MSB-first, rebuild 16-bit value
                uint16_t inputWord = ((uint16_t)buffer[bytesProcessed] << 8) | buffer[bytesProcessed + 1];
                bytesProcessed += 2;
                *(io->adc_raw[i]) = (uint32_t)inputWord;
                input_value = (uint32_t)inputWord;
            }

            // Apply low-pass filter
            io->old_adc_raw[i] = (input_value * ALPHA) + (io->old_adc_raw[i] * (1.0 - ALPHA));
            current_value = (float_t)io->old_adc_raw[i];

            // Apply inversion if HAL pin adc_invert is TRUE
            if (io->adc_invert[i] && *(io->adc_invert[i]) == 1)
            {
                current_value = max_value - current_value;
            }

            io->adc_raw_joy[i] = current_value;

            if ((*(io->adc_joy_deadb[i]) > 0) && (*(io->adc_joy_deadb[i]) < 1))
            {
                joy_deadb = (int32_t)(max_value_2 * (*(io->adc_joy_deadb[i])));
            }
            else
            {
                joy_deadb = 0;
            }
            if ((*(io->adc_joy_center[i]) > -1) && (*(io->adc_joy_center[i]) < 1))
            {
                joy_center = (int32_t)(max_value_2 + (max_value_2 * (*(io->adc_joy_center[i]))));
            }
            else
            {
                joy_center = max_value_2;
            }
            max_joy_ms = 2000;
            min_joy_ms = 100;

            if ((*(io->adc_joy_factor[i]) >= 0) && (*(io->adc_joy_factor[i]) <= 0.999))
            {
                min_joy_ms = (int32_t)(min_joy_ms * (1 - *(io->adc_joy_factor[i])));
            }
            else
            {
                min_joy_ms = (int32_t)(min_joy_ms * 0.001);
            }

            joy_center_plus = joy_center + joy_deadb;
            joy_center_minus = joy_center - joy_deadb;

            if (io->adc_joy_valid[i] == 0)
            {
                if ((io->adc_raw_joy[i] <= joy_center_plus) && (io->adc_raw_joy[i] >= joy_center_minus))
                {
                    // Consider value valid for the first time.
                    io->adc_joy_valid[i] = 1;
                }
            }

            if (io->adc_raw_joy[i] > (joy_center_plus))
            {
                time_plus_ns = (long)map((io->adc_raw_joy[i] - joy_center_plus), 0, (max_value - joy_center_plus), max_joy_ms, min_joy_ms);
                time_plus_ns = time_plus_ns * 1000 * 1000;
                time_minus_ns = 0;
                joy_plus_ok = 1;
                joy_minus_ok = 0;
            }
            else if (io->adc_raw_joy[i] < (joy_center_minus))
            {
                time_minus_ns = (long)map((joy_center_minus - io->adc_raw_joy[i]), 0, joy_center_minus, max_joy_ms, min_joy_ms);
                time_minus_ns = time_minus_ns * 1000 * 1000;
                time_plus_ns = 0;
                joy_minus_ok = 1;
                joy_plus_ok = 0;
            }
            else
            {
                joy_plus_ok = 0;
                joy_minus_ok = 0;
                time_plus_ns = 0;
                time_minus_ns = 0;
                io->next_adc_joy_ns[i] = current_time_ns; // Schedule next attempt immediately
            }
            if ((*(io->hal_comm_state) == HAL_STATE_COMMUNICATING) || (*(io->hal_comm_state) == HAL_STATE_COMMUNICATING_WAIT_RESPONSE))
            {
                if (io->adc_joy_valid[i])
                {
                    if (joy_plus_ok == 1)
                    {
                        if (current_time_ns > io->next_adc_joy_ns[i])
                        {
                            io->next_adc_joy_ns[i] = current_time_ns + time_plus_ns; // Schedule next attempt
                            *(io->adc_joy_pulse_up[i]) = 1;
                            (*(io->adc_joy_count[i]))++;
                        }
                        if (current_time_ns > (io->next_adc_joy_ns[i] - (time_plus_ns / 2)))
                        {
                            *(io->adc_joy_pulse_up[i]) = 0;
                        }
                    }
                    else
                    {
                        *(io->adc_joy_pulse_up[i]) = 0;
                    }

                    if (joy_minus_ok == 1)
                    {
                        if (current_time_ns > io->next_adc_joy_ns[i])
                        {
                            io->next_adc_joy_ns[i] = current_time_ns + time_minus_ns; // Schedule next attempt
                            *(io->adc_joy_pulse_down[i]) = 1;
                            (*(io->adc_joy_count[i]))--;
                        }
                        if (current_time_ns > (io->next_adc_joy_ns[i] - (time_minus_ns / 2)))
                        {
                            *(io->adc_joy_pulse_down[i]) = 0;
                        }
                    }
                    else
                    {
                        *(io->adc_joy_pulse_down[i]) = 0;
                    }
                }
            }
            else
            {
                *(io->adc_joy_pulse_up[i]) = 0;
                *(io->adc_joy_pulse_down[i]) = 0;
                io->adc_joy_valid[i] = 0;
            }

            // check to avoid negative division
            float_t scale_check;
            if (*(io->adc_scale[i]) > 0)
            {
                scale_check = *(io->adc_scale[i]);
            }
            else
            {
                scale_check = 1;
            }
            // Update cumulative float pin value
            *(io->adc[i]) = (current_value / max_value) * scale_check;

            // debug check
            if (i == 0)
            {
                float app_time;
                if (time_plus_ns != 0)
                {
                    app_time = (float)((time_plus_ns / 1000) / 1000);
                    *(io->debug_f) = (1000 * (1 / app_time));
                }
                else
                {
                    *(io->debug_f) = 0;
                }
                if (time_minus_ns != 0)
                {
                    app_time = (float)((time_minus_ns / 1000) / 1000);
                    *(io->debug_f2) = (1000 * (1 / app_time));
                }
                else
                {
                    *(io->debug_f2) = 0;
                }
            }
        }
    }

    //	for (adc_idx = 0; adc_idx < io->firmware_n_adc; adc_idx++) {

    //	//}
}

void DAC_out_routine(hal_io_decoder_t *io, uint8_t *data, uint8_t length)
{

    int i;
    uint8_t bytesWritten = 0;  // Count of bytes written to buffer
    float_t current_value = 0; // Use numeric type to handle DAC value

    if (io->dac_refresh == 1)
    {
        io->dac_refresh = 0;
        float_t max_value = (1 << io->arduino_bit_dac) - 1;
        for (int i = 0; i < io->firmware_n_dac; i++)
        {
            if ((bytesWritten + io->arduino_byte_dac) > length)
            {          // Check buffer space
                break; // Not enough space, exit loop
            }
            // check to avoid division by 0 or negative
            float_t scale_check;
            if (*(io->dac_scale[i]) > 0)
            {
                scale_check = *(io->dac_scale[i]);
            }
            else
            {
                scale_check = 1;
            }
            // apply conversion to write value
            current_value = (*(io->dac[i]) / scale_check) * max_value;
            // Apply inversion if HAL pin adc_invert is TRUE
            if (io->dac_invert[i] && *(io->dac_invert[i]) == 1)
            {
                current_value = max_value - current_value;
            }

            if (io->arduino_bit_dac <= 8)
            {
                data[bytesWritten++] = (uint8_t)current_value; // Write byte to buffer
            }
            else
            {
                uint16_t outputWord = (uint16_t)current_value;
                data[bytesWritten++] = (uint8_t)((outputWord >> 8) & 0xFF); // High byte
                data[bytesWritten++] = (uint8_t)(outputWord & 0xFF);        // Low byte
            }
        }
    }
}

uint8_t in_expansion_routine(hal_io_decoder_t *io, uint8_t *buffer, uint8_t length)
{

    int i;
    uint8_t bytesWritten = 0;  // Count of bytes read from buffer
    float_t current_value = 0; // Use numeric type to handle values

    if (io->in_exp_refresh == 1)
    {
        io->in_exp_refresh = 0;
        // Implement logic based on EXPANSION_CODE
        switch (io->arduino_firmware)
        {
        case 101: // EXPANSION_CODE = 0: "No expansion"
            // If no input expansion or placeholder,
            // write bytes but do nothing.
            break;

            // --- PREPARATION FOR FUTURE INPUT EXPANSIONS ---
            // case 1: // Example: Expansion to receive values from external device
            //
            //     break;

            // case 2: // Example: Expansion for external ADC SPI/I2C
            //
            //     break;

        default:
            // Handle unrecognized expansion codes
            //
            // Do nothing on hardware or set safe state if possible.
            break;
        }
    }
}

void out_expansion_routine(hal_io_decoder_t *io, uint8_t *data, uint8_t length)
{

    int i;
    uint8_t bytesWritten = 0;  // Count of bytes written to buffer
    float_t current_value = 0; // Use numeric type to handle values

    if (io->out_exp_refresh == 1)
    {
        io->out_exp_refresh = 0;
        // Implement logic based on EXPANSION_CODE
        switch (io->arduino_firmware)
        {
        case 101: // EXPANSION_CODE = 0: "No expansion"
            // If no output expansion or placeholder,
            // read bytes but do nothing on hardware.
            break;

            // --- PREPARATION FOR FUTURE OUTPUT EXPANSIONS ---
            // case 1: // Example: Expansion for relay/LED control (digital output)
            //
            //     break;

            // case 2: // Example: Expansion for external DAC SPI/I2C
            //
            //     break;

        default:
            // Handle unrecognized expansion codes
            //
            // Do nothing on hardware or set safe state if possible.
            break;
        }
    }
}

static void emit(int fd, int type, int code, int value)
{
    struct input_event ev;

    // Ensure safe initialization
    memset(&ev, 0, sizeof(ev));

    ev.type = type;
    ev.code = code;
    ev.value = value;

    /* Writes to /dev/uinput are NOT realtime-safe.
       Ensure emit() is called only from the non-realtime USB thread.
       Minimal validation + best-effort write. */
    if (fd < 0)
        return;
    ssize_t w = write(fd, &ev, sizeof(ev));
    (void)w; /* ignore errors here; usb thread may log if needed */
}

static void key_simulation_loop(void *arg)
{

    hal_io_decoder_t *io = (hal_io_decoder_t *)arg;

    int i, j;
    uint8_t bitValue;
    uint32_t keysym;
    uint32_t keysym_modifier;
    int send_status;
    long current_time_ns;

    current_time_ns = rtapi_get_time(); // Update current time at start of each cycle

    if (current_time_ns < 0)
    {
        // If overflow, re-normalize to positive to avoid breaking calculations
        current_time_ns = current_time_ns & (~0x80000000); // Mask 32nd bit
        if (current_time_ns < next_key_attempt_time_ns - KEY_SCAN_DELAY_NS)
        {                                                                   // first wrap
            next_key_attempt_time_ns = current_time_ns + KEY_SCAN_DELAY_NS; // Schedule next attempt
        }
    }

    if (current_time_ns >= next_key_attempt_time_ns)
    {

        for (j = 1; j < 4; j++)
        {
            for (i = 0; i < input; i++)
            {
                // FILTER SCAN BY TYPE (MODIFIER FIRST, NORMAL AFTER)
                if (((j == KEY_TYPE_MODIFIER) || (j == KEY_TYPE_BOTH)) && (key_input_map[i].key_type == j))
                {
                    key_input_map[i].current_state = *(io->in_bit[i]);
                    // TRANSITION LOGIC (Down and Up)
                    if (key_input_map[i].current_state != key_input_map[i].previous_state)
                    {

                        // keysym = key_input_map[i].keycode_value;
                        keysym_modifier = key_input_map[i].modifier_value;

                        int suppress_base_key = 0; // 0 = false, 1 = true

                        // *** KEY DOWN CALL ***
                        if (key_input_map[i].current_state == 1)
                        {
                            if (keysym_modifier != 0)
                            {
                                switch (keysym_modifier)
                                {
                                case KEY_LEFTSHIFT:
                                    if (shift_count == 0)
                                    {
                                        emit(io->uinput_fd, EV_KEY, keysym_modifier, 1);
                                        emit(io->uinput_fd, EV_SYN, SYN_REPORT, 0); // Don't forget sync!
                                    }
                                    shift_count++;
                                    break;
                                case KEY_RIGHTCTRL:
                                    if (ctrl_count == 0)
                                    {
                                        emit(io->uinput_fd, EV_KEY, keysym_modifier, 1);
                                        emit(io->uinput_fd, EV_SYN, SYN_REPORT, 0); // Don't forget sync!
                                        suppress_base_key = 1;
                                    }
                                    ctrl_count++;
                                    break;
                                case KEY_LEFTALT:
                                    if (l_alt_count == 0)
                                    {
                                        // keysym = 0;
                                        emit(io->uinput_fd, EV_KEY, keysym_modifier, 1);
                                        emit(io->uinput_fd, EV_SYN, SYN_REPORT, 0); // Don't forget sync!
                                    }
                                    l_alt_count++;
                                    break;
                                case KEY_RIGHTALT:
                                    if (r_alt_count == 0)
                                    {
                                        emit(io->uinput_fd, EV_KEY, keysym_modifier, 1);
                                        emit(io->uinput_fd, EV_SYN, SYN_REPORT, 0); // Don't forget sync!
                                    }
                                    r_alt_count++;
                                    break;
                                default:

                                    break;
                                }
                            }
                        }

                        // *** KEY UP CALL ***
                        if (key_input_map[i].current_state == 0)
                        {
                            if (keysym_modifier != 0)
                            {
                                switch (keysym_modifier)
                                {
                                case KEY_LEFTSHIFT:
                                    shift_count--;
                                    if (shift_count <= 0)
                                    {
                                        shift_count = 0;
                                        emit(io->uinput_fd, EV_KEY, keysym_modifier, 0);
                                        emit(io->uinput_fd, EV_SYN, SYN_REPORT, 0); // Don't forget sync!
                                    }
                                    break;
                                case KEY_RIGHTCTRL:
                                    ctrl_count--;
                                    if (ctrl_count <= 0)
                                    {
                                        ctrl_count = 0;
                                        emit(io->uinput_fd, EV_KEY, keysym_modifier, 0);
                                        emit(io->uinput_fd, EV_SYN, SYN_REPORT, 0); // Don't forget sync!
                                    }
                                    break;
                                case KEY_LEFTALT:
                                    l_alt_count--;
                                    if (l_alt_count <= 0)
                                    {
                                        l_alt_count = 0;
                                        emit(io->uinput_fd, EV_KEY, keysym_modifier, 0);
                                        emit(io->uinput_fd, EV_SYN, SYN_REPORT, 0); // Don't forget sync!
                                    }
                                    break;
                                case KEY_RIGHTALT:
                                    r_alt_count--;
                                    if (r_alt_count <= 0)
                                    {
                                        r_alt_count = 0;
                                        emit(io->uinput_fd, EV_KEY, keysym_modifier, 0);
                                        emit(io->uinput_fd, EV_SYN, SYN_REPORT, 0); // Don't forget sync!
                                    }
                                    break;
                                default:

                                    break;
                                }
                            }
                        }

                        if (j == KEY_TYPE_MODIFIER)
                        {
                            // Update previous state AFTER action
                            key_input_map[i].previous_state = key_input_map[i].current_state;
                        }
                    }
                }
                if (j == KEY_TYPE_NORMAL)
                {
                    if ((key_input_map[i].key_type == KEY_TYPE_NORMAL) || (key_input_map[i].key_type == KEY_TYPE_BOTH))
                    {
                        key_input_map[i].current_state = *(io->in_bit[i]);
                        // TRANSITION LOGIC (Down and Up)
                        if (key_input_map[i].current_state != key_input_map[i].previous_state)
                        {

                            keysym = key_input_map[i].keycode_value;
                            if (key_input_map[i].current_state == 1)
                            {
                                if (keysym != 0)
                                {
                                    emit(io->uinput_fd, EV_KEY, keysym, 1);
                                    emit(io->uinput_fd, EV_SYN, SYN_REPORT, 0); // Don't forget sync!
                                }
                            }
                            if (key_input_map[i].current_state == 0)
                            {
                                if (keysym != 0)
                                {
                                    emit(io->uinput_fd, EV_KEY, keysym, 0);
                                    emit(io->uinput_fd, EV_SYN, SYN_REPORT, 0);
                                }
                            }

                            key_input_map[i].previous_state = key_input_map[i].current_state;
                        }
                    }
                }
            }
        }
        next_key_attempt_time_ns = current_time_ns + KEY_SCAN_DELAY_NS; // Schedule next attempt
    }
}

// Function to map prefix to modifier KEY_ code
static int get_modifier_keycode(char prefix_char)
{
    switch (prefix_char)
    {
    case 's':
        return KEY_LEFTSHIFT;
    case 'S':
        return KEY_LEFTSHIFT;

    case 'c':
        return KEY_RIGHTCTRL;
    case 'C':
        return KEY_RIGHTCTRL;

    case 'a':
        return KEY_LEFTALT;
    case 'A':
        return KEY_LEFTALT;

    case 'g':
        return KEY_RIGHTALT;
    case 'G':
        return KEY_RIGHTALT;

    case '0':
        return 0;
    case '1':
        return 0;
    case '2':
        return 0;
    case '3':
        return 0;
    case '4':
        return 0;
    case '5':
        return 0;
    case '6':
        return 0;
    case '7':
        return 0;
    case '8':
        return 0;
    case '9':
        return 0;

    default:
        return -1; // No modifier
    }
}

int load_key_map(const char *filename)
{
    FILE *fp;
    char line[256];
    char temp_line[256];
    char *token;
    char pin_name[32];
    int keycode, index_found;
    int line_count = 0;
    int board_num, pin_num;
    int i;
    char key_config_str[32];    // Space for string like "s45"
    int modifier_key = 0;       // NEW VARIABLE
    uint32_t base_key_code = 0; // NEW VARIABLE
    char prefix_char = 0;       // NEW VARIABLE

    for (i = 0; i < MAX_INPUT; i++)
    {
        key_input_map[i].current_state = 0;
        key_input_map[i].previous_state = 0;
        key_input_map[i].key_type = 0;
        key_input_map[i].keycode_value = 0;
        key_input_map[i].modifier_value = 0;
        key_input_map[i].both_loop = 0;
    }

    fp = fopen(filename, "r");
    if (fp == NULL)
    {
        if (verbose >= VERBOSE_USB)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Warning! Key-map file \n%s \nnot found\nThe component is loaded without keyboard functionallity", filename);
        }
        keyboard_initialized_ok = 0;
        return 0;
    }
    else
    {
        keyboard_initialized_ok = 1;
        rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: Loading custom keymap from: %s\n", filename);
    }

    if (keyboard_initialized_ok == 1)
    {
        while (fgets(line, sizeof(line), fp))
        {
            line_count++;

            // Ignore empty lines or comments (#)
            if (line[0] == '\n' || line[0] == '#' || line[0] == '\r')
                continue;

            // Find first '#' and truncate there.
            char *comment_start = strchr(line, '#');
            if (comment_start != NULL)
            {
                *comment_start = '\0'; // Replace '#' with string terminator
            }

            // Work on a copy, since strtok modifies the original
            strncpy(temp_line, line, sizeof(temp_line) - 1);
            temp_line[sizeof(temp_line) - 1] = '\0';
            /* Trim leading and trailing whitespace (spaces, tabs, CR, LF).
               This handles lines with indentation or CRLF endings so empty/comment-only
               lines are skipped reliably. */
            char *p = temp_line;
            while (*p && isspace((unsigned char)*p))
                p++;
            if (*p == '\0')
                continue; // empty or whitespace-only line
            /* trim trailing whitespace */
            char *end = p + strlen(p) - 1;
            while (end > p && isspace((unsigned char)*end))
            {
                *end = '\0';
                end--;
            }
            // 1. Robust extraction of first token (pin) and second token (keycode)
            // Normalize common invisible chars: remove UTF-8 BOM and replace NBSP (0xA0)
            char *q = p;
            if ((unsigned char)q[0] == 0xEF && (unsigned char)q[1] == 0xBB && (unsigned char)q[2] == 0xBF)
                q += 3; // skip BOM if present
            for (char *r = q; *r; r++)
                if ((unsigned char)*r == 0xA0)
                    *r = ' ';
            char pin_token[64] = {0};
            char key_token[64] = {0};
            int adv = 0;
            if (sscanf(q, "%63s%n", pin_token, &adv) < 1)
            {
                rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Warning!\nKeyboard configuration file\nLine %d: Missing pin name.\n", line_count);
                continue;
            }
            // advance pointer past the pin token and any whitespace
            q += adv;
            while (*q && isspace((unsigned char)*q))
                q++;
            if (*q == '\0')
            {
                rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Warning!\nKeyboard configuration file\nLine %d: Missing keycode for pin %s.\n", line_count, pin_token);
                continue;
            }
            // read next token until whitespace (handles tabs/spaces)
            int ki = 0;
            while (*q && !isspace((unsigned char)*q) && ki < (int)sizeof(key_token) - 1)
                key_token[ki++] = *q++;
            key_token[ki] = '\0';
            // copy pin and key token into existing buffers
            strncpy(pin_name, pin_token, sizeof(pin_name) - 1);
            pin_name[sizeof(pin_name) - 1] = '\0';
            strncpy(key_config_str, key_token, sizeof(key_config_str) - 1);
            key_config_str[sizeof(key_config_str) - 1] = '\0';
            prefix_char = key_config_str[0];
            modifier_key = get_modifier_keycode(prefix_char);

            if (modifier_key != 0)
            {
                if (modifier_key > 0)
                {
                    // key_config_str + 1 points to number (e.g. "45")
                    base_key_code = (uint32_t)strtol(key_config_str + 1, NULL, 10);
                    keycode = 1;
                }
                else
                {
                    rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Warning!\nKeyboard configuration file\nLine %d:\nIncorrect modifier!\nInserted: %c\nAllowed: s, c, a, g .\n", line_count, prefix_char);
                    keycode = 0;
                    continue;
                }
            }
            else
            {
                // No modifier (e.g. "45"): entire string is base key
                base_key_code = (uint32_t)strtol(key_config_str, NULL, 10);
                keycode = 0;
            }

            // -------------------------------------------------------------------
            // Format: "in." (literal) followed by integer (%d) - (dash) followed by integer (%d)
            if (sscanf(pin_name, "in.%d-%d", &board_num, &pin_num) != 2)
            {
                // Error: string doesn't match expected format
                rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Warning!\nKeyboard configuration file\nInvalid pin format: \nInserted: %s.\nCorrect format: in.MM-N\nData Ignored.\n", pin_name);
                continue;
            }

            index_found = (board_num * 8) + pin_num;
            key_input_map[index_found].keycode_value = base_key_code;
            if ((index_found >= input) || (pin_num > 7))
            {
                // Error: values outside declared limits
                index_found = (input - 1) / 8;
                rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Warning!\nKeyboard configuration file\nInvalid pin format: \nValues outside declared limits\nMax in.%d-N\nMax in.MM-7\nInserted: %s.\nData Ignored.\n", index_found, pin_name);
                continue;
            }

            // Check if keycode is one of known modifiers.
            // Not all modifiers need be used, only those of interest.
            if (base_key_code == KEY_LEFTSHIFT || base_key_code == KEY_RIGHTSHIFT ||
                base_key_code == KEY_LEFTCTRL || base_key_code == KEY_RIGHTCTRL ||
                base_key_code == KEY_LEFTALT || base_key_code == KEY_RIGHTALT)
            {
                key_input_map[index_found].key_type = KEY_TYPE_MODIFIER;
                key_input_map[index_found].keycode_value = base_key_code;
                key_input_map[index_found].modifier_value = base_key_code;
            }
            else
            {
                key_input_map[index_found].key_type = KEY_TYPE_NORMAL;
            }

            if (keycode == 1)
            {
                key_input_map[index_found].key_type = KEY_TYPE_BOTH;
                key_input_map[index_found].modifier_value = (uint32_t)modifier_key;
            }

            if (base_key_code > 0)
            {
                in_bit_keycode_value[keycode_number] = base_key_code;
                keycode_number++;
            }
            if (key_input_map[index_found].modifier_value > 0)
            {
                in_bit_keycode_value[keycode_number] = key_input_map[index_found].modifier_value;
                keycode_number++;
            }
        }
        if (keycode_number == 0)
        {
            keyboard_initialized_ok = 0;
            if (verbose >= VERBOSE_USB)
            {
                rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Warning!\nKeyboard configuration file empty\nNo configured pins.\nThe component is loaded without keyboard functionality\n");
            }
        }
    }
    fclose(fp);

    return 0;
}

static int init_uinput(hal_io_decoder_t *addr)
{
    struct uinput_user_dev uidev;
    int i;
    int retval = 0; // Local return code

    // Static globals populated by load_key_map (assumed available)
    extern uint32_t in_bit_keycode_value[];
    extern int keycode_number;

    if (uinput_chmod_cmd != NULL && strlen(uinput_chmod_cmd) > 0)
    {
        // 1. Open descriptor and set permissions
        // system("chmod 0666 /dev/uinput");
        // system(uinput_chmod_cmd);
        /* Prefer using chmod() with an octal permission string to avoid spawning a shell.
           Accept forms like "0666", "666" or "chmod 0666 /dev/uinput".
           If parsing fails, fallback to system() for compatibility (with warning). */
        const char *s = uinput_chmod_cmd;
        if (strncmp(s, "chmod ", 6) == 0)
            s += 6;
        while (*s == ' ')
            s++;
        char *endptr = NULL;
        long mode = strtol(s, &endptr, 8);
        if (endptr != s && (*endptr == '\0' || isspace((unsigned char)*endptr)))
        {
            if (chmod("/dev/uinput", (mode_t)mode) != 0)
            {
                rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: chmod /dev/uinput failed: %s\n", strerror(errno));
            }
            else
            {
                rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: chmod /dev/uinput %o OK\n", (unsigned)mode);
            }
        }
        else
        {
            rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: uinput_chmod_cmd not an octal mode, falling back to system(): '%s'\n", uinput_chmod_cmd);
            if (system(uinput_chmod_cmd) == -1)
            {
                rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: system(%s) failed: %s\n", uinput_chmod_cmd, strerror(errno));
            }
        }
    }
    else
    {
        rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: Automatic /dev/uinput permission change disabled by user.\n");
    }
    addr->uinput_fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (addr->uinput_fd < 0)
    {
        // rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Error opening /dev/uinput. Keyboard device disabled (Permissions?).\n");
        if (verbose >= VERBOSE_USB)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Error opening /dev/uinput (Code %d): %s\n", errno, strerror(errno));
        }
        addr->uinput_fd = -1;
        return -1; // Return critical error
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: Opening /dev/uinput OK. Configuring keyboard...\n");

    // 2. Enable EV_KEY and EV_SYN event types
    if (ioctl(addr->uinput_fd, UI_SET_EVBIT, EV_KEY) < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ioctl UI_SET_EVBIT EV_KEY Error.\n");
        retval = -1;
    }
    if (ioctl(addr->uinput_fd, UI_SET_EVBIT, EV_SYN) < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ioctl UI_SET_EVBIT EV_SYN Error.\n");
        retval = -1;
    }

    // 3. DYNAMIC ENABLE OF KEYCODES
    for (i = 0; i < keycode_number; i++)
    {
        uint32_t current_keycode = in_bit_keycode_value[i];

        if (current_keycode > 0)
        {
            if (ioctl(addr->uinput_fd, UI_SET_KEYBIT, current_keycode) < 0)
            {
                rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ioctl UI_SET_KEYBIT error for keycode %d.\n", current_keycode);
                // Do not use retval here; failure on a single key is not critical
            }
        }
    }

    // 4. Configure device
    memset(&uidev, 0, sizeof(uidev));
    snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "LinuxCNC_io_decoder");
    uidev.id.bustype = BUS_USB;
    uidev.id.vendor = 0x1A2B;
    uidev.id.product = 0x3C4D;
    uidev.id.version = 1;

    // 5. Write configuration and create device
    if (write(addr->uinput_fd, &uidev, sizeof(uidev)) < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Uinput write error.\n");
        retval = -1;
    }
    if (ioctl(addr->uinput_fd, UI_DEV_CREATE) < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ioctl UI_DEV_CREATE Error.\n");
        retval = -1;
    }
    else
    {
        rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: Virtual keyboard device 'LinuxCNC_io_decoder' created successfully.\n");
    }

    return retval;
}

// Servo thread function (HAL component function)
static void calc_io_decoder(void *arg, long period)
{
    hal_io_decoder_t *io = (hal_io_decoder_t *)arg;

    int i;
    int bufferSize_out;
    int bytesRead;
    int bufferSize_in;

    if ((io->usb_data_busy == 0) && (*(io->hal_comm_state) == HAL_STATE_COMMUNICATING))
    {

        // call output subroutines
        bufferSize_out = io->arduino_byte_npic_output;                                   // size of npic output buffer
        npic_out_routine(io, &io->usb_tx_payload[0], bufferSize_out);                    // starts at position 0 for number of npic outputs
        DAC_out_routine(io, &io->usb_tx_payload[bufferSize_out], io->arduino_bytes_dac); // starts after npic outputs for DAC bytes
        bufferSize_out = bufferSize_out + io->arduino_bytes_dac;
        out_expansion_routine(io, &io->usb_tx_payload[bufferSize_out], io->arduino_out_byte_expansion); // starts after DAC bytes for OUT_EXPANSION_BYTES

        // call input subroutines
        bytesRead = 0;
        bufferSize_in = io->arduino_byte_npic_input;
        bytesRead += npic_in_routine(io, &io->usb_rx_payload[0], bufferSize_in);                         // starts at position 0 for npic inputs
        bytesRead += encoder_routine(io, &io->usb_rx_payload[bufferSize_in], io->arduino_bytes_encoder); // starts after npic input for encoder bytes
        bufferSize_in = bufferSize_in + io->arduino_bytes_encoder;
        bytesRead += adc_routine(io, &io->usb_rx_payload[bufferSize_in], io->arduino_bytes_adc); // starts after encoder bytes for ADC bytes
        bufferSize_in = bufferSize_in + io->arduino_bytes_adc;
        bytesRead += in_expansion_routine(io, &io->usb_rx_payload[bufferSize_in], io->arduino_in_byte_expansion); // starts after ADC bytes for IN_EXPANSION bytes

        /*
        if (keyboard_initialized_ok == 1)
        {
                // key_simulation_loop(io);
                // keyboard simulation moved to non-realtime USB thread (usb_thread_function)
                // key_simulation_loop is intentionally NOT called here to keep this function RT-safe
        }
        */
    }
    else
    {
        for (i = 0; i < input; i++)
        {
            *(io->in_bit[i]) = 0;
        }
    }
}

/***********************************************************************
 * LOCAL FUNCTION DEFINITIONS                         *
 ************************************************************************/

int rtapi_app_main(void)
{
    // hal_io_decoder_t *io_decoder;
    rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: rtapi_app_main started.\n");

    int n, retval, i;
    int howmany;
    int somma_canali;
    int maxchan;
    int inchan;
    int outchan;
    int minchan;

    n = 0;

    hal_io_decoder_t *addr;
    //    hal_io_decoder_t *io = &(io_decoder_array[n]);

    struct uinput_user_dev uidev;

    maxchan = MAX_CHAN;
    minchan = MIN_CHAN / 2;
    inchan = MAX_INPUT;
    outchan = MAX_OUTPUT;

    switch (firmware)
    {
    case 255:
        // Set specific values for firmware 255 (demo/eval)
        input = 4;
        output = 4;
        break;

    default:
        somma_canali = input + output;
        if (somma_canali > maxchan)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "sum of input + output must be max %d \n", maxchan);
            return -EINVAL;
        }

        if (input % 8 != 0 || input < minchan || input > inchan)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "input is %d and must be a multiple of 8, between %d and %d.\n",
                            input, minchan, inchan);
            return -EINVAL;
        }

        if (output % 8 != 0 || output < minchan || output > outchan)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "output is %d and must be a multiple of 8, between %d and %d.\n",
                            output, minchan, outchan);
            return -EINVAL;
        }
    }

    if ((verbose > VERBOSE_ALL) || (verbose < VERBOSE_NULL))
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "value allowed %d to %d. inserted %d \n", VERBOSE_NULL, VERBOSE_ALL, verbose);
        return -EINVAL;
    }

    // Check if HAL provided a valid path from .ini
    if (keymap_file != NULL && strlen(keymap_file) > 0)
    {

        // Call utility to parse the file.
        // Parameter is path read from .ini.
        retval = load_key_map(keymap_file);

        if (retval != 0)
        {
            // load_key_map returned error (e.g. file not found)
            rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Fatal error loading keycodes mapping file.\n");
            // In a HAL component better to exit on critical configuration error.
            hal_exit(comp_id);
            return -1;
        }
    }
    else
    {
        rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: No path specified for 'keymap-filename'. Using default mapping (0).\n");
    }

    // have good config info, connect to the HAL
    comp_id = hal_init("io_decoder");
    if (comp_id < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ERROR: hal_init() failed\n");
        return -1;
    }

    howmany = 1;
    io_decoder_array = hal_malloc(howmany * sizeof(hal_io_decoder_t));
    if (io_decoder_array == 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "io_decoder: ERROR: hal_malloc() failed\n");
        hal_exit(comp_id);
        hal_shutdown_flag = 1;
        return -1;
    }

    addr = &(io_decoder_array[0]);

    // Zero allocated memory for the struct (good practice)
    memset(&(io_decoder_array[n]), 0, sizeof(hal_io_decoder_t));

    /* Ensure file descriptors and thread id start as invalid values */
    io_decoder_array[n].uinput_fd = -1;
    io_decoder_array[n].serial_fd = -1;
    io_decoder_array[n].usb_thread_id = -1;

    io_decoder_array[n].input = input;   // Copy from global static 'input'
    io_decoder_array[n].output = output; // Copy from global static 'output'
    // Copy USB port name safely
    strncpy(io_decoder_array[n].usb_port_name, usb_port_name, sizeof(io_decoder_array[n].usb_port_name) - 1);
    io_decoder_array[n].usb_port_name[sizeof(io_decoder_array[n].usb_port_name) - 1] = '\0'; // Ensure null termination
    io_decoder_array[n].firmware = firmware;                                                 // Copy firmware value

    // Pass module parameters to component struct

    switch (firmware)
    {
    case 101:
        // Set values specific to firmware 101
        io_decoder_array[n].firmware_firmware = 101; // Example: new struct member
        io_decoder_array[n].firmware_n_encoder = 2;  // then 4
        io_decoder_array[n].firmware_n_dac = 2;
        io_decoder_array[n].firmware_n_adc = 3;
        io_decoder_array[n].firmware_in_bit_expansion = 8;
        io_decoder_array[n].firmware_out_bit_expansion = 8;
        break;

    case 102:
        // Set values specific to firmware 102
        io_decoder_array[n].firmware_firmware = 102; // Example: new struct member
        io_decoder_array[n].firmware_n_encoder = 2;  // then 4
        io_decoder_array[n].firmware_n_dac = 2;
        io_decoder_array[n].firmware_n_adc = 1;
        io_decoder_array[n].firmware_in_bit_expansion = 8;
        io_decoder_array[n].firmware_out_bit_expansion = 8;
        break;

    case 255:
        // Set values specific to firmware 255
        io_decoder_array[n].firmware_firmware = 255; // Example: new struct member
        io_decoder_array[n].firmware_n_encoder = 1;  // then 4
        io_decoder_array[n].firmware_n_dac = 1;
        io_decoder_array[n].firmware_n_adc = 1;
        io_decoder_array[n].firmware_in_bit_expansion = 8;
        io_decoder_array[n].firmware_out_bit_expansion = 8;
        break;

    default:
        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ERROR: Firmware %d not supported.\n", firmware);
        hal_exit(comp_id);
        hal_shutdown_flag = 1;
        return -EINVAL;
    }

    // ***************************************************************
    // CALL UINPUT SUBROUTINE
    // ***************************************************************
    if (keyboard_initialized_ok == 1)
    {
        if (init_uinput(addr) == 0)
        {
            keyboard_initialized_ok = 1;
        }
        else
        {
            keyboard_initialized_ok = 0;
            if (verbose >= VERBOSE_USB)
                rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Uinput initialization failed. Keyboard simulation will NOT work.\n");
        }
    }

    /****************************************************************************************************************
     * **************************************************************************************************************
     * read .ini file before this point *******************************************************************
     * **************************************************************************************************************
     * *************************************************************************************************************/

    // export variables and functions for each io_decoder
    char buf[HAL_NAME_LEN + 1];
    rtapi_snprintf(buf, sizeof(buf), "io_decoder");
    retval = export_io_decoder(n, &(io_decoder_array[n]), buf);
    if (retval < 0)
    { // Check return value of export_io_decoder
        hal_exit(comp_id);
        hal_shutdown_flag = 1;
        return retval;
    }

    // ADDED: Create the USB communication thread
    io_decoder_array[n].usb_thread_id = rtapi_task_new(
        usb_thread_function,
        &(io_decoder_array[n]),
        USB_THREAD_PRIORITY,
        comp_id,
        USB_THREAD_STACKSIZE,
        RTAPI_NO_FP);

    if (io_decoder_array[n].usb_thread_id < 0)
    {

        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ERROR: Failed to create USB thread\n");
        hal_exit(comp_id);
        hal_shutdown_flag = 1;
        return -1;
    }
    else
    { // ALSO ADD SUCCESS MESSAGE
        rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: USB thread created successfully with ID: %d\n", io_decoder_array[n].usb_thread_id);
        rtapi_task_start(io_decoder_array[n].usb_thread_id, 0);
        rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: USB Task started.\n");
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: installed %d input and %d output\n", input, output);
    hal_ready(comp_id);

    return 0;
}

void rtapi_app_exit(void)
{
    rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: rtapi_app_exit started.\n");

    hal_shutdown_flag = 1;

    if (io_decoder_array != NULL)
    {
        if (io_decoder_array[0].usb_thread_id >= 0)
        {
            rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: Deleting USB thread id %d\n", io_decoder_array[0].usb_thread_id);
            rtapi_task_delete(io_decoder_array[0].usb_thread_id);
            /* wait up to 500 ms for thread to clear running flag */
            int wait_ms = 0;
            while (io_decoder_array[0].usb_thread_running && wait_ms < 500)
            {
                rtapi_delay(1000); // 1 ms
                wait_ms++;
            }
            if (io_decoder_array[0].usb_thread_running)
                rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: USB thread did not exit within timeout.\n");
        }

        if (io_decoder_array[0].uinput_fd >= 0)
        {
            rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: Destroying uinput device.\n");
            ioctl(io_decoder_array[0].uinput_fd, UI_DEV_DESTROY);
            close(io_decoder_array[0].uinput_fd);
            io_decoder_array[0].uinput_fd = -1;
        }

        if (io_decoder_array[0].serial_fd >= 0)
        {
            close(io_decoder_array[0].serial_fd);
            io_decoder_array[0].serial_fd = -1;
        }

        hal_free(io_decoder_array);
        io_decoder_array = NULL;
    }

    if (comp_id >= 0)
    {
        hal_exit(comp_id);
        comp_id = -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: rtapi_app_exit finished.\n");
}

/***********************************************************************
 * LOCAL FUNCTION DEFINITIONS                         *
 ************************************************************************/

static int export_io_decoder(int num, hal_io_decoder_t *addr, char *prefix)
{
    int retval;
    int i = 0;
    int j = 0;
    int k = 0;

    addr->usb_tx_payload_len = 0; // Actual payload length in tx buffer
    addr->usb_rx_payload_len = 0; // Actual payload length in rx buffer

    for (i = 0; i < MAX_RX_PAYLOAD_SIZE; i++)
    {
        addr->serial_rx_buffer[i] = 0;
    }
    addr->serial_rx_buffer_len = 0;
    for (i = 0; i < MAX_TX_PAYLOAD_SIZE; i++)
    {
        addr->tx_packet_buffer[i] = 0;
    }

    // export pins

    // IO_encoder section ##############################################

    for (i = 0; i < input; i++)
    {
        //		rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: i = %d.\n", i);
        if (key_input_map[i].key_type == KEY_TYPE_UNUSED)
        {
            retval = hal_pin_bit_newf(HAL_OUT, &(addr->in_bit[i]), comp_id,
                                      "%s.in.%02i-%01i", prefix, j, k);
            if (retval != 0)
            {
                return retval;
            }
            retval = hal_pin_bit_newf(HAL_OUT, &(addr->in_bit_t[i]), comp_id,
                                      "%s.in.%02i-%01i.toggle", prefix, j, k);
            if (retval != 0)
            {
                return retval;
            }
            *(addr->in_bit_t[i]) = 0;
        }
        else
        {
            retval = hal_pin_bit_newf(HAL_OUT, &(addr->in_bit[i]), comp_id,
                                      "%s.in.%02i-%01i-keyboard", prefix, j, k);
        }
        k++;
        if (k == 8)
        {
            k = 0;
            j++;
        }

        *(addr->in_bit[i]) = 0;
        in_bit_d[i] = 0;
        in_bit_t_state[i] = 0;
    }

    i = 0;
    j = 0;
    k = 0;

    for (i = 0; i < output; i++)
    {
        retval = hal_pin_bit_newf(HAL_IN, &(addr->out_bit[i]), comp_id,
                                  "%s.out.%02i-%01i", prefix, j, k);
        retval = hal_pin_float_newf(HAL_IN, &(addr->out_bit_blink_freq[i]), comp_id,
                                    "%s.out.%02i-%01i.blink-freq", prefix, j, k);
        retval = hal_pin_bit_newf(HAL_IN, &(addr->out_bit_blink_en[i]), comp_id,
                                  "%s.out.%02i-%01i.blink-en", prefix, j, k);
        retval = hal_pin_float_newf(HAL_IN, &(addr->out_bit_blink_width[i]), comp_id,
                                    "%s.out.%02i-%01i.blink-width", prefix, j, k);
        k++;
        if (k == 8)
        {
            k = 0;
            j++;
        }

        if (retval != 0)
        {
            return retval;
        }
        *(addr->out_bit[i]) = 0;
        *(addr->out_bit_blink_freq[i]) = 1;
        *(addr->out_bit_blink_en[i]) = 0;
        *(addr->out_bit_blink_width[i]) = 0.5;
        addr->out_blink_time_ns[i] = 0;
    }

    // encoder jog section ##############################################

    for (i = 0; i < addr->firmware_n_encoder; i++)
    {

        retval = hal_pin_s32_newf(HAL_OUT, &(addr->enc[i]), comp_id, // Using array for enc
                                  "%s.enc.%01i", prefix, i);
        if (retval != 0)
        {
            return retval;
        }
        *(addr->enc[i]) = 0;

        retval = hal_pin_bit_newf(HAL_IN, &(addr->enc_invert[i]), comp_id, // Using array for enc_invert
                                  "%s.enc.%01i.invert", prefix, i);
        if (retval != 0)
        {
            return retval;
        }
        *(addr->enc_invert[i]) = 0;

        retval = hal_pin_bit_newf(HAL_OUT, &(addr->enc_up[i]), comp_id, // Using array for enc_up
                                  "%s.enc.%01i.up", prefix, i);
        if (retval != 0)
        {
            return retval;
        }
        *(addr->enc_up[i]) = 0;

        retval = hal_pin_bit_newf(HAL_OUT, &(addr->enc_down[i]), comp_id, // Using array for enc_down
                                  "%s.enc.%01i.down", prefix, i);
        if (retval != 0)
        {
            return retval;
        }
        *(addr->enc_down[i]) = 0;

        addr->enc_delta[i] = 0;
        addr->enc_delta_sign[i] = 0;
    }

    // DAC section ##############################################

    for (i = 0; i < addr->firmware_n_dac; i++)
    {
        retval = hal_pin_float_newf(HAL_IN, &(addr->dac[i]), comp_id, // Using array for dac
                                    "%s.dac.%01i", prefix, i);
        if (retval != 0)
        {
            return retval;
        }
        *(addr->dac[i]) = 0;

        retval = hal_pin_float_newf(HAL_IN, &(addr->dac_scale[i]), comp_id, // Using array for dac
                                    "%s.dac.%01i.scale", prefix, i);
        if (retval != 0)
        {
            return retval;
        }
        *(addr->dac_scale[i]) = 0;

        retval = hal_pin_bit_newf(HAL_IN, &(addr->dac_invert[i]), comp_id, // Using array for dac
                                  "%s.dac.%01i.invert", prefix, i);
        if (retval != 0)
        {
            return retval;
        }
        *(addr->dac_invert[i]) = 0;
    }

    // ADC section ##############################################

    for (i = 0; i < addr->firmware_n_adc; i++)
    {
        retval = hal_pin_float_newf(HAL_OUT, &(addr->adc[i]), comp_id, // Using array for dac
                                    "%s.adc.%01i", prefix, i);
        if (retval != 0)
        {
            return retval;
        }
        *(addr->adc[i]) = 0;

        retval = hal_pin_float_newf(HAL_IN, &(addr->adc_scale[i]), comp_id, // Using array for dac
                                    "%s.adc.%01i.scale", prefix, i);
        if (retval != 0)
        {
            return retval;
        }
        *(addr->adc_scale[i]) = 0;

        retval = hal_pin_bit_newf(HAL_IN, &(addr->adc_invert[i]), comp_id, // Using array for dac
                                  "%s.adc.%01i.invert", prefix, i);
        if (retval != 0)
        {
            return retval;
        }
        *(addr->adc_invert[i]) = 0;

        retval = hal_pin_u32_newf(HAL_OUT, &(addr->adc_raw[i]), comp_id, // Using array for dac
                                  "%s.adc.%01i.raw", prefix, i);
        if (retval != 0)
        {
            return retval;
        }
        *(addr->adc_raw[i]) = 0;

        retval = hal_pin_float_newf(HAL_IN, &(addr->adc_joy_center[i]), comp_id, // Using array for dac
                                    "%s.adc.%01i.joy.center", prefix, i);
        if (retval != 0)
        {
            return retval;
        }
        *(addr->adc_joy_center[i]) = 0;

        retval = hal_pin_float_newf(HAL_IN, &(addr->adc_joy_deadb[i]), comp_id, // Using array for dac
                                    "%s.adc.%01i.joy.deadband", prefix, i);
        if (retval != 0)
        {
            return retval;
        }
        *(addr->adc_joy_deadb[i]) = 0;

        retval = hal_pin_float_newf(HAL_IN, &(addr->adc_joy_factor[i]), comp_id, // Using array for dac
                                    "%s.adc.%01i.joy.factor", prefix, i);
        if (retval != 0)
        {
            return retval;
        }
        *(addr->adc_joy_factor[i]) = 0.5;

        retval = hal_pin_bit_newf(HAL_OUT, &(addr->adc_joy_pulse_up[i]), comp_id, // Using array for dac
                                  "%s.adc.%01i.joy.pulse.up", prefix, i);
        if (retval != 0)
        {
            return retval;
        }
        *(addr->adc_joy_pulse_up[i]) = 0;

        retval = hal_pin_bit_newf(HAL_OUT, &(addr->adc_joy_pulse_down[i]), comp_id, // Using array for dac
                                  "%s.adc.%01i.joy.pulse.down", prefix, i);
        if (retval != 0)
        {
            return retval;
        }
        *(addr->adc_joy_pulse_down[i]) = 0;

        retval = hal_pin_s32_newf(HAL_OUT, &(addr->adc_joy_count[i]), comp_id, // Using array for dac
                                  "%s.adc.%01i.joy.count", prefix, i);
        if (retval != 0)
        {
            return retval;
        }
        *(addr->adc_joy_count[i]) = 0;

        addr->old_adc_raw[i] = 0;
        addr->adc_raw_joy[i] = 0;
        addr->next_adc_joy_ns[i] = 0;
        addr->adc_joy_valid[i] = 0;
    }

    // accessories section ##############################################

    retval = hal_pin_bit_newf(HAL_OUT, &(addr->led), comp_id,
                              "%s.diag.led", prefix);
    if (retval != 0)
    {
        return retval;
    }
    *(addr->led) = 0;

    retval = hal_pin_float_newf(HAL_OUT, &(addr->debug_f), comp_id,
                                "%s.diag.debug-f", prefix); // Use prefix directly
    if (retval != 0)
    {
        return retval;
    }
    *(addr->debug_f) = 0.0f;

    retval = hal_pin_float_newf(HAL_OUT, &(addr->debug_f2), comp_id,
                                "%s.diag.debug-f2", prefix); // Use prefix directly
    if (retval != 0)
    {
        return retval;
    }
    *(addr->debug_f2) = 0.0f;

    retval = hal_pin_s32_newf(HAL_IN, &(addr->servo_thread_time), comp_id,
                              "%s.diag.servo-thread-time", prefix); // Use prefix directly
    if (retval != 0)
    {
        return retval;
    }
    *(addr->servo_thread_time) = 0;

    retval = hal_pin_s32_newf(HAL_OUT, &(addr->servo_jitter_pin), comp_id,
                              "%s.diag.servo-thread-jitter", prefix); // Use prefix directly
    if (retval != 0)
    {
        return retval;
    }
    *(addr->servo_jitter_pin) = 0;

    retval = hal_pin_s32_newf(HAL_IN, &(addr->base_thread_time), comp_id,
                              "%s.diag.base-thread-time", prefix); // Use prefix directly
    if (retval != 0)
    {
        return retval;
    }
    *(addr->base_thread_time) = 0;

    retval = hal_pin_s32_newf(HAL_OUT, &(addr->base_jitter_pin), comp_id,
                              "%s.diag.base-thread-jitter", prefix); // Use prefix directly
    if (retval != 0)
    {
        return retval;
    }
    *(addr->base_jitter_pin) = 0;

    // ADDED: Export HAL pins for communication state and error count
    retval = hal_pin_s32_newf(HAL_OUT, &(addr->hal_comm_state), comp_id,
                              "%s.diag.comm-state", prefix); // Use prefix directly
    if (retval != 0)
    {
        return retval;
    }
    *(addr->hal_comm_state) = HAL_STATE_HANDSHAKE;

    retval = hal_pin_float_newf(HAL_OUT, &(addr->hal_error_count), comp_id,
                                "%s.diag.error-count", prefix); // Use prefix directly
    if (retval != 0)
    {
        return retval;
    }
    *(addr->hal_error_count) = 0.0f;

    retval = hal_pin_float_newf(HAL_OUT, &(addr->parse_error_percentile), comp_id,
                                "%s.diag.parse-error", prefix); // Use prefix directly
    if (retval != 0)
    {
        return retval;
    }
    *(addr->parse_error_percentile) = 0.0f;

    retval = hal_pin_bit_newf(HAL_OUT, &(addr->usb_connected), comp_id,
                              "%s.diag.usb-connected", prefix); // Use prefix directly
    if (retval != 0)
    {
        return retval;
    }
    *(addr->usb_connected) = 0;

    retval = hal_pin_s32_newf(HAL_OUT, &(addr->jitter_usb_comm), comp_id,
                              "%s.diag.usb-communication-jitter", prefix); // Use prefix directly
    if (retval != 0)
    {
        return retval;
    }
    *(addr->jitter_usb_comm) = 0;

    retval = hal_pin_s32_newf(HAL_OUT, &(addr->jitter_usb_loop), comp_id,
                              "%s.diag.usb-thread-jitter", prefix); // Use prefix directly
    if (retval != 0)
    {
        return retval;
    }
    *(addr->jitter_usb_loop) = 0;

    // export function for this loop

    char temp_name[HAL_NAME_LEN + 1];
    rtapi_snprintf(temp_name, sizeof(temp_name), "%s.update", prefix);

    retval =
        hal_export_funct(temp_name, calc_io_decoder, &(io_decoder_array[num]), 0, 0,
                         comp_id);
    if (retval != 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "io_decoder: ERROR: update funct export failed\n");
        hal_exit(comp_id);
        hal_shutdown_flag = 1;
        return -1;
    }
    return 0;
}