/**
 * @file soldered_adsb_receiver.h
 * @brief Public API for the soldered-adsb-receiver component
 *
 * ESP-IDF driver for the Soldered ADS-B 1090MHz Receiver, built around the
 * Aerobits TT-SC1b OEM ADS-B receiver module. The module listens for the
 * squitters aircraft broadcast on 1090 MHz and reports every aircraft's
 * position, altitude, speed, callsign and squawk over UART, either as CSV
 * lines or as MAVLink frames. It is configured over the same UART with AT
 * commands.
 *
 * The driver owns a UART port and takes care of three things: the AT command
 * layer and the RUN / CONFIGURATION / BOOTLOADER state machine, decoding of
 * the CSV stream, and decoding of the MAVLink stream. adsb_receiver_poll()
 * tells CSV from MAVLink byte by byte, so the same application keeps working
 * whichever output protocol the module is set to.
 *
 * @author Soldered Electronics
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "driver/uart.h"
#include "esp_err.h"
#include "adsb_receiver_dfs.h"
#include "adsb_receiver_mavlink.h"

/**
 * @brief Callback for a decoded "#A:" aircraft message
 *
 * @param[in] aircraft Decoded aircraft, only valid for the duration of the call
 * @param[in] arg The pointer handed to adsb_receiver_set_aircraft_callback()
 */
typedef void (*adsb_receiver_aircraft_cb_t)(const adsb_receiver_aircraft_t *aircraft, void *arg);

/**
 * @brief Callback for a decoded "#AS:" ADS-B statistics message
 *
 * @param[in] stats Decoded statistics, only valid for the duration of the call
 * @param[in] arg The pointer handed to adsb_receiver_set_adsb_stats_callback()
 */
typedef void (*adsb_receiver_adsb_stats_cb_t)(const adsb_receiver_adsb_stats_t *stats, void *arg);

/**
 * @brief Callback for a decoded "#S:" system statistics message
 *
 * @param[in] stats Decoded statistics, only valid for the duration of the call
 * @param[in] arg The pointer handed to adsb_receiver_set_system_stats_callback()
 */
typedef void (*adsb_receiver_system_stats_cb_t)(const adsb_receiver_system_stats_t *stats, void *arg);

/**
 * @brief Callback for a received line of text
 *
 * @param[in] line Line without its line ending, only valid for the duration of the call
 * @param[in] arg The pointer handed along with the callback
 */
typedef void (*adsb_receiver_line_cb_t)(const char *line, void *arg);

/**
 * @brief Callback for a received MAVLink frame
 *
 * @param[in] frame Raw frame, only valid for the duration of the call
 * @param[in] arg The pointer handed to adsb_receiver_set_mavlink_frame_callback()
 */
typedef void (*adsb_receiver_mavlink_frame_cb_t)(const adsb_receiver_mavlink_frame_t *frame, void *arg);

/**
 * @brief Callback for a decoded MAVLink ADSB_VEHICLE message
 *
 * @param[in] vehicle Decoded vehicle, only valid for the duration of the call
 * @param[in] arg The pointer handed to adsb_receiver_set_adsb_vehicle_callback()
 */
typedef void (*adsb_receiver_adsb_vehicle_cb_t)(const adsb_receiver_adsb_vehicle_t *vehicle, void *arg);

/**
 * @brief How the module is wired up
 *
 * Use ::ADSB_RECEIVER_DEFAULT_CONFIG to fill this in for the Soldered board and
 * then change what differs on custom wiring.
 */
typedef struct {
    uart_port_t uart_port; /**< UART peripheral to drive the module with, avoid the console UART */
    int rx_pin;            /**< Pin wired to the TX pin of the module */
    int tx_pin;            /**< Pin wired to the RX pin of the module */
    int reset_pin;         /**< Pin wired to the RESET pin of the module, or -1 if not connected */
    uint32_t baud_rate;    /**< Baud rate to start at, the module's BAUDRATE setting */
    int rx_buffer_size;    /**< Receive ring buffer in bytes, at least ::ADSB_RECEIVER_MIN_RX_BUFFER_SIZE */
} adsb_receiver_config_t;

/**
 * @brief Configuration for the Soldered ADS-B 1090MHz Receiver board
 *
 * UART1 on IO17 and IO18, RESET on IO21, 115200 baud.
 */
#define ADSB_RECEIVER_DEFAULT_CONFIG()                                                                                 \
    {                                                                                                                  \
        .uart_port = UART_NUM_1, .rx_pin = ADSB_RECEIVER_DEFAULT_RX_PIN, .tx_pin = ADSB_RECEIVER_DEFAULT_TX_PIN,       \
        .reset_pin = ADSB_RECEIVER_DEFAULT_RESET_PIN, .baud_rate = ADSB_RECEIVER_DEFAULT_BAUD_RATE,                    \
        .rx_buffer_size = ADSB_RECEIVER_DEFAULT_RX_BUFFER_SIZE                                                         \
    }

/**
 * @brief Handle for one module
 *
 * Create one per module. All fields are managed by the driver; treat the struct
 * as opaque. The driver is not thread safe, so use a handle from one task only.
 */
typedef struct {
    uart_port_t uart_port;                           /**< UART port opened by adsb_receiver_init() */
    int reset_pin;                                   /**< RESET pin, or -1 if there is none */
    char line_buf[ADSB_RECEIVER_MAX_LINE_LEN + 1];   /**< Line being assembled */
    size_t line_len;                                 /**< Characters in line_buf */
    adsb_receiver_mavlink_parser_t mavlink;          /**< MAVLink frame parser */
    adsb_receiver_aircraft_cb_t aircraft_cb;         /**< Callback for "#A:" messages, or NULL */
    void *aircraft_arg;                              /**< Argument passed to aircraft_cb */
    adsb_receiver_adsb_stats_cb_t adsb_stats_cb;     /**< Callback for "#AS:" messages, or NULL */
    void *adsb_stats_arg;                            /**< Argument passed to adsb_stats_cb */
    adsb_receiver_system_stats_cb_t system_stats_cb; /**< Callback for "#S:" messages, or NULL */
    void *system_stats_arg;                          /**< Argument passed to system_stats_cb */
    adsb_receiver_line_cb_t unknown_line_cb;         /**< Callback for unrecognized lines, or NULL */
    void *unknown_line_arg;                          /**< Argument passed to unknown_line_cb */
    adsb_receiver_line_cb_t line_cb;                 /**< Callback for every line, or NULL */
    void *line_arg;                                  /**< Argument passed to line_cb */
    adsb_receiver_mavlink_frame_cb_t mavlink_frame_cb; /**< Callback for MAVLink frames, or NULL */
    void *mavlink_frame_arg;                         /**< Argument passed to mavlink_frame_cb */
    adsb_receiver_adsb_vehicle_cb_t adsb_vehicle_cb; /**< Callback for ADSB_VEHICLE messages, or NULL */
    void *adsb_vehicle_arg;                          /**< Argument passed to adsb_vehicle_cb */
} adsb_receiver_t;

// *****************************************************************************
// Section: Lifetime

/**
 * @brief Open the UART link to the module
 *
 * Installs the UART driver on the port named in `config` and wires it to the
 * given pins at 8N1. The RESET pin, if any, is left as a high impedance input:
 * only adsb_receiver_hardware_reset() drives it, and only for the length of
 * the pulse.
 *
 * @param[out] dev Handle to initialize
 * @param[in] config How the module is wired up
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument or a
 *         receive buffer below ::ADSB_RECEIVER_MIN_RX_BUFFER_SIZE, or the error
 *         returned by the UART or GPIO driver
 */
esp_err_t adsb_receiver_init(adsb_receiver_t *dev, const adsb_receiver_config_t *config);

/**
 * @brief Close the UART link
 *
 * @param[in,out] dev Handle previously initialized with adsb_receiver_init()
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL handle, or the error
 *         returned by uart_driver_delete()
 */
esp_err_t adsb_receiver_deinit(adsb_receiver_t *dev);

/**
 * @brief Reset the module by pulsing its RESET pin
 *
 * Afterwards the module restarts: it sits in the BOOTLOADER state at a fixed
 * 115200 baud for about 3 s and then moves on to RUN on its own, unless it is
 * locked. Anything received before the reset is thrown away.
 *
 * @param[in,out] dev Handle
 * @param[in] pulse_ms How long to hold the module in reset
 * @param[in] active_low true if the module resets while RESET is driven low, as it normally does
 * @param[in] resync_to_115200 Also switch the local UART to 115200 baud, which
 *            is what the module comes back up at, so the next command is not
 *            sent at a stale baud rate
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL handle,
 *         ESP_ERR_NOT_SUPPORTED if no RESET pin was configured, or the error
 *         returned by the GPIO or UART driver
 */
esp_err_t adsb_receiver_hardware_reset(adsb_receiver_t *dev, uint32_t pulse_ms, bool active_low,
                                       bool resync_to_115200);

// *****************************************************************************
// Section: AT command transport

/**
 * @brief Send one AT command and collect the reply
 *
 * Sends `AT+<body>\r\n` and collects every `AT+` line the module sends back,
 * joined with '\n', until a terminal line arrives. Terminal lines are AT+OK,
 * AT+ERROR and the state markers AT+CONFIG_START, AT+RUN_START and
 * AT+BOOTLOADER_START. A query, a body ending in '?', only gets a single
 * `AT+NAME=value` reply, so for a query the first reply line is terminal.
 *
 * Non-AT lines received while waiting, such as CSV traffic still in flight
 * during a RUN to CONFIGURATION transition, are handed to the data callbacks
 * rather than thrown away.
 *
 * @param[in,out] dev Handle
 * @param[in] body Command without the leading "AT+" and without a line ending,
 *            such as "SERIAL_NUMBER?" or "SYSTEM_LOG=1"
 * @param[out] response Buffer for the reply lines, or NULL if not wanted
 * @param[in] response_size Size of the buffer; a longer reply is cut short
 * @param[in] timeout_ms How long to wait for a terminal line
 * @param[in] wait_for_prefix If not NULL, keep waiting past a generic AT+OK and
 *            only stop on a line starting with this prefix, or on AT+ERROR
 *
 * @return ESP_OK if a terminal line arrived, ESP_ERR_INVALID_ARG on a NULL
 *         argument or a body longer than ::ADSB_RECEIVER_MAX_COMMAND_LEN, or
 *         ESP_ERR_TIMEOUT if none arrived in time
 */
esp_err_t adsb_receiver_send_at(adsb_receiver_t *dev, const char *body, char *response, size_t response_size,
                                uint32_t timeout_ms, const char *wait_for_prefix);

/**
 * @brief Check whether a reply holds AT+OK
 *
 * @param[in] response Reply collected by adsb_receiver_send_at()
 *
 * @return true if the reply holds AT+OK
 */
bool adsb_receiver_is_ok(const char *response);

/**
 * @brief Check whether a reply holds AT+ERROR
 *
 * @param[in] response Reply collected by adsb_receiver_send_at()
 * @param[out] description Buffer for the text in parentheses after AT+ERROR, or NULL if not wanted
 * @param[in] description_size Size of the description buffer
 *
 * @return true if the reply holds AT+ERROR
 */
bool adsb_receiver_is_error(const char *response, char *description, size_t description_size);

// *****************************************************************************
// Section: State machine

/**
 * @brief Move from RUN to CONFIGURATION
 *
 * Sends AT+CONFIG=1, switches the local UART to 115200 baud, the fixed baud
 * rate of the CONFIGURATION state, and confirms the change with AT+CONFIG?.
 * Settings can only be read and written in CONFIGURATION.
 *
 * @param[in,out] dev Handle
 *
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if the module did not answer,
 *         ESP_FAIL if it answered with AT+ERROR, or ESP_ERR_INVALID_STATE if it
 *         did not end up in CONFIGURATION
 */
esp_err_t adsb_receiver_enter_configuration(adsb_receiver_t *dev);

/**
 * @brief Move from CONFIGURATION back to RUN
 *
 * Sends AT+CONFIG=0, switches the local UART to `run_baud` and confirms the
 * change with AT+CONFIG?.
 *
 * @param[in,out] dev Handle
 * @param[in] run_baud Baud rate of the RUN state, must match the BAUDRATE setting
 *
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if the module did not answer,
 *         ESP_FAIL if it answered with AT+ERROR, or ESP_ERR_INVALID_STATE if it
 *         did not end up in RUN
 */
esp_err_t adsb_receiver_exit_configuration(adsb_receiver_t *dev, uint32_t run_baud);

/**
 * @brief Read AT+CONFIG?, the state the module is in
 *
 * @param[in,out] dev Handle
 * @param[out] state One of ::adsb_receiver_config_state_t
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument,
 *         ESP_ERR_TIMEOUT if the module did not answer, or
 *         ESP_ERR_INVALID_RESPONSE if the reply could not be read
 */
esp_err_t adsb_receiver_get_config_state(adsb_receiver_t *dev, uint8_t *state);

/**
 * @brief Reboot into the BOOTLOADER state, AT+REBOOT_BOOTLOADER
 *
 * Also sets the lock, so the module stays in BOOTLOADER afterwards until
 * adsb_receiver_set_lock() releases it.
 *
 * @param[in,out] dev Handle
 *
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if the module did not answer, or
 *         ESP_FAIL if it did not confirm
 */
esp_err_t adsb_receiver_reboot_to_bootloader(adsb_receiver_t *dev);

/**
 * @brief Reboot the module, AT+REBOOT
 *
 * @param[in,out] dev Handle
 *
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if the module did not answer,
 *         ESP_FAIL if it answered with AT+ERROR, or ESP_ERR_INVALID_RESPONSE on
 *         any other reply
 */
esp_err_t adsb_receiver_reboot(adsb_receiver_t *dev);

/**
 * @brief Set or release the lock that holds the module in BOOTLOADER, AT+LOCK
 *
 * @param[in,out] dev Handle
 * @param[in] locked true to lock
 *
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if the module did not answer,
 *         ESP_FAIL if it answered with AT+ERROR, or ESP_ERR_INVALID_RESPONSE on
 *         any other reply
 */
esp_err_t adsb_receiver_set_lock(adsb_receiver_t *dev, bool locked);

/**
 * @brief Read the lock, AT+LOCK?
 *
 * @param[in,out] dev Handle
 * @param[out] locked true if the module is locked
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument,
 *         ESP_ERR_TIMEOUT if the module did not answer, or
 *         ESP_ERR_INVALID_RESPONSE if the reply could not be read
 */
esp_err_t adsb_receiver_get_lock(adsb_receiver_t *dev, bool *locked);

/**
 * @brief Check whether the module is in the BOOTLOADER state, AT+BOOT?
 *
 * @param[in,out] dev Handle
 * @param[out] in_bootloader true if the module is in BOOTLOADER
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument,
 *         ESP_ERR_TIMEOUT if the module did not answer, or
 *         ESP_ERR_INVALID_RESPONSE if the reply could not be read
 */
esp_err_t adsb_receiver_is_bootloader(adsb_receiver_t *dev, bool *in_bootloader);

// *****************************************************************************
// Section: Generic settings and commands
//
// All of these only work in the CONFIGURATION state, see
// adsb_receiver_enter_configuration().

/**
 * @brief Write a setting, AT+<name>=<value>
 *
 * @param[in,out] dev Handle
 * @param[in] name Setting name, such as "SYSTEM_LOG"
 * @param[in] value Value as text
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument,
 *         ESP_ERR_TIMEOUT if the module did not answer, ESP_FAIL if it answered
 *         with AT+ERROR, or ESP_ERR_INVALID_RESPONSE on any other reply
 */
esp_err_t adsb_receiver_set_setting(adsb_receiver_t *dev, const char *name, const char *value);

/**
 * @brief Read a setting, AT+<name>?
 *
 * @param[in,out] dev Handle
 * @param[in] name Setting name, such as "BAUDRATE"
 * @param[out] value Buffer for the value as text
 * @param[in] value_size Size of the buffer; a longer value is cut short
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument,
 *         ESP_ERR_TIMEOUT if the module did not answer, or
 *         ESP_ERR_INVALID_RESPONSE if the reply did not hold the setting
 */
esp_err_t adsb_receiver_get_setting(adsb_receiver_t *dev, const char *name, char *value, size_t value_size);

/**
 * @brief Read the description of a setting, AT+<name>=?
 *
 * @param[in,out] dev Handle
 * @param[in] name Setting name
 * @param[out] out Buffer for the reply lines
 * @param[in] out_size Size of the buffer
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument,
 *         ESP_ERR_TIMEOUT if nothing came back, or ESP_ERR_INVALID_SIZE if the
 *         reply did not fit and was cut short
 */
esp_err_t adsb_receiver_get_setting_description(adsb_receiver_t *dev, const char *name, char *out, size_t out_size);

/**
 * @brief List every setting with its value, AT+SETTINGS?
 *
 * @param[in,out] dev Handle
 * @param[out] out Buffer for the reply lines
 * @param[in] out_size Size of the buffer
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument,
 *         ESP_ERR_TIMEOUT if nothing came back, or ESP_ERR_INVALID_SIZE if the
 *         reply did not fit and was cut short
 */
esp_err_t adsb_receiver_get_all_settings(adsb_receiver_t *dev, char *out, size_t out_size);

/**
 * @brief Read the full help text, AT+HELP
 *
 * @param[in,out] dev Handle
 * @param[out] out Buffer for the reply lines
 * @param[in] out_size Size of the buffer
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument,
 *         ESP_ERR_TIMEOUT if nothing came back, or ESP_ERR_INVALID_SIZE if the
 *         reply did not fit and was cut short
 */
esp_err_t adsb_receiver_get_full_help(adsb_receiver_t *dev, char *out, size_t out_size);

/**
 * @brief Put every setting back to its default, AT+SETTINGS_DEFAULT
 *
 * @param[in,out] dev Handle
 *
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if the module did not answer,
 *         ESP_FAIL if it answered with AT+ERROR, or ESP_ERR_INVALID_RESPONSE on
 *         any other reply
 */
esp_err_t adsb_receiver_reset_settings_to_default(adsb_receiver_t *dev);

/**
 * @brief Read the serial number, AT+SERIAL_NUMBER?
 *
 * @param[in,out] dev Handle
 * @param[out] out Buffer for the serial number
 * @param[in] out_size Size of the buffer
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument,
 *         ESP_ERR_TIMEOUT if the module did not answer, or
 *         ESP_ERR_INVALID_RESPONSE if the reply could not be read
 */
esp_err_t adsb_receiver_get_serial_number(adsb_receiver_t *dev, char *out, size_t out_size);

/**
 * @brief Read the firmware version, AT+FIRMWARE_VERSION?
 *
 * @param[in,out] dev Handle
 * @param[out] out Buffer for the firmware version
 * @param[in] out_size Size of the buffer
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument,
 *         ESP_ERR_TIMEOUT if the module did not answer, or
 *         ESP_ERR_INVALID_RESPONSE if the reply could not be read
 */
esp_err_t adsb_receiver_get_firmware_version(adsb_receiver_t *dev, char *out, size_t out_size);

/**
 * @brief Read the device type, AT+DEVICE?
 *
 * @param[in,out] dev Handle
 * @param[out] out Buffer for the device type
 * @param[in] out_size Size of the buffer
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument,
 *         ESP_ERR_TIMEOUT if the module did not answer, or
 *         ESP_ERR_INVALID_RESPONSE if the reply could not be read
 */
esp_err_t adsb_receiver_get_device_type(adsb_receiver_t *dev, char *out, size_t out_size);

/**
 * @brief Check the connection, AT+TEST
 *
 * @param[in,out] dev Handle
 *
 * @return ESP_OK if the module answered AT+OK, ESP_ERR_TIMEOUT if it did not
 *         answer, ESP_FAIL if it answered with AT+ERROR, or
 *         ESP_ERR_INVALID_RESPONSE on any other reply
 */
esp_err_t adsb_receiver_test(adsb_receiver_t *dev);

// *****************************************************************************
// Section: Typed settings
//
// The settings the SC1 supports. Like the generic ones, they only work in the
// CONFIGURATION state. Each returns ESP_OK on success, ESP_ERR_INVALID_ARG on a
// value out of range, or an error from adsb_receiver_set_setting().

/**
 * @brief Set BAUDRATE, the baud rate of the RUN state
 *
 * Takes effect on the next entry into RUN, so pass the matching baud rate to
 * adsb_receiver_exit_configuration().
 *
 * @param[in,out] dev Handle
 * @param[in] baudrate New baud rate
 *
 * @return See the section description
 */
esp_err_t adsb_receiver_set_baudrate(adsb_receiver_t *dev, adsb_receiver_baudrate_t baudrate);

/**
 * @brief Set SYSTEM_LOG, whether the module sends system log lines
 *
 * @param[in,out] dev Handle
 * @param[in] enable true to send log lines
 *
 * @return See the section description
 */
esp_err_t adsb_receiver_set_system_log(adsb_receiver_t *dev, bool enable);

/**
 * @brief Set SYSTEM_STATISTICS, the protocol of the "#S:" system statistics
 *
 * @param[in,out] dev Handle
 * @param[in] protocol Protocol, or ::ADSB_RECEIVER_STATS_PROTOCOL_NONE to turn them off
 *
 * @return See the section description
 */
esp_err_t adsb_receiver_set_system_statistics_protocol(adsb_receiver_t *dev,
                                                       adsb_receiver_stats_protocol_t protocol);

/**
 * @brief Set ADSB_RX_PROTOCOL_DECODED, the protocol aircraft are reported in
 *
 * @param[in,out] dev Handle
 * @param[in] protocol CSV, MAVLink, or none
 *
 * @return See the section description
 */
esp_err_t adsb_receiver_set_adsb_rx_protocol_decoded(adsb_receiver_t *dev,
                                                     adsb_receiver_decoded_protocol_t protocol);

/**
 * @brief Set ADSB_RX_PROTOCOL_INC, how often aircraft are reported
 *
 * @param[in,out] dev Handle
 * @param[in] mode Reporting mode
 *
 * @return See the section description
 */
esp_err_t adsb_receiver_set_adsb_rx_protocol_inc(adsb_receiver_t *dev, adsb_receiver_inc_mode_t mode);

/**
 * @brief Set ADSB_STATISTICS, the protocol of the "#AS:" ADS-B statistics
 *
 * @param[in,out] dev Handle
 * @param[in] protocol Protocol, or ::ADSB_RECEIVER_STATS_PROTOCOL_NONE to turn them off
 *
 * @return See the section description
 */
esp_err_t adsb_receiver_set_adsb_statistics_protocol(adsb_receiver_t *dev, adsb_receiver_stats_protocol_t protocol);

// *****************************************************************************
// Section: Reading data in the RUN state

/**
 * @brief Read whatever the module has sent and decode it
 *
 * Drains the UART receive buffer without blocking, assembles CSV lines and
 * MAVLink frames, telling them apart byte by byte, and calls the registered
 * callbacks from the calling task.
 *
 * This has to be called often enough that the receive buffer never overflows.
 * Use adsb_receiver_poll_timeout() instead of a plain delay in a loop that has
 * nothing else to do.
 *
 * @param[in,out] dev Handle
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL handle, or ESP_FAIL
 *         if the UART could not be read
 */
esp_err_t adsb_receiver_poll(adsb_receiver_t *dev);

/**
 * @brief Keep reading and decoding for a while
 *
 * Same as adsb_receiver_poll(), except that it keeps going for `duration_ms`,
 * blocking on the UART while it waits, so other tasks get to run.
 *
 * @param[in,out] dev Handle
 * @param[in] duration_ms How long to keep reading, in ms
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL handle, or ESP_FAIL
 *         if the UART could not be read
 */
esp_err_t adsb_receiver_poll_timeout(adsb_receiver_t *dev, uint32_t duration_ms);

/**
 * @brief Register a callback for decoded "#A:" aircraft messages
 *
 * @param[in,out] dev Handle
 * @param[in] cb Callback, or NULL to remove it
 * @param[in] arg Passed to the callback untouched
 */
void adsb_receiver_set_aircraft_callback(adsb_receiver_t *dev, adsb_receiver_aircraft_cb_t cb, void *arg);

/**
 * @brief Register a callback for decoded "#AS:" ADS-B statistics messages
 *
 * @param[in,out] dev Handle
 * @param[in] cb Callback, or NULL to remove it
 * @param[in] arg Passed to the callback untouched
 */
void adsb_receiver_set_adsb_stats_callback(adsb_receiver_t *dev, adsb_receiver_adsb_stats_cb_t cb, void *arg);

/**
 * @brief Register a callback for decoded "#S:" system statistics messages
 *
 * @param[in,out] dev Handle
 * @param[in] cb Callback, or NULL to remove it
 * @param[in] arg Passed to the callback untouched
 */
void adsb_receiver_set_system_stats_callback(adsb_receiver_t *dev, adsb_receiver_system_stats_cb_t cb, void *arg);

/**
 * @brief Register a callback for lines that are not a known message
 *
 * @param[in,out] dev Handle
 * @param[in] cb Callback, or NULL to remove it
 * @param[in] arg Passed to the callback untouched
 */
void adsb_receiver_set_unknown_line_callback(adsb_receiver_t *dev, adsb_receiver_line_cb_t cb, void *arg);

/**
 * @brief Register a callback for every line received
 *
 * Fires for every complete line before it is decoded, including the ones later
 * recognized as "#A:", "#AS:" or "#S:", and for AT replies. Handy for seeing
 * exactly what is on the wire.
 *
 * @param[in,out] dev Handle
 * @param[in] cb Callback, or NULL to remove it
 * @param[in] arg Passed to the callback untouched
 */
void adsb_receiver_set_line_callback(adsb_receiver_t *dev, adsb_receiver_line_cb_t cb, void *arg);

/**
 * @brief Register a callback for every MAVLink frame received
 *
 * Fires for every frame whatever its message id. `crc_valid` only means
 * something for HEARTBEAT and ADSB_VEHICLE, see ::adsb_receiver_mavlink_frame_t.
 *
 * @param[in,out] dev Handle
 * @param[in] cb Callback, or NULL to remove it
 * @param[in] arg Passed to the callback untouched
 */
void adsb_receiver_set_mavlink_frame_callback(adsb_receiver_t *dev, adsb_receiver_mavlink_frame_cb_t cb, void *arg);

/**
 * @brief Register a callback for decoded MAVLink ADSB_VEHICLE messages
 *
 * Only fires for frames that pass checksum verification.
 *
 * @param[in,out] dev Handle
 * @param[in] cb Callback, or NULL to remove it
 * @param[in] arg Passed to the callback untouched
 */
void adsb_receiver_set_adsb_vehicle_callback(adsb_receiver_t *dev, adsb_receiver_adsb_vehicle_cb_t cb, void *arg);

// *****************************************************************************
// Section: Stand-alone decoders
//
// These need no handle, so they also work on logged data.

/**
 * @brief Work out the CRC16 the module appends to its CSV lines
 *
 * @param[in] data Bytes to check
 * @param[in] length How many bytes there are
 *
 * @return CRC16, byte order as the module sends it
 */
uint16_t adsb_receiver_crc16(const uint8_t *data, size_t length);

/**
 * @brief Read and check the CRC field at the end of a CSV line
 *
 * @param[in] line CSV line starting with '#'
 * @param[out] computed CRC worked out over everything before the last comma
 * @param[out] received CRC sent in the last field
 *
 * @return true if the line has a CRC field; compare the two to check it
 */
bool adsb_receiver_verify_csv_crc(const char *line, uint16_t *computed, uint16_t *received);

/**
 * @brief Decode a "#A:" aircraft line
 *
 * @param[in] line Line as received
 * @param[out] out Decoded aircraft
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument, or
 *         ESP_ERR_INVALID_RESPONSE if the line is not a whole "#A:" message
 */
esp_err_t adsb_receiver_parse_aircraft_line(const char *line, adsb_receiver_aircraft_t *out);

/**
 * @brief Decode a "#AS:" ADS-B statistics line
 *
 * @param[in] line Line as received
 * @param[out] out Decoded statistics
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument, or
 *         ESP_ERR_INVALID_RESPONSE if the line is not a whole "#AS:" message
 */
esp_err_t adsb_receiver_parse_adsb_stats_line(const char *line, adsb_receiver_adsb_stats_t *out);

/**
 * @brief Decode a "#S:" system statistics line
 *
 * @param[in] line Line as received
 * @param[out] out Decoded statistics
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument, or
 *         ESP_ERR_INVALID_RESPONSE if the line is not a whole "#S:" message
 */
esp_err_t adsb_receiver_parse_system_stats_line(const char *line, adsb_receiver_system_stats_t *out);

/**
 * @brief Decode a received MAVLink frame as ADSB_VEHICLE
 *
 * @param[in] frame Received frame
 * @param[out] out Decoded vehicle
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument, or
 *         ESP_ERR_INVALID_RESPONSE if the frame is not an ADSB_VEHICLE message
 */
esp_err_t adsb_receiver_parse_mavlink_adsb_vehicle(const adsb_receiver_mavlink_frame_t *frame,
                                                   adsb_receiver_adsb_vehicle_t *out);

#ifdef __cplusplus
}
#endif
