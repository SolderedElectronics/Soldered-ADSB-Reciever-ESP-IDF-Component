/**
 * @file soldered_adsb_receiver.c
 * @brief Implementation for the soldered-adsb-receiver component
 *
 * The AT command set and the RUN / CONFIGURATION / BOOTLOADER state machine
 * follow the Aerobits OEM SC1/SC1-EXT datasheet.
 *
 * @author Soldered Electronics
 */

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soldered_adsb_receiver.h"

static const char *TAG = "ADSB_RECEIVER";

/* How much is pulled off the UART per read while draining the receive buffer */
#define READ_CHUNK_SIZE 64

/* How long a write is given before it is treated as a failure */
#define WRITE_TIMEOUT_MS 100

/* Room for the reply to a command whose reply is only looked at, not handed back */
#define RESPONSE_BUF_SIZE 256

/* Timeouts the module is given to answer, in ms */
#define AT_TIMEOUT_MS           500
#define STATE_CHANGE_TIMEOUT_MS 1500
#define REBOOT_TIMEOUT_MS       2000

/* Multi-line replies have no terminal line, so they are taken as done once the
 * module has been quiet for this long, in ms */
#define MULTILINE_QUIET_MS          150
#define MULTILINE_TIMEOUT_MS        1500
#define DESCRIPTION_TIMEOUT_MS      800

/* Time the module is given to finish switching state and baud rate, in ms */
#define STATE_SETTLE_MS 100

// *****************************************************************************
// Section: Small helpers

/**
 * @brief Turn a duration into ticks, rounding up so a short wait never becomes no wait
 *
 * @param[in] ms Duration in ms
 *
 * @return Duration in ticks, at least one
 */
static TickType_t ms_to_ticks_min_one(uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);
    return ticks == 0 ? 1 : ticks;
}

/**
 * @brief Check whether a string starts with a prefix
 *
 * @param[in] s String
 * @param[in] prefix Prefix
 *
 * @return true if `s` starts with `prefix`
 */
static bool starts_with(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

/**
 * @brief Take the whitespace off both ends of a string, in place
 *
 * @param[in,out] s String
 *
 * @return Start of the trimmed string, inside `s`
 */
static char *trim_in_place(char *s)
{
    while (*s != '\0' && isspace((unsigned char) * s)) {
        s++;
    }

    size_t length = strlen(s);
    while (length > 0 && isspace((unsigned char)s[length - 1])) {
        s[--length] = '\0';
    }

    return s;
}

/**
 * @brief Add a line to a reply buffer, separated from what is already there by '\n'
 *
 * @param[in,out] out Reply buffer, NUL terminated
 * @param[in] out_size Size of the buffer
 * @param[in] line Line to add
 *
 * @return false if the line did not fit and was cut short
 */
static bool append_line(char *out, size_t out_size, const char *line)
{
    if (out == NULL || out_size == 0) {
        return true;
    }

    size_t used = strlen(out);
    size_t needed = (used > 0 ? 1 : 0) + strlen(line);
    int written = snprintf(out + used, out_size - used, "%s%s", used > 0 ? "\n" : "", line);

    return written >= 0 && (size_t)written == needed && used + needed < out_size;
}

/**
 * @brief Find the value of `AT+<name>=<value>` in a reply
 *
 * @param[in] response Reply lines, separated by '\n'
 * @param[in] name Setting name
 * @param[out] value Buffer for the value
 * @param[in] value_size Size of the buffer
 *
 * @return true if the reply holds the setting
 */
static bool extract_assignment(const char *response, const char *name, char *value, size_t value_size)
{
    char prefix[ADSB_RECEIVER_MAX_COMMAND_LEN + 8];
    int prefix_len = snprintf(prefix, sizeof(prefix), "AT+%s=", name);
    if (prefix_len < 0 || (size_t)prefix_len >= sizeof(prefix)) {
        return false;
    }

    const char *line = response;
    while (line != NULL && *line != '\0') {
        const char *nl = strchr(line, '\n');
        size_t line_len = (nl != NULL) ? (size_t)(nl - line) : strlen(line);

        if (line_len >= (size_t)prefix_len && strncmp(line, prefix, (size_t)prefix_len) == 0) {
            size_t length = line_len - (size_t)prefix_len;
            if (length >= value_size) {
                length = value_size - 1;
            }
            memcpy(value, line + prefix_len, length);
            value[length] = '\0';
            return true;
        }

        line = (nl != NULL) ? nl + 1 : NULL;
    }

    return false;
}

/**
 * @brief Map the reply to a command that should be answered with AT+OK onto an error code
 *
 * @param[in] response Reply
 *
 * @return ESP_OK on AT+OK, ESP_FAIL on AT+ERROR, ESP_ERR_INVALID_RESPONSE otherwise
 */
static esp_err_t check_ok(const char *response)
{
    if (adsb_receiver_is_ok(response)) {
        return ESP_OK;
    }

    if (adsb_receiver_is_error(response, NULL, 0)) {
        return ESP_FAIL;
    }

    return ESP_ERR_INVALID_RESPONSE;
}

/**
 * @brief Check whether a line ends the wait for an AT reply
 *
 * AT+RUN_END and AT+CONFIG_END are exit notices the module sends before the
 * real arrival marker, RUN_END before CONFIG_START and CONFIG_END before
 * RUN_START, so they must not end the wait on their own.
 *
 * @param[in] line Received line
 *
 * @return true if the line is terminal
 */
static bool is_terminal_line(const char *line)
{
    return starts_with(line, "AT+OK") || starts_with(line, "AT+ERROR") || starts_with(line, "AT+CONFIG_START") ||
           starts_with(line, "AT+RUN_START") || starts_with(line, "AT+BOOTLOADER_START");
}

/**
 * @brief Throw away everything received so far, including half assembled lines and frames
 *
 * @param[in,out] dev Handle
 */
static void discard_input(adsb_receiver_t *dev)
{
    uart_flush_input(dev->uart_port);
    dev->line_len = 0;
    adsb_receiver_mavlink_reset(&dev->mavlink);
}

/**
 * @brief Add a received character to the line being assembled
 *
 * @param[in,out] dev Handle
 * @param[in] c Received character
 *
 * @return true when a non-empty line is complete in `dev->line_buf`; the caller
 *         sets `dev->line_len` back to zero once done with it
 */
static bool feed_line(adsb_receiver_t *dev, char c)
{
    if (c == '\r') {
        return false;
    }

    if (c == '\n') {
        if (dev->line_len == 0) {
            return false;
        }
        dev->line_buf[dev->line_len] = '\0';
        return true;
    }

    // A line this long is garbage, start over rather than keep growing it
    if (dev->line_len >= ADSB_RECEIVER_MAX_LINE_LEN) {
        dev->line_len = 0;
        return false;
    }

    dev->line_buf[dev->line_len++] = c;

    return false;
}

/**
 * @brief Decode a complete non-AT line and hand it to the matching callback
 *
 * @param[in,out] dev Handle
 * @param[in,out] raw_line Line, trimmed in place
 */
static void dispatch_line(adsb_receiver_t *dev, char *raw_line)
{
    char *line = trim_in_place(raw_line);
    if (*line == '\0') {
        return;
    }

    if (dev->line_cb != NULL) {
        dev->line_cb(line, dev->line_arg);
    }

    // "#AS:" is checked before "#A:", which it also starts with
    if (starts_with(line, "#AS:")) {
        adsb_receiver_adsb_stats_t stats;
        if (adsb_receiver_parse_adsb_stats_line(line, &stats) == ESP_OK && dev->adsb_stats_cb != NULL) {
            dev->adsb_stats_cb(&stats, dev->adsb_stats_arg);
        }
        return;
    }

    if (starts_with(line, "#A:")) {
        adsb_receiver_aircraft_t aircraft;
        if (adsb_receiver_parse_aircraft_line(line, &aircraft) == ESP_OK && dev->aircraft_cb != NULL) {
            dev->aircraft_cb(&aircraft, dev->aircraft_arg);
        }
        return;
    }

    if (starts_with(line, "#S:")) {
        adsb_receiver_system_stats_t stats;
        if (adsb_receiver_parse_system_stats_line(line, &stats) == ESP_OK && dev->system_stats_cb != NULL) {
            dev->system_stats_cb(&stats, dev->system_stats_arg);
        }
        return;
    }

    if (dev->unknown_line_cb != NULL) {
        dev->unknown_line_cb(line, dev->unknown_line_arg);
    }
}

/**
 * @brief Hand a complete MAVLink frame to the matching callbacks
 *
 * @param[in,out] dev Handle
 */
static void dispatch_mavlink_frame(adsb_receiver_t *dev)
{
    const adsb_receiver_mavlink_frame_t *frame = &dev->mavlink.frame;

    if (dev->mavlink_frame_cb != NULL) {
        dev->mavlink_frame_cb(frame, dev->mavlink_frame_arg);
    }

    if (frame->crc_valid && frame->msgid == ADSB_RECEIVER_MAVLINK_MSG_ID_ADSB_VEHICLE &&
            dev->adsb_vehicle_cb != NULL) {
        adsb_receiver_adsb_vehicle_t vehicle;
        if (adsb_receiver_parse_mavlink_adsb_vehicle(frame, &vehicle) == ESP_OK) {
            dev->adsb_vehicle_cb(&vehicle, dev->adsb_vehicle_arg);
        }
    }
}

/**
 * @brief Feed one received byte to the CSV line assembler or the MAVLink parser
 *
 * @param[in,out] dev Handle
 * @param[in] byte Received byte
 */
static void feed_byte(adsb_receiver_t *dev, uint8_t byte)
{
    if (adsb_receiver_mavlink_busy(&dev->mavlink)) {
        if (adsb_receiver_mavlink_feed(&dev->mavlink, byte)) {
            dispatch_mavlink_frame(dev);
        }
        return;
    }

    // A MAVLink start byte can only begin a frame between lines, never inside one
    if (dev->line_len == 0 && adsb_receiver_mavlink_start(&dev->mavlink, byte)) {
        return;
    }

    if (feed_line(dev, (char)byte)) {
        dispatch_line(dev, dev->line_buf);
        dev->line_len = 0;
    }
}

/**
 * @brief Read whatever is waiting on the UART and decode it
 *
 * @param[in,out] dev Handle
 * @param[in] wait_ticks How long the first read may block for
 *
 * @return ESP_OK on success, or ESP_FAIL if the UART could not be read
 */
static esp_err_t read_and_decode(adsb_receiver_t *dev, TickType_t wait_ticks)
{
    uint8_t buffer[READ_CHUNK_SIZE];

    while (1) {
        int length = uart_read_bytes(dev->uart_port, buffer, sizeof(buffer), wait_ticks);
        if (length < 0) {
            ESP_LOGE(TAG, "Could not read UART%d", (int)dev->uart_port);
            return ESP_FAIL;
        }

        for (int i = 0; i < length; i++) {
            feed_byte(dev, buffer[i]);
        }

        // A short read means the receive buffer has been drained
        if (length < (int)sizeof(buffer)) {
            break;
        }

        // Anything already waiting is taken without blocking again
        wait_ticks = 0;
    }

    return ESP_OK;
}

/**
 * @brief Send `AT+<body>\r\n` after throwing away whatever was received before
 *
 * @param[in,out] dev Handle
 * @param[in] body Command body
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a body that is too long, or
 *         ESP_ERR_TIMEOUT if the command could not be written
 */
static esp_err_t write_command(adsb_receiver_t *dev, const char *body)
{
    char command[ADSB_RECEIVER_MAX_COMMAND_LEN + 6];
    int length = snprintf(command, sizeof(command), "AT+%s\r\n", body);
    if (length < 0 || (size_t)length >= sizeof(command) || strlen(body) > ADSB_RECEIVER_MAX_COMMAND_LEN) {
        ESP_LOGE(TAG, "AT command body is longer than %d characters", ADSB_RECEIVER_MAX_COMMAND_LEN);
        return ESP_ERR_INVALID_ARG;
    }

    discard_input(dev);

    if (uart_write_bytes(dev->uart_port, command, (size_t)length) != length) {
        return ESP_ERR_TIMEOUT;
    }

    return uart_wait_tx_done(dev->uart_port, pdMS_TO_TICKS(WRITE_TIMEOUT_MS)) == ESP_OK ? ESP_OK : ESP_ERR_TIMEOUT;
}

/**
 * @brief Read one byte off the UART, waiting until a deadline at most
 *
 * @param[in] dev Handle
 * @param[in] deadline_us esp_timer time to give up at
 * @param[out] byte Byte read
 *
 * @return true if a byte was read
 */
static bool read_byte_until(adsb_receiver_t *dev, int64_t deadline_us, uint8_t *byte)
{
    int64_t left_us = deadline_us - esp_timer_get_time();
    if (left_us < 0) {
        left_us = 0;
    }

    return uart_read_bytes(dev->uart_port, byte, 1, ms_to_ticks_min_one((uint32_t)(left_us / 1000))) == 1;
}

/**
 * @brief Send a command whose reply is several lines long with no terminal line
 *
 * Collects every AT line until the module has been quiet for `quiet_ms` after
 * the first one, or `overall_timeout_ms` runs out.
 *
 * @param[in,out] dev Handle
 * @param[in] body Command body
 * @param[out] out Buffer for the reply lines
 * @param[in] out_size Size of the buffer
 * @param[in] overall_timeout_ms Longest time to wait for the whole reply
 * @param[in] quiet_ms Silence that marks the end of the reply
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument,
 *         ESP_ERR_TIMEOUT if nothing came back, or ESP_ERR_INVALID_SIZE if the
 *         reply was cut short
 */
static esp_err_t send_at_multiline(adsb_receiver_t *dev, const char *body, char *out, size_t out_size,
                                   uint32_t overall_timeout_ms, uint32_t quiet_ms)
{
    if (dev == NULL || body == NULL || out == NULL || out_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    out[0] = '\0';

    esp_err_t err = write_command(dev, body);
    if (err != ESP_OK) {
        return err;
    }

    bool any = false;
    bool fits = true;
    int64_t deadline_us = esp_timer_get_time() + (int64_t)overall_timeout_ms * 1000;
    int64_t last_line_us = esp_timer_get_time();

    while (esp_timer_get_time() < deadline_us) {
        // Once a reply has started, a quiet spell is what ends it
        int64_t wait_until_us = deadline_us;
        if (any && last_line_us + (int64_t)quiet_ms * 1000 < wait_until_us) {
            wait_until_us = last_line_us + (int64_t)quiet_ms * 1000;
        }

        uint8_t byte;
        if (!read_byte_until(dev, wait_until_us, &byte)) {
            if (any && esp_timer_get_time() - last_line_us > (int64_t)quiet_ms * 1000) {
                break;
            }
            continue;
        }

        if (!feed_line(dev, (char)byte)) {
            continue;
        }

        last_line_us = esp_timer_get_time();

        if (starts_with(dev->line_buf, "AT+")) {
            if (dev->line_cb != NULL) {
                dev->line_cb(dev->line_buf, dev->line_arg);
            }
            fits = append_line(out, out_size, dev->line_buf) && fits;
            any = true;
        } else {
            dispatch_line(dev, dev->line_buf);
        }
        dev->line_len = 0;
    }

    if (!any) {
        return ESP_ERR_TIMEOUT;
    }

    return fits ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

/**
 * @brief Read a setting into a buffer and convert it to a number
 *
 * @param[in,out] dev Handle
 * @param[in] name Setting name
 * @param[out] value Value read
 *
 * @return ESP_OK on success, or an error from adsb_receiver_get_setting()
 */
static esp_err_t get_setting_int(adsb_receiver_t *dev, const char *name, long *value)
{
    char text[32];
    esp_err_t err = adsb_receiver_get_setting(dev, name, text, sizeof(text));
    if (err != ESP_OK) {
        return err;
    }

    *value = strtol(text, NULL, 10);

    return ESP_OK;
}

/**
 * @brief Switch the local UART to a new baud rate once everything queued has gone out
 *
 * @param[in] dev Handle
 * @param[in] baud_rate New baud rate
 *
 * @return ESP_OK on success, or the error returned by the UART driver
 */
static esp_err_t switch_baud_rate(adsb_receiver_t *dev, uint32_t baud_rate)
{
    uart_wait_tx_done(dev->uart_port, pdMS_TO_TICKS(WRITE_TIMEOUT_MS));

    return uart_set_baudrate(dev->uart_port, baud_rate);
}

/**
 * @brief Send a command that the module answers with AT+OK
 *
 * @param[in,out] dev Handle
 * @param[in] body Command body
 * @param[in] timeout_ms How long to wait for the reply
 *
 * @return ESP_OK on AT+OK, or an error from adsb_receiver_send_at() or check_ok()
 */
static esp_err_t send_expect_ok(adsb_receiver_t *dev, const char *body, uint32_t timeout_ms)
{
    char response[RESPONSE_BUF_SIZE];
    esp_err_t err = adsb_receiver_send_at(dev, body, response, sizeof(response), timeout_ms, NULL);
    if (err != ESP_OK) {
        return err;
    }

    return check_ok(response);
}

/**
 * @brief Send a query and read the `AT+<name>=<value>` reply
 *
 * @param[in,out] dev Handle
 * @param[in] name Name queried, without the '?'
 * @param[out] value Buffer for the value
 * @param[in] value_size Size of the buffer
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument, or an
 *         error from adsb_receiver_send_at(), ESP_ERR_INVALID_RESPONSE if the
 *         reply did not hold the value
 */
static esp_err_t query(adsb_receiver_t *dev, const char *name, char *value, size_t value_size)
{
    if (name == NULL || value == NULL || value_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    char body[ADSB_RECEIVER_MAX_COMMAND_LEN + 2];
    int length = snprintf(body, sizeof(body), "%s?", name);
    if (length < 0 || (size_t)length >= sizeof(body)) {
        return ESP_ERR_INVALID_ARG;
    }

    char response[RESPONSE_BUF_SIZE];
    esp_err_t err = adsb_receiver_send_at(dev, body, response, sizeof(response), AT_TIMEOUT_MS, NULL);
    if (err != ESP_OK) {
        return err;
    }

    return extract_assignment(response, name, value, value_size) ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

/**
 * @brief Send AT+CONFIG=<value>, switch the local baud rate and check the state the module ended up in
 *
 * In practice the AT+RUN_END / AT+CONFIG_START notices are not always sent;
 * often only a generic AT+OK comes back, likely depending on the SYSTEM_LOG
 * setting. So success is not tied to a notice: the AT+OK is accepted, the baud
 * rate switched, and the change confirmed with AT+CONFIG? instead.
 *
 * @param[in,out] dev Handle
 * @param[in] enter true to enter CONFIGURATION, false to go back to RUN
 * @param[in] baud_rate Baud rate of the state being entered
 *
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if the module did not answer,
 *         ESP_FAIL if it answered with AT+ERROR, or ESP_ERR_INVALID_STATE if it
 *         did not end up in the expected state
 */
static esp_err_t change_config_state(adsb_receiver_t *dev, bool enter, uint32_t baud_rate)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    char response[RESPONSE_BUF_SIZE];
    esp_err_t err = adsb_receiver_send_at(dev, enter ? "CONFIG=1" : "CONFIG=0", response, sizeof(response),
                                          STATE_CHANGE_TIMEOUT_MS, NULL);
    if (err != ESP_OK) {
        return err;
    }

    if (adsb_receiver_is_error(response, NULL, 0)) {
        return ESP_FAIL;
    }

    err = switch_baud_rate(dev, baud_rate);
    if (err != ESP_OK) {
        return err;
    }

    // Give the module a moment to finish switching state and baud rate
    vTaskDelay(ms_to_ticks_min_one(STATE_SETTLE_MS));

    uint8_t state = 0;
    err = adsb_receiver_get_config_state(dev, &state);
    if (err != ESP_OK) {
        return err;
    }

    bool in_config = (state == ADSB_RECEIVER_CONFIG_STATE_CONFIG_115200 ||
                      state == ADSB_RECEIVER_CONFIG_STATE_CONFIG_SET_BAUD);
    bool reached = enter ? in_config : (state == ADSB_RECEIVER_CONFIG_STATE_RUN);
    if (!reached) {
        ESP_LOGW(TAG, "Module reports state %u after AT+CONFIG=%d", state, enter ? 1 : 0);
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

// *****************************************************************************
// Section: Lifetime

esp_err_t adsb_receiver_init(adsb_receiver_t *dev, const adsb_receiver_config_t *config)
{
    if (dev == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->rx_buffer_size < ADSB_RECEIVER_MIN_RX_BUFFER_SIZE) {
        ESP_LOGE(TAG, "Receive buffer of %d bytes is below the minimum of %d", config->rx_buffer_size,
                 ADSB_RECEIVER_MIN_RX_BUFFER_SIZE);
        return ESP_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));
    dev->uart_port = config->uart_port;
    dev->reset_pin = config->reset_pin;

    /* Left as a high impedance input rather than driven: holding a guessed idle
     * level would keep the module in reset for good if the guess were wrong.
     * Only adsb_receiver_hardware_reset() drives it, and only for the pulse. */
    if (dev->reset_pin >= 0) {
        gpio_config_t io_cfg = {
            .pin_bit_mask = 1ULL << dev->reset_pin,
                                 .mode = GPIO_MODE_INPUT,
                                 .pull_up_en = GPIO_PULLUP_DISABLE,
                                 .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                 .intr_type = GPIO_INTR_DISABLE,
        };
        esp_err_t err = gpio_config(&io_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Could not configure the RESET pin: %s", esp_err_to_name(err));
            return err;
        }
    }

    uart_config_t uart_cfg = {
        .baud_rate = (int)config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* No transmit ring buffer, so writes go straight out, and no event queue,
     * since the application polls with adsb_receiver_poll() */
    esp_err_t err = uart_driver_install(dev->uart_port, config->rx_buffer_size, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not install the UART driver: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_param_config(dev->uart_port, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not configure UART%d: %s", (int)dev->uart_port, esp_err_to_name(err));
        uart_driver_delete(dev->uart_port);
        return err;
    }

    err = uart_set_pin(dev->uart_port, config->tx_pin, config->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not route UART%d to the given pins: %s", (int)dev->uart_port, esp_err_to_name(err));
        uart_driver_delete(dev->uart_port);
        return err;
    }

    // Whatever arrived before the driver was ready is a partial line
    discard_input(dev);

    return ESP_OK;
}

esp_err_t adsb_receiver_deinit(adsb_receiver_t *dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return uart_driver_delete(dev->uart_port);
}

esp_err_t adsb_receiver_hardware_reset(adsb_receiver_t *dev, uint32_t pulse_ms, bool active_low,
                                       bool resync_to_115200)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (dev->reset_pin < 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Level first, so the pin never drives the wrong level when it turns into an output
    esp_err_t err = gpio_set_level(dev->reset_pin, active_low ? 0 : 1);
    if (err == ESP_OK) {
        err = gpio_set_direction(dev->reset_pin, GPIO_MODE_OUTPUT);
    }
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(ms_to_ticks_min_one(pulse_ms));

    // Released back to high impedance rather than held at a guessed idle level
    err = gpio_set_direction(dev->reset_pin, GPIO_MODE_INPUT);
    if (err != ESP_OK) {
        return err;
    }

    discard_input(dev);

    if (resync_to_115200) {
        return uart_set_baudrate(dev->uart_port, ADSB_RECEIVER_CONFIG_BAUD_RATE);
    }

    return ESP_OK;
}

// *****************************************************************************
// Section: AT command transport

esp_err_t adsb_receiver_send_at(adsb_receiver_t *dev, const char *body, char *response, size_t response_size,
                                uint32_t timeout_ms, const char *wait_for_prefix)
{
    if (dev == NULL || body == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (response != NULL && response_size > 0) {
        response[0] = '\0';
    }

    /* A query only ever gets a single "AT+NAME=value" reply with no AT+OK after
     * it. AT+SETTINGS? and AT+HELP are multi-line and go through
     * send_at_multiline() instead, so this only covers single-line queries. */
    size_t body_len = strlen(body);
    bool is_query = (wait_for_prefix == NULL) && body_len > 0 && body[body_len - 1] == '?';

    esp_err_t err = write_command(dev, body);
    if (err != ESP_OK) {
        return err;
    }

    int64_t deadline_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    /* Read a byte at a time, so whatever follows the reply stays in the
     * receive buffer for adsb_receiver_poll() */
    while (esp_timer_get_time() < deadline_us) {
        uint8_t byte;
        if (!read_byte_until(dev, deadline_us, &byte) || !feed_line(dev, (char)byte)) {
            continue;
        }

        if (!starts_with(dev->line_buf, "AT+")) {
            // Data still in flight, decoded as usual; dispatch_line() fires the line callback itself
            dispatch_line(dev, dev->line_buf);
            dev->line_len = 0;
            continue;
        }

        // AT replies never reach dispatch_line(), so the line callback is fired here
        if (dev->line_cb != NULL) {
            dev->line_cb(dev->line_buf, dev->line_arg);
        }
        append_line(response, response_size, dev->line_buf);

        bool done;
        if (wait_for_prefix != NULL) {
            // A generic AT+OK or an exit notice may come before the line waited for
            done = starts_with(dev->line_buf, wait_for_prefix) || starts_with(dev->line_buf, "AT+ERROR");
        } else {
            done = is_terminal_line(dev->line_buf) || is_query;
        }
        dev->line_len = 0;

        if (done) {
            return ESP_OK;
        }
    }

    return ESP_ERR_TIMEOUT;
}

bool adsb_receiver_is_ok(const char *response)
{
    return response != NULL && strstr(response, "AT+OK") != NULL;
}

bool adsb_receiver_is_error(const char *response, char *description, size_t description_size)
{
    if (response == NULL) {
        return false;
    }

    const char *error = strstr(response, "AT+ERROR");
    if (error == NULL) {
        return false;
    }

    if (description != NULL && description_size > 0) {
        description[0] = '\0';

        const char *open = strchr(error, '(');
        const char *close = strchr(error, ')');
        if (open != NULL && close != NULL && close > open) {
            size_t length = (size_t)(close - open - 1);
            if (length >= description_size) {
                length = description_size - 1;
            }
            memcpy(description, open + 1, length);
            description[length] = '\0';
        }
    }

    return true;
}

// *****************************************************************************
// Section: State machine

esp_err_t adsb_receiver_enter_configuration(adsb_receiver_t *dev)
{
    return change_config_state(dev, true, ADSB_RECEIVER_CONFIG_BAUD_RATE);
}

esp_err_t adsb_receiver_exit_configuration(adsb_receiver_t *dev, uint32_t run_baud)
{
    return change_config_state(dev, false, run_baud);
}

esp_err_t adsb_receiver_get_config_state(adsb_receiver_t *dev, uint8_t *state)
{
    if (dev == NULL || state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    long value;
    esp_err_t err = get_setting_int(dev, "CONFIG", &value);
    if (err != ESP_OK) {
        return err;
    }

    *state = (uint8_t)value;

    return ESP_OK;
}

esp_err_t adsb_receiver_reboot_to_bootloader(adsb_receiver_t *dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    char response[RESPONSE_BUF_SIZE];
    esp_err_t err = adsb_receiver_send_at(dev, "REBOOT_BOOTLOADER", response, sizeof(response), REBOOT_TIMEOUT_MS,
                                          NULL);
    if (err != ESP_OK) {
        return err;
    }

    if (adsb_receiver_is_ok(response) || strstr(response, "AT+BOOTLOADER_START") != NULL) {
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t adsb_receiver_reboot(adsb_receiver_t *dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return send_expect_ok(dev, "REBOOT", REBOOT_TIMEOUT_MS);
}

esp_err_t adsb_receiver_set_lock(adsb_receiver_t *dev, bool locked)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return send_expect_ok(dev, locked ? "LOCK=1" : "LOCK=0", AT_TIMEOUT_MS);
}

esp_err_t adsb_receiver_get_lock(adsb_receiver_t *dev, bool *locked)
{
    if (dev == NULL || locked == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    long value;
    esp_err_t err = get_setting_int(dev, "LOCK", &value);
    if (err != ESP_OK) {
        return err;
    }

    *locked = (value != 0);

    return ESP_OK;
}

esp_err_t adsb_receiver_is_bootloader(adsb_receiver_t *dev, bool *in_bootloader)
{
    if (dev == NULL || in_bootloader == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    long value;
    esp_err_t err = get_setting_int(dev, "BOOT", &value);
    if (err != ESP_OK) {
        return err;
    }

    *in_bootloader = (value != 0);

    return ESP_OK;
}

// *****************************************************************************
// Section: Generic settings and commands

esp_err_t adsb_receiver_set_setting(adsb_receiver_t *dev, const char *name, const char *value)
{
    if (dev == NULL || name == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    char body[ADSB_RECEIVER_MAX_COMMAND_LEN + 1];
    int length = snprintf(body, sizeof(body), "%s=%s", name, value);
    if (length < 0 || (size_t)length >= sizeof(body)) {
        return ESP_ERR_INVALID_ARG;
    }

    return send_expect_ok(dev, body, AT_TIMEOUT_MS);
}

esp_err_t adsb_receiver_get_setting(adsb_receiver_t *dev, const char *name, char *value, size_t value_size)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return query(dev, name, value, value_size);
}

esp_err_t adsb_receiver_get_setting_description(adsb_receiver_t *dev, const char *name, char *out, size_t out_size)
{
    if (dev == NULL || name == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    char body[ADSB_RECEIVER_MAX_COMMAND_LEN + 1];
    int length = snprintf(body, sizeof(body), "%s=?", name);
    if (length < 0 || (size_t)length >= sizeof(body)) {
        return ESP_ERR_INVALID_ARG;
    }

    return send_at_multiline(dev, body, out, out_size, DESCRIPTION_TIMEOUT_MS, MULTILINE_QUIET_MS);
}

esp_err_t adsb_receiver_get_all_settings(adsb_receiver_t *dev, char *out, size_t out_size)
{
    return send_at_multiline(dev, "SETTINGS?", out, out_size, MULTILINE_TIMEOUT_MS, MULTILINE_QUIET_MS);
}

esp_err_t adsb_receiver_get_full_help(adsb_receiver_t *dev, char *out, size_t out_size)
{
    return send_at_multiline(dev, "HELP", out, out_size, MULTILINE_TIMEOUT_MS, MULTILINE_QUIET_MS);
}

esp_err_t adsb_receiver_reset_settings_to_default(adsb_receiver_t *dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return send_expect_ok(dev, "SETTINGS_DEFAULT", AT_TIMEOUT_MS);
}

esp_err_t adsb_receiver_get_serial_number(adsb_receiver_t *dev, char *out, size_t out_size)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return query(dev, "SERIAL_NUMBER", out, out_size);
}

esp_err_t adsb_receiver_get_firmware_version(adsb_receiver_t *dev, char *out, size_t out_size)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return query(dev, "FIRMWARE_VERSION", out, out_size);
}

esp_err_t adsb_receiver_get_device_type(adsb_receiver_t *dev, char *out, size_t out_size)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return query(dev, "DEVICE", out, out_size);
}

esp_err_t adsb_receiver_test(adsb_receiver_t *dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return send_expect_ok(dev, "TEST", AT_TIMEOUT_MS);
}

// *****************************************************************************
// Section: Typed settings

esp_err_t adsb_receiver_set_baudrate(adsb_receiver_t *dev, adsb_receiver_baudrate_t baudrate)
{
    if (baudrate > ADSB_RECEIVER_BAUDRATE_57600) {
        return ESP_ERR_INVALID_ARG;
    }

    char value[4];
    snprintf(value, sizeof(value), "%d", (int)baudrate);

    return adsb_receiver_set_setting(dev, "BAUDRATE", value);
}

esp_err_t adsb_receiver_set_system_log(adsb_receiver_t *dev, bool enable)
{
    return adsb_receiver_set_setting(dev, "SYSTEM_LOG", enable ? "1" : "0");
}

esp_err_t adsb_receiver_set_system_statistics_protocol(adsb_receiver_t *dev,
                                                       adsb_receiver_stats_protocol_t protocol)
{
    if (protocol > ADSB_RECEIVER_STATS_PROTOCOL_CSV) {
        return ESP_ERR_INVALID_ARG;
    }

    // SYSTEM_STATISTICS is an integer setting, unlike ADSB_STATISTICS
    return adsb_receiver_set_setting(dev, "SYSTEM_STATISTICS", protocol == ADSB_RECEIVER_STATS_PROTOCOL_CSV ? "1" : "0");
}

esp_err_t adsb_receiver_set_adsb_rx_protocol_decoded(adsb_receiver_t *dev,
                                                     adsb_receiver_decoded_protocol_t protocol)
{
    /* A string setting on the module, confirmed with AT+ADSB_RX_PROTOCOL_DECODED=?,
     * so it takes the protocol's name rather than an index */
    const char *name;
    switch (protocol) {
    case ADSB_RECEIVER_DECODED_PROTOCOL_NONE:
        name = "None";
        break;
    case ADSB_RECEIVER_DECODED_PROTOCOL_CSV:
        name = "CSV";
        break;
    case ADSB_RECEIVER_DECODED_PROTOCOL_MAVLINK:
        name = "Mavlink";
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    return adsb_receiver_set_setting(dev, "ADSB_RX_PROTOCOL_DECODED", name);
}

esp_err_t adsb_receiver_set_adsb_rx_protocol_inc(adsb_receiver_t *dev, adsb_receiver_inc_mode_t mode)
{
    if (mode > ADSB_RECEIVER_INC_IMMEDIATE_ON_POSITION_UPDATE) {
        return ESP_ERR_INVALID_ARG;
    }

    char value[4];
    snprintf(value, sizeof(value), "%d", (int)mode);

    return adsb_receiver_set_setting(dev, "ADSB_RX_PROTOCOL_INC", value);
}

esp_err_t adsb_receiver_set_adsb_statistics_protocol(adsb_receiver_t *dev, adsb_receiver_stats_protocol_t protocol)
{
    if (protocol > ADSB_RECEIVER_STATS_PROTOCOL_CSV) {
        return ESP_ERR_INVALID_ARG;
    }

    // A string setting like ADSB_RX_PROTOCOL_DECODED, so it takes the name
    return adsb_receiver_set_setting(dev, "ADSB_STATISTICS",
                                     protocol == ADSB_RECEIVER_STATS_PROTOCOL_CSV ? "CSV" : "None");
}

// *****************************************************************************
// Section: Reading data in the RUN state

esp_err_t adsb_receiver_poll(adsb_receiver_t *dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return read_and_decode(dev, 0);
}

esp_err_t adsb_receiver_poll_timeout(adsb_receiver_t *dev, uint32_t duration_ms)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t deadline_us = esp_timer_get_time() + (int64_t)duration_ms * 1000;

    do {
        int64_t left_us = deadline_us - esp_timer_get_time();
        if (left_us < 0) {
            left_us = 0;
        }

        esp_err_t err = read_and_decode(dev, ms_to_ticks_min_one((uint32_t)(left_us / 1000)));
        if (err != ESP_OK) {
            return err;
        }
    } while (esp_timer_get_time() < deadline_us);

    return ESP_OK;
}

void adsb_receiver_set_aircraft_callback(adsb_receiver_t *dev, adsb_receiver_aircraft_cb_t cb, void *arg)
{
    if (dev == NULL) {
        return;
    }

    dev->aircraft_cb = cb;
    dev->aircraft_arg = arg;
}

void adsb_receiver_set_adsb_stats_callback(adsb_receiver_t *dev, adsb_receiver_adsb_stats_cb_t cb, void *arg)
{
    if (dev == NULL) {
        return;
    }

    dev->adsb_stats_cb = cb;
    dev->adsb_stats_arg = arg;
}

void adsb_receiver_set_system_stats_callback(adsb_receiver_t *dev, adsb_receiver_system_stats_cb_t cb, void *arg)
{
    if (dev == NULL) {
        return;
    }

    dev->system_stats_cb = cb;
    dev->system_stats_arg = arg;
}

void adsb_receiver_set_unknown_line_callback(adsb_receiver_t *dev, adsb_receiver_line_cb_t cb, void *arg)
{
    if (dev == NULL) {
        return;
    }

    dev->unknown_line_cb = cb;
    dev->unknown_line_arg = arg;
}

void adsb_receiver_set_line_callback(adsb_receiver_t *dev, adsb_receiver_line_cb_t cb, void *arg)
{
    if (dev == NULL) {
        return;
    }

    dev->line_cb = cb;
    dev->line_arg = arg;
}

void adsb_receiver_set_mavlink_frame_callback(adsb_receiver_t *dev, adsb_receiver_mavlink_frame_cb_t cb, void *arg)
{
    if (dev == NULL) {
        return;
    }

    dev->mavlink_frame_cb = cb;
    dev->mavlink_frame_arg = arg;
}

void adsb_receiver_set_adsb_vehicle_callback(adsb_receiver_t *dev, adsb_receiver_adsb_vehicle_cb_t cb, void *arg)
{
    if (dev == NULL) {
        return;
    }

    dev->adsb_vehicle_cb = cb;
    dev->adsb_vehicle_arg = arg;
}
