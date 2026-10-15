/**
 * @file main.c
 * @brief Example for reading the identity and settings of the module
 *
 * Enters the CONFIGURATION state and prints the serial number, the firmware
 * version, the device type, the full AT+HELP text and the current AT+SETTINGS?
 * dump, then returns the module to the RUN state.
 *
 * Every raw line the module sends is printed as well, which shows what is
 * really happening on the wire.
 *
 * Product used is www.solde.red/333388
 *
 * Connections:
 *   The module is wired to the onboard ESP32-S3, nothing to connect.
 *
 * @author Soldered Electronics
 */

#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "soldered_adsb_receiver.h"

static const char *TAG = "DEVICE_INFO";

// Room for the AT+HELP and AT+SETTINGS? replies, which run to a few kB
#define TEXT_BUF_SIZE 8192

/**
 * @brief Print every raw line the module sends
 *
 * @param[in] line Line as received
 * @param[in] arg Unused
 */
static void on_line(const char *line, void *arg)
{
    printf("[module] %s\n", line);
}

void app_main(void)
{
    static adsb_receiver_t adsb;
    static char text[TEXT_BUF_SIZE];
    adsb_receiver_config_t config = ADSB_RECEIVER_DEFAULT_CONFIG();

    ESP_ERROR_CHECK(adsb_receiver_init(&adsb, &config));

    adsb_receiver_set_line_callback(&adsb, on_line, NULL);

    // Settings can only be read or written in the CONFIGURATION state
    ESP_LOGI(TAG, "Entering CONFIGURATION state...");
    esp_err_t err = adsb_receiver_enter_configuration(&adsb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter CONFIGURATION state: %s", esp_err_to_name(err));
        return;
    }

    char value[64];

    // Read the serial number of the module
    if (adsb_receiver_get_serial_number(&adsb, value, sizeof(value)) == ESP_OK) {
        ESP_LOGI(TAG, "Serial number: %s", value);
    }

    // Read the firmware version currently installed
    if (adsb_receiver_get_firmware_version(&adsb, value, sizeof(value)) == ESP_OK) {
        ESP_LOGI(TAG, "Firmware version: %s", value);
    }

    // Read the device type the module reports
    if (adsb_receiver_get_device_type(&adsb, value, sizeof(value)) == ESP_OK) {
        ESP_LOGI(TAG, "Device type: %s", value);
    }

    // Simple connectivity check, the module should always answer AT+OK
    ESP_LOGI(TAG, "Connection test (AT+TEST): %s", adsb_receiver_test(&adsb) == ESP_OK ? "OK" : "FAILED");

    // Ask the module to list every setting and its current value
    printf("\n--- AT+SETTINGS? ---\n");
    if (adsb_receiver_get_all_settings(&adsb, text, sizeof(text)) != ESP_ERR_TIMEOUT) {
        printf("%s\n", text);
    }

    // Ask the module for its full built-in help text
    printf("\n--- AT+HELP ---\n");
    if (adsb_receiver_get_full_help(&adsb, text, sizeof(text)) != ESP_ERR_TIMEOUT) {
        printf("%s\n", text);
    }

    // Leave CONFIGURATION and resume normal aircraft tracking. The baud rate
    // must match the BAUDRATE setting printed above
    ESP_LOGI(TAG, "Returning to RUN state...");
    err = adsb_receiver_exit_configuration(&adsb, ADSB_RECEIVER_DEFAULT_BAUD_RATE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to return to RUN state: %s", esp_err_to_name(err));
    }
}
