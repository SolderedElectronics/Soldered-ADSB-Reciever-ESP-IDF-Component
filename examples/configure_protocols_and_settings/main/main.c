/**
 * @file main.c
 * @brief Example for changing every setting the module supports
 *
 * Writes every SYSTEM and ADS-B setting the SC1 supports: the decoded output
 * protocol (CSV or MAVLink), how often aircraft are reported, the statistics
 * protocols, the system log and the baud rate. Settings are kept by the module
 * across power cycles.
 *
 * Product used is www.solde.red/333388
 *
 * Connections:
 *   The module is wired to the onboard ESP32-S3, nothing to connect.
 *
 * @author Soldered Electronics
 */

#include "esp_err.h"
#include "esp_log.h"
#include "soldered_adsb_receiver.h"

static const char *TAG = "CONFIGURE";

/**
 * @brief Print whether a setting write succeeded
 *
 * @param[in] what Setting written
 * @param[in] err What the write returned
 */
static void print_result(const char *what, esp_err_t err)
{
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "%s -> AT+OK", what);
    } else {
        ESP_LOGE(TAG, "%s -> FAILED (%s)", what, esp_err_to_name(err));
    }
}

void app_main(void)
{
    static adsb_receiver_t adsb;
    adsb_receiver_config_t config = ADSB_RECEIVER_DEFAULT_CONFIG();

    ESP_ERROR_CHECK(adsb_receiver_init(&adsb, &config));

    // Settings can only be written in the CONFIGURATION state
    esp_err_t err;
    do{
    err = adsb_receiver_enter_configuration(&adsb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter CONFIGURATION state: %s", esp_err_to_name(err));
        //return;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    }while(err != ESP_OK);

    // --- ADS-B decoded output protocol ---
    print_result("ADSB_RX_PROTOCOL_DECODED=CSV",
                 adsb_receiver_set_adsb_rx_protocol_decoded(&adsb, ADSB_RECEIVER_DECODED_PROTOCOL_CSV));
    vTaskDelay(pdMS_TO_TICKS(500));
    // How often decoded aircraft are reported: as soon as their position updates
    print_result("ADSB_RX_PROTOCOL_INC=IMMEDIATE_ON_POSITION_UPDATE",
                 adsb_receiver_set_adsb_rx_protocol_inc(&adsb, ADSB_RECEIVER_INC_IMMEDIATE_ON_POSITION_UPDATE));
    vTaskDelay(pdMS_TO_TICKS(500));
    // ADS-B receiver statistics, frames per second and PPS calibration, see the statistics example
    print_result("ADSB_STATISTICS=CSV",
                 adsb_receiver_set_adsb_statistics_protocol(&adsb, ADSB_RECEIVER_STATS_PROTOCOL_CSV));
    vTaskDelay(pdMS_TO_TICKS(500));
    // --- SYSTEM settings ---
    print_result("SYSTEM_LOG=0", adsb_receiver_set_system_log(&adsb, false));
    print_result("SYSTEM_STATISTICS=CSV",
                 adsb_receiver_set_system_statistics_protocol(&adsb, ADSB_RECEIVER_STATS_PROTOCOL_CSV));
    vTaskDelay(pdMS_TO_TICKS(500));
    // BAUDRATE takes effect on the next entry into RUN, so the baud rate handed to
    // adsb_receiver_exit_configuration() below has to match it
    print_result("BAUDRATE=115200", adsb_receiver_set_baudrate(&adsb, ADSB_RECEIVER_BAUDRATE_115200));

    ESP_LOGI(TAG, "Returning to RUN state...");
    err = adsb_receiver_exit_configuration(&adsb, 115200);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to return to RUN state: %s", esp_err_to_name(err));
    }
}
