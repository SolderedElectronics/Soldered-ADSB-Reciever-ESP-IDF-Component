/**
 * @file main.c
 * @brief Example for reading the statistics messages of the module
 *
 * Turns on and reads the two CSV statistics messages: system health ("#S:")
 * and ADS-B receiver health ("#AS:"). Handy for keeping an eye on CPU load,
 * uptime, received frame rate and PPS calibration in the field.
 *
 * Product used is www.solde.red/333388
 *
 * Connections:
 *   The module is wired to the onboard ESP32-S3, nothing to connect.
 *
 * @author Soldered Electronics
 */

#include <inttypes.h>
#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "soldered_adsb_receiver.h"

static const char *TAG = "STATISTICS";

/**
 * @brief Print the system statistics, sent once a second
 *
 * @param[in] stats CPU load and uptime of the module
 * @param[in] arg Unused
 */
static void on_system_stats(const adsb_receiver_system_stats_t *stats, void *arg)
{
    printf("[SYSTEM] CPU load=%.2f%%  uptime=%" PRIu32 "s  crc_valid=%s\n", stats->cpu_load, stats->uptime,
           stats->crc_valid ? "yes" : "no");
}

/**
 * @brief Print the ADS-B receiver statistics, sent once a second
 *
 * @param[in] stats Received frame rates and PPS calibration
 * @param[in] arg Unused
 */
static void on_adsb_stats(const adsb_receiver_adsb_stats_t *stats, void *arg)
{
    printf("[ADS-B]  Mode-S fps=%" PRIu32 "  Mode-A/C fps=%" PRIu32 "  PPS calib=%.2f  crc_valid=%s\n", stats->fpss,
           stats->fpsac, stats->calib, stats->crc_valid ? "yes" : "no");
}

void app_main(void)
{
    static adsb_receiver_t adsb;
    adsb_receiver_config_t config = ADSB_RECEIVER_DEFAULT_CONFIG();

    ESP_ERROR_CHECK(adsb_receiver_init(&adsb, &config));

    // Register the callbacks that receive the statistics messages
    adsb_receiver_set_system_stats_callback(&adsb, on_system_stats, NULL);
    adsb_receiver_set_adsb_stats_callback(&adsb, on_adsb_stats, NULL);

    // Both statistics protocols are off by default, so turn them on here
    esp_err_t err = adsb_receiver_enter_configuration(&adsb);
    if (err == ESP_OK) {
        adsb_receiver_set_system_statistics_protocol(&adsb, ADSB_RECEIVER_STATS_PROTOCOL_CSV);
        adsb_receiver_set_adsb_statistics_protocol(&adsb, ADSB_RECEIVER_STATS_PROTOCOL_CSV);
        adsb_receiver_exit_configuration(&adsb, ADSB_RECEIVER_DEFAULT_BAUD_RATE);
    } else {
        ESP_LOGE(TAG, "Failed to enter CONFIGURATION state: %s", esp_err_to_name(err));
    }

    while (1) {
        ESP_ERROR_CHECK(adsb_receiver_poll_timeout(&adsb, 100));
    }
}
