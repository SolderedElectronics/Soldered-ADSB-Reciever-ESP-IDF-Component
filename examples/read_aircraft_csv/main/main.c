/**
 * @file main.c
 * @brief Example for reading decoded aircraft reports off the module
 *
 * Reads the "#A:" aircraft reports the module sends in its default RUN state
 * and CSV protocol, and prints every one of them with whichever fields the
 * module had data for. Nothing needs configuring: the module starts sending
 * aircraft as soon as it is powered and hears any.
 *
 * Aircraft only show up within range of the antenna, so connect one and give
 * it a view of the sky.
 *
 * Using a different board? See the custom_board_wiring example.
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

static const char *TAG = "READ_AIRCRAFT_CSV";

/**
 * @brief Print one decoded aircraft report
 *
 * Invoked by the driver from adsb_receiver_poll_timeout() below, once per
 * decoded aircraft update.
 *
 * @param[in] aircraft Decoded aircraft
 * @param[in] arg Unused
 */
static void on_aircraft(const adsb_receiver_aircraft_t *aircraft, void *arg)
{
    printf("ICAO %s", aircraft->icao);

    if (aircraft->callsign[0] != '\0') {
        printf("  call=%s", aircraft->callsign);
    }
    if (aircraft->has_position) {
        printf("  lat=%.5f lon=%.5f", aircraft->lat, aircraft->lon);
    }
    if (aircraft->has_alt_baro) {
        printf("  alt_baro=%ldft", (long)aircraft->alt_baro);
    }
    if (aircraft->has_vel_h) {
        printf("  vel=%.2fkt", aircraft->vel_h);
    }
    if (aircraft->squawk[0] != '\0') {
        printf("  squawk=%s", aircraft->squawk);
    }
    printf("  on_ground=%s", adsb_receiver_aircraft_on_ground(aircraft) ? "yes" : "no");
    printf("  crc_valid=%s\n", aircraft->crc_valid ? "yes" : "no");
}

void app_main(void)
{
    static adsb_receiver_t adsb;
    adsb_receiver_config_t config = ADSB_RECEIVER_DEFAULT_CONFIG();

    // Open the UART link to the module at its factory default RUN baud rate
    ESP_ERROR_CHECK(adsb_receiver_init(&adsb, &config));

    // Register the function to call whenever a decoded aircraft report arrives
    adsb_receiver_set_aircraft_callback(&adsb, on_aircraft, NULL);

    ESP_LOGI(TAG, "Waiting for ADS-B traffic...");

    while (1) {
        // Reads incoming bytes and fires the registered callbacks
        ESP_ERROR_CHECK(adsb_receiver_poll_timeout(&adsb, 100));
    }
}
