/**
 * @file main.c
 * @brief Example for reading aircraft as MAVLink ADSB_VEHICLE messages
 *
 * Switches the module to MAVLink output and decodes the ADSB_VEHICLE message it
 * sends for every tracked aircraft. Also shows the generic frame callback,
 * which fires for any MAVLink message the module sends and prints its raw
 * payload bytes.
 *
 * The output protocol is kept by the module across power cycles; run the
 * configure_protocols_and_settings example to switch back to CSV.
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

static const char *TAG = "MAVLINK_ADSB_VEHICLE";

/**
 * @brief Print one decoded ADSB_VEHICLE message, one per tracked aircraft
 *
 * @param[in] vehicle Decoded vehicle
 * @param[in] arg Unused
 */
static void on_adsb_vehicle(const adsb_receiver_adsb_vehicle_t *vehicle, void *arg)
{
    printf("ICAO %06" PRIX32, vehicle->icao_address);

    if (adsb_receiver_vehicle_has_flag(vehicle, ADSB_RECEIVER_VEHICLE_FLAG_VALID_CALLSIGN)) {
        printf("  call=%s", vehicle->callsign);
    }
    if (adsb_receiver_vehicle_has_flag(vehicle, ADSB_RECEIVER_VEHICLE_FLAG_VALID_COORDS)) {
        printf("  lat=%.5f lon=%.5f", adsb_receiver_vehicle_lat_degrees(vehicle),
               adsb_receiver_vehicle_lon_degrees(vehicle));
    }
    if (adsb_receiver_vehicle_has_flag(vehicle, ADSB_RECEIVER_VEHICLE_FLAG_VALID_ALTITUDE)) {
        printf("  alt=%.2fm", adsb_receiver_vehicle_altitude_meters(vehicle));
    }
    if (adsb_receiver_vehicle_has_flag(vehicle, ADSB_RECEIVER_VEHICLE_FLAG_VALID_VELOCITY)) {
        printf("  vel=%.2fm/s", vehicle->hor_velocity / 100.0);
    }
    if (adsb_receiver_vehicle_has_flag(vehicle, ADSB_RECEIVER_VEHICLE_FLAG_VALID_SQUAWK)) {
        printf("  squawk=%u", vehicle->squawk);
    }
    printf("  tslc=%us\n", vehicle->tslc);
}

/**
 * @brief Print every MAVLink frame received, whatever its message id
 *
 * @param[in] frame Raw frame
 * @param[in] arg Unused
 */
static void on_frame(const adsb_receiver_mavlink_frame_t *frame, void *arg)
{
    printf("[mavlink] msgid=%" PRIu32 " sysid=%u crc_valid=%s payload=", frame->msgid, frame->sysid,
           frame->crc_valid ? "yes" : "no");
    for (uint8_t i = 0; i < frame->payload_len; i++) {
        printf("%02X", frame->payload[i]);
    }
    printf("\n");
}

void app_main(void)
{
    static adsb_receiver_t adsb;
    adsb_receiver_config_t config = ADSB_RECEIVER_DEFAULT_CONFIG();

    ESP_ERROR_CHECK(adsb_receiver_init(&adsb, &config));

    // Register the callbacks that receive the decoded and the raw MAVLink data
    adsb_receiver_set_adsb_vehicle_callback(&adsb, on_adsb_vehicle, NULL);
    adsb_receiver_set_mavlink_frame_callback(&adsb, on_frame, NULL);

    // The module sends CSV by default, so switch it to MAVLink here
    esp_err_t err = adsb_receiver_enter_configuration(&adsb);
    if (err == ESP_OK) {
        adsb_receiver_set_adsb_rx_protocol_decoded(&adsb, ADSB_RECEIVER_DECODED_PROTOCOL_MAVLINK);
        adsb_receiver_exit_configuration(&adsb, ADSB_RECEIVER_DEFAULT_BAUD_RATE);
    } else {
        ESP_LOGE(TAG, "Failed to enter CONFIGURATION state: %s", esp_err_to_name(err));
    }

    while (1) {
        ESP_ERROR_CHECK(adsb_receiver_poll_timeout(&adsb, 100));
    }
}
