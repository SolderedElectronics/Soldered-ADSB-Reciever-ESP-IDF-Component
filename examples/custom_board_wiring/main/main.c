/**
 * @file main.c
 * @brief Example for using the component with custom wiring
 *
 * Shows how to drive a module that is not wired the way the Soldered ADS-B
 * 1090MHz Receiver board wires it (UART1, IO17/IO18, RESET on IO21). Start from
 * ADSB_RECEIVER_DEFAULT_CONFIG() and override only the fields that differ:
 *
 *   uart_port - which UART peripheral to use, avoid the console UART
 *   rx_pin    - ESP32 pin wired to the TX pin of the module
 *   tx_pin    - ESP32 pin wired to the RX pin of the module
 *   reset_pin - ESP32 pin wired to the RESET pin of the module, or -1 if not connected
 *
 * Here the module sits on UART2 on IO4/IO5, with no RESET pin wired at all.
 *
 * Product used is www.solde.red/333388
 *
 * Connections:
 *   TT-SC1b      ESP32
 *   TX           GPIO4
 *   RX           GPIO5
 *   GND          GND
 *
 * @author Soldered Electronics
 */

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "soldered_adsb_receiver.h"

static const char *TAG = "CUSTOM_BOARD_WIRING";

// Change these to match how your module is wired
#define ADSB_UART_PORT UART_NUM_2
#define PIN_NUM_RX     GPIO_NUM_4 // Goes to the TX pin of the module
#define PIN_NUM_TX     GPIO_NUM_5 // Goes to the RX pin of the module
#define PIN_NUM_RESET  -1         // RESET pin of the module, -1 if not connected

/**
 * @brief Print the address of every decoded aircraft
 *
 * @param[in] aircraft Decoded aircraft
 * @param[in] arg Unused
 */
static void on_aircraft(const adsb_receiver_aircraft_t *aircraft, void *arg)
{
    printf("ICAO %s\n", aircraft->icao);
}

void app_main(void)
{
    static adsb_receiver_t adsb;
    adsb_receiver_config_t config = ADSB_RECEIVER_DEFAULT_CONFIG();
    config.uart_port = ADSB_UART_PORT;
    config.rx_pin = PIN_NUM_RX;
    config.tx_pin = PIN_NUM_TX;
    config.reset_pin = PIN_NUM_RESET;

    // Open the UART link on the pins set above
    ESP_ERROR_CHECK(adsb_receiver_init(&adsb, &config));

    // Register the function to call whenever a decoded aircraft report arrives
    adsb_receiver_set_aircraft_callback(&adsb, on_aircraft, NULL);

    ESP_LOGI(TAG, "Waiting for ADS-B traffic...");

    while (1) {
        ESP_ERROR_CHECK(adsb_receiver_poll_timeout(&adsb, 100));
    }
}
