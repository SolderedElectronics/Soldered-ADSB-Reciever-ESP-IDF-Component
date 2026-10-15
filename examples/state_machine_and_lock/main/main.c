/**
 * @file main.c
 * @brief Example walking through the whole state machine of the module
 *
 * Goes hardware RESET -> BOOTLOADER -> RUN -> CONFIGURATION -> BOOTLOADER
 * (locked) -> RUN, using the RESET pin (IO21 on this board) and the lock to
 * hold the module in BOOTLOADER on purpose.
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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soldered_adsb_receiver.h"

static const char *TAG = "STATE_MACHINE";

// How long the RESET pin is held, and how long the module then stays in BOOTLOADER
#define RESET_PULSE_MS       20
#define BOOTLOADER_HOLD_MS   4000

// Time the module is given to switch state after a command
#define STATE_SWITCH_MS 500

void app_main(void)
{
    static adsb_receiver_t adsb;
    adsb_receiver_config_t config = ADSB_RECEIVER_DEFAULT_CONFIG();

    ESP_ERROR_CHECK(adsb_receiver_init(&adsb, &config));

    ESP_LOGI(TAG, "Hardware-resetting the module (RESET pin)...");
    esp_err_t err = adsb_receiver_hardware_reset(&adsb, RESET_PULSE_MS, true, true);
    if (err == ESP_OK) {
        // BOOTLOADER holds for about 3 s at the fixed 115200 baud, then the module
        // moves on to RUN unless it is locked. Wait it out before talking to it
        vTaskDelay(pdMS_TO_TICKS(BOOTLOADER_HOLD_MS));
    } else if (err == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "No RESET pin configured, skipping the hardware reset");
    } else {
        ESP_LOGE(TAG, "Hardware reset failed: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "RUN -> CONFIGURATION");
    err = adsb_receiver_enter_configuration(&adsb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter CONFIGURATION state: %s", esp_err_to_name(err));
        return;
    }

    // Ask the module to confirm which state it is actually in
    uint8_t state;
    if (adsb_receiver_get_config_state(&adsb, &state) == ESP_OK) {
        ESP_LOGI(TAG, "AT+CONFIG? = %u", state);
    }

    ESP_LOGI(TAG, "CONFIGURATION -> BOOTLOADER (this also sets AT+LOCK=1)");
    err = adsb_receiver_reboot_to_bootloader(&adsb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reboot to BOOTLOADER state: %s", esp_err_to_name(err));
        return;
    }

    // BOOTLOADER always runs at the fixed 115200 baud, same as CONFIGURATION
    vTaskDelay(pdMS_TO_TICKS(STATE_SWITCH_MS));

    // Confirm the module really is in BOOTLOADER now
    bool in_bootloader = false;
    if (adsb_receiver_is_bootloader(&adsb, &in_bootloader) == ESP_OK) {
        ESP_LOGI(TAG, "AT+BOOT? = %s", in_bootloader ? "1 (BOOTLOADER)" : "0 (CONFIGURATION)");
    }

    // The lock should be set, since rebooting to the bootloader sets it
    bool locked = false;
    if (adsb_receiver_get_lock(&adsb, &locked) == ESP_OK) {
        ESP_LOGI(TAG, "AT+LOCK? = %s", locked ? "1 (locked)" : "0 (unlocked)");
    }

    ESP_LOGI(TAG, "Releasing the lock, the module falls through to the RUN state");
    adsb_receiver_set_lock(&adsb, false);

    // The module leaves BOOTLOADER and enters RUN at its BAUDRATE setting, 115200 by default
    vTaskDelay(pdMS_TO_TICKS(STATE_SWITCH_MS));
    ESP_LOGI(TAG, "Done. The module should now be broadcasting in the RUN state again");

    while (1) {
        ESP_ERROR_CHECK(adsb_receiver_poll_timeout(&adsb, 100));
    }
}
