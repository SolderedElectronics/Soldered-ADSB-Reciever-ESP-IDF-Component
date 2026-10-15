# Soldered ADS-B 1090MHz Receiver ESP32-S3 Component

| ![ADS-B 1090MHz Receiver ESP32-S3](https://cms.soldered.com/products/333388/media/333388_photo_b98aaf.webp) |
| :---------------------------------------------------------------------------------------------------------: |
|                    [ADS-B 1090MHz Receiver ESP32-S3](https://solde.red/333388)                              |

ESP-IDF driver for the Soldered ADS-B 1090MHz Receiver ESP32-S3, built around the Aerobits TT-SC1b OEM ADS-B receiver module. The module listens for the ADS-B squitters aircraft broadcast on 1090 MHz and reports each aircraft's position, altitude, speed, callsign and squawk over UART. It is configured over the same UART with AT commands, and streams live traffic as either CSV or MAVLink.

This component covers what the base SC1 module supports: CSV and MAVLink output. Mode-A/C/S decode, USB, multilateration timestamps and the JSON, GDL90, ASTERIX and RAW protocols are exclusive to the SC1-EXT variant and are not included.

- **State machine**: moves the module into and out of its CONFIGURATION state to change settings, plus BOOTLOADER control, locking and a hardware reset over the RESET pin.
- **AT command layer**: read and write every setting, list all settings, full help text, reset to defaults, read the serial number, firmware version and device type, connection test, reboot (normal or to the bootloader).
- **ADS-B settings**: CSV or MAVLink output, how often aircraft are reported, and the statistics protocols.
- **CSV decoding**: aircraft state vectors (`#A:`), ADS-B receiver statistics (`#AS:`) and system statistics (`#S:`), with CRC16 verification of every line.
- **MAVLink decoding**: MAVLink v1 and v2 frames with checksum verification, decoded into an ADSB_VEHICLE struct, the one message the module sends for aircraft. Any other message is still handed over raw through a generic frame callback.

### Repository Contents

- **/src** - source files (.c)
  - `soldered_adsb_receiver.c` - the driver: UART, AT commands, state machine, data pump
  - `adsb_receiver_csv.c` - the CSV decoders and CRC16
  - `adsb_receiver_mavlink.c` - the MAVLink frame parser and ADSB_VEHICLE decoder
- **/include** - header files (.h)
  - `soldered_adsb_receiver.h` - the public API
  - `adsb_receiver_dfs.h` - definitions, setting values and decoded data types
  - `adsb_receiver_mavlink.h` - MAVLink parser state, used through the API above
- **/examples** - examples for using the library
  - `read_aircraft_csv` - default RUN state, decoded aircraft from the CSV stream
  - `device_info` - CONFIGURATION state, serial number, firmware version, `AT+HELP`, `AT+SETTINGS?`
  - `configure_protocols_and_settings` - writing every supported setting
  - `statistics` - system (`#S:`) and ADS-B (`#AS:`) statistics messages
  - `state_machine_and_lock` - full RESET -> RUN -> CONFIGURATION -> BOOTLOADER (locked) -> RUN cycle
  - `mavlink_adsb_vehicle` - MAVLink output, decoded ADSB_VEHICLE messages, generic frame callback
  - `custom_board_wiring` - driving a module wired to a different UART and pins
- **_other_** - idf_component.yml manifest file for ESP Component Registry

### Usage

On the Soldered board the module is wired to UART1 of the ESP32-S3 on IO17 and IO18, with its RESET pin on IO21, and leaves the factory sending CSV at 115200 baud. Opening the UART is all the setup it needs:

```c
static adsb_receiver_t adsb;
adsb_receiver_config_t config = ADSB_RECEIVER_DEFAULT_CONFIG();
ESP_ERROR_CHECK(adsb_receiver_init(&adsb, &config));

adsb_receiver_set_aircraft_callback(&adsb, on_aircraft, NULL);

while (1) {
    // Reads and decodes for 100 ms, firing callbacks, instead of sleeping through it
    ESP_ERROR_CHECK(adsb_receiver_poll_timeout(&adsb, 100));
}
```

**Reading data:** the module keeps sending whether or not anybody is listening, so `adsb_receiver_poll()` has to be called often enough that the UART receive buffer never overflows. In a loop that has nothing else to do, `adsb_receiver_poll_timeout()` replaces a plain delay and keeps decoding for the whole wait. Decoded messages are handed over through callbacks, which run in the task that polls: `adsb_receiver_set_aircraft_callback()`, `adsb_receiver_set_adsb_stats_callback()` and `adsb_receiver_set_system_stats_callback()` for the CSV stream, `adsb_receiver_set_adsb_vehicle_callback()` and `adsb_receiver_set_mavlink_frame_callback()` for MAVLink. `adsb_receiver_set_line_callback()` shows every raw line, which is the quickest way to see what is on the wire. CSV and MAVLink are told apart byte by byte, so the same application works whichever protocol the module is set to.

**Configuration:** settings can only be read and written in the CONFIGURATION state, which always runs at 115200 baud. `adsb_receiver_enter_configuration()` gets there and switches the local UART to match, and `adsb_receiver_exit_configuration()` goes back to RUN at the baud rate it is given, which must match the module's BAUDRATE setting. The typed setters cover every setting the SC1 supports; anything else goes through `adsb_receiver_set_setting()` and `adsb_receiver_get_setting()`, or straight through `adsb_receiver_send_at()`. Settings are kept by the module across power cycles.

```c
ESP_ERROR_CHECK(adsb_receiver_enter_configuration(&adsb));
ESP_ERROR_CHECK(adsb_receiver_set_adsb_rx_protocol_decoded(&adsb, ADSB_RECEIVER_DECODED_PROTOCOL_MAVLINK));
ESP_ERROR_CHECK(adsb_receiver_exit_configuration(&adsb, 115200));
```

**Custom wiring:** `ADSB_RECEIVER_DEFAULT_CONFIG()` describes the Soldered board; change `uart_port`, `rx_pin`, `tx_pin` and `reset_pin` for anything else, with `reset_pin = -1` if RESET is not wired. See the `custom_board_wiring` example.

**Stand-alone decoders:** `adsb_receiver_parse_aircraft_line()`, `adsb_receiver_parse_adsb_stats_line()`, `adsb_receiver_parse_system_stats_line()`, `adsb_receiver_parse_mavlink_adsb_vehicle()` and `adsb_receiver_crc16()` need no handle, so they also work on logged data.

The driver is not thread safe: use a handle from a single task, and do not call AT commands from inside a callback.

### Original source

This is a port of the [Soldered ADS-B Receiver Arduino library](https://github.com/SolderedElectronics/Soldered-ADSB-Receiver-Arduino-Library).

### Hardware design

You can find hardware design for this board in _Soldered ADS-B 1090MHz Receiver ESP32-S3_ hardware repository.

### Documentation

Access library documentation [here](https://docs.soldered.com/).

### About Soldered

<img src="https://raw.githubusercontent.com/SolderedElectronics/Soldered-Generic-Arduino-Library/dev/extras/Soldered-logo-color.png" alt="soldered-logo" width="500"/>

At Soldered, we design and manufacture a wide selection of electronic products to help you turn your ideas into acts and bring you one step closer to your final project. Our products are intented for makers and crafted in-house by our experienced team in Osijek, Croatia. We believe that sharing is a crucial element for improvement and innovation, and we work hard to stay connected with all our makers regardless of their skill or experience level. Therefore, all our products are open-source. Finally, we always have your back. If you face any problem concerning either your shopping experience or your electronics project, our team will help you deal with it, offering efficient customer service and cost-free technical support anytime. Some of those might be useful for you:

- [Web Store](https://www.soldered.com/shop)
- [Tutorials & Projects](https://soldered.com/learn)
- [Documentation](https://docs.soldered.com)

### Open-source license

Soldered invests vast amounts of time into hardware & software for these products, which are all open-source. Please support future development by buying one of our products.

Check license details in the LICENSE file. Long story short, use these open-source files for any purpose you want to, as long as you apply the same open-source licence to it and disclose the original source. No warranty - all designs in this repository are distributed in the hope that they will be useful, but without any warranty. They are provided "AS IS", therefore without warranty of any kind, either expressed or implied. The entire quality and performance of what you do with the contents of this repository are your responsibility. In no event, Soldered (TAVU) will be liable for your damages, losses, including any general, special, incidental or consequential damage arising out of the use or inability to use the contents of this repository.

## Have fun!

And thank you from your fellow makers at Soldered Electronics.
