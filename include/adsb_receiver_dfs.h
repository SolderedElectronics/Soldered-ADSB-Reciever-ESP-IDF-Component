/**
 * @file adsb_receiver_dfs.h
 * @brief Definitions, enumerations and data types for the Aerobits TT-SC1b
 *
 * The AT command set, the CSV data protocol and the settings described here
 * follow the Aerobits OEM SC1/SC1-EXT datasheet. The MAVLink ADSB_VEHICLE
 * message follows the MAVLink common message set:
 * https://mavlink.io/en/messages/common.html#ADSB_VEHICLE
 *
 * Only what the base SC1 module supports is covered: CSV and MAVLink output.
 * Mode-A/C/S decode, multilateration timestamps and the JSON, GDL90, ASTERIX
 * and RAW protocols are exclusive to the SC1-EXT variant.
 *
 * @author Soldered Electronics
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

// *****************************************************************************
// Section: Defaults
//
// The Soldered ADS-B 1090MHz Receiver board wires the module to UART1 of the
// onboard ESP32-S3, on IO17 (module TX) and IO18 (module RX), with its RESET
// pin on IO21. The module leaves the factory at 115200 baud.

#define ADSB_RECEIVER_DEFAULT_RX_PIN         17     /**< ESP32 pin wired to the TX pin of the module */
#define ADSB_RECEIVER_DEFAULT_TX_PIN         18     /**< ESP32 pin wired to the RX pin of the module */
#define ADSB_RECEIVER_DEFAULT_RESET_PIN      21     /**< ESP32 pin wired to the RESET pin of the module */
#define ADSB_RECEIVER_DEFAULT_BAUD_RATE      115200 /**< Baud rate of the RUN state out of the box */
#define ADSB_RECEIVER_DEFAULT_RX_BUFFER_SIZE 4096   /**< Receive ring buffer, roomy enough for dense traffic */
#define ADSB_RECEIVER_MIN_RX_BUFFER_SIZE     256    /**< Smallest receive ring buffer accepted */

/** Fixed baud rate of the CONFIGURATION and BOOTLOADER states */
#define ADSB_RECEIVER_CONFIG_BAUD_RATE 115200

/** Longest line kept while assembling CSV and AT lines, longer ones are dropped */
#define ADSB_RECEIVER_MAX_LINE_LEN 512

/** Longest AT command body adsb_receiver_send_at() accepts */
#define ADSB_RECEIVER_MAX_COMMAND_LEN 128

// *****************************************************************************
// Section: Field sizes
//
// Sizes include the string terminator. Anything longer on the wire is cut short.

#define ADSB_RECEIVER_ICAO_SIZE     8  /**< ICAO address, 6 hex digits */
#define ADSB_RECEIVER_CALLSIGN_SIZE 9  /**< Callsign, up to 8 characters */
#define ADSB_RECEIVER_SQUAWK_SIZE   8  /**< Squawk, 4 octal digits */

/** Largest payload a MAVLink frame can carry */
#define ADSB_RECEIVER_MAVLINK_MAX_PAYLOAD_LEN 255

/** MAVLink message id of HEARTBEAT */
#define ADSB_RECEIVER_MAVLINK_MSG_ID_HEARTBEAT 0
/** MAVLink message id of ADSB_VEHICLE */
#define ADSB_RECEIVER_MAVLINK_MSG_ID_ADSB_VEHICLE 246

// *****************************************************************************
// Section: Settings

/**
 * @brief Values of the ADSB_RX_PROTOCOL_DECODED setting supported by the SC1
 */
typedef enum {
    ADSB_RECEIVER_DECODED_PROTOCOL_NONE = 0,    /**< No decoded aircraft output */
    ADSB_RECEIVER_DECODED_PROTOCOL_CSV = 1,     /**< "#A:" CSV lines, the default */
    ADSB_RECEIVER_DECODED_PROTOCOL_MAVLINK = 2, /**< MAVLink ADSB_VEHICLE messages */
} adsb_receiver_decoded_protocol_t;

/**
 * @brief Values of the SYSTEM_STATISTICS and ADSB_STATISTICS settings supported by the SC1
 */
typedef enum {
    ADSB_RECEIVER_STATS_PROTOCOL_NONE = 0, /**< No statistics output, the default */
    ADSB_RECEIVER_STATS_PROTOCOL_CSV = 1,  /**< "#S:" or "#AS:" CSV lines once a second */
} adsb_receiver_stats_protocol_t;

/**
 * @brief Values of the BAUDRATE setting, the baud rate of the main UART in the RUN state
 */
typedef enum {
    ADSB_RECEIVER_BAUDRATE_115200 = 0,  /**< 115200 baud, the default */
    ADSB_RECEIVER_BAUDRATE_921600 = 1,  /**< 921600 baud */
    ADSB_RECEIVER_BAUDRATE_3000000 = 2, /**< 3000000 baud */
    ADSB_RECEIVER_BAUDRATE_57600 = 3,   /**< 57600 baud */
} adsb_receiver_baudrate_t;

/**
 * @brief Values of the ADSB_RX_PROTOCOL_INC setting, how often aircraft reports are sent
 */
typedef enum {
    ADSB_RECEIVER_INC_ALWAYS_ONCE_PER_SECOND = 0,       /**< Every aircraft once a second */
    ADSB_RECEIVER_INC_ONCE_PER_SECOND_IF_UPDATED = 1,   /**< Once a second, only aircraft that changed */
    ADSB_RECEIVER_INC_IMMEDIATE_ON_POSITION_UPDATE = 2, /**< As soon as a new position comes in */
} adsb_receiver_inc_mode_t;

/**
 * @brief Values of AT+CONFIG?, the state the module reports being in
 */
typedef enum {
    ADSB_RECEIVER_CONFIG_STATE_RUN = 0,             /**< RUN state */
    ADSB_RECEIVER_CONFIG_STATE_CONFIG_115200 = 1,   /**< CONFIGURATION state at 115200 baud */
    ADSB_RECEIVER_CONFIG_STATE_CONFIG_SET_BAUD = 2, /**< CONFIGURATION state at the BAUDRATE setting */
} adsb_receiver_config_state_t;

// *****************************************************************************
// Section: CSV messages

/** FLAGS bits of a "#A:" aircraft message */
#define ADSB_RECEIVER_AIRCRAFT_FLAG_ON_GROUND        0x0001 /**< Aircraft is on the ground */
#define ADSB_RECEIVER_AIRCRAFT_FLAG_MILITARY         0x0002 /**< Aircraft is military */
#define ADSB_RECEIVER_AIRCRAFT_FLAG_UPDATE_ALT_BARO  0x0100 /**< Barometric altitude was updated */
#define ADSB_RECEIVER_AIRCRAFT_FLAG_UPDATE_POSITION  0x0200 /**< Position was updated */
#define ADSB_RECEIVER_AIRCRAFT_FLAG_UPDATE_TRACK     0x0400 /**< Track was updated */
#define ADSB_RECEIVER_AIRCRAFT_FLAG_UPDATE_VELO_H    0x0800 /**< Horizontal velocity was updated */
#define ADSB_RECEIVER_AIRCRAFT_FLAG_UPDATE_VELO_V    0x1000 /**< Vertical velocity was updated */
#define ADSB_RECEIVER_AIRCRAFT_FLAG_UPDATE_ALT_GEO   0x2000 /**< Geometric altitude was updated */

/**
 * @brief Decoded "#A:" aircraft state vector
 *
 * Any field the module had no data for is sent empty; its `has_` flag is then
 * false and the value is left at zero.
 */
typedef struct {
    char icao[ADSB_RECEIVER_ICAO_SIZE];         /**< ICAO 24-bit address, hex string */
    uint16_t flags;                             /**< ADSB_RECEIVER_AIRCRAFT_FLAG_ bits */
    char callsign[ADSB_RECEIVER_CALLSIGN_SIZE]; /**< Callsign, empty if not known */
    char squawk[ADSB_RECEIVER_SQUAWK_SIZE];     /**< Squawk, empty if not known */
    bool has_position;                          /**< lat and lon are valid */
    double lat;                                 /**< Latitude in degrees */
    double lon;                                 /**< Longitude in degrees */
    bool has_alt_baro;                          /**< alt_baro is valid */
    int32_t alt_baro;                           /**< Barometric altitude in feet */
    bool has_track;                             /**< track is valid */
    float track;                                /**< Track in degrees, 0 to 360 */
    bool has_vel_h;                             /**< vel_h is valid */
    float vel_h;                                /**< Horizontal velocity in knots */
    bool has_vel_v;                             /**< vel_v is valid */
    int32_t vel_v;                              /**< Vertical velocity in ft/min */
    bool has_sig_s;                             /**< sig_s is valid */
    int16_t sig_s;                              /**< Signal strength in dBm */
    bool has_sig_q;                             /**< sig_q is valid */
    uint8_t sig_q;                              /**< Signal quality in dB */
    uint16_t fps;                               /**< Raw Mode-S frames received last second */
    bool has_nic_nac;                           /**< nic_nac is valid */
    uint16_t nic_nac;                           /**< Raw NIC/NAC bitfield */
    bool has_alt_geo;                           /**< alt_geo is valid */
    int32_t alt_geo;                            /**< Geometric altitude in feet */
    bool has_ecat;                              /**< ecat is valid */
    uint8_t ecat;                               /**< Emitter category code */
    uint16_t crc;                               /**< CRC sent with the line */
    bool crc_valid;                             /**< CRC sent matches the one worked out */
} adsb_receiver_aircraft_t;

/**
 * @brief Decoded "#AS:" ADS-B receiver statistics
 */
typedef struct {
    uint32_t fpss;  /**< Mode-S frames received per second */
    uint32_t fpsac; /**< Mode-A/C frames received per second */
    float calib;    /**< Real microcontroller frequency, calibrated against the GNSS PPS */
    uint16_t crc;   /**< CRC sent with the line */
    bool crc_valid; /**< CRC sent matches the one worked out */
} adsb_receiver_adsb_stats_t;

/**
 * @brief Decoded "#S:" system statistics
 */
typedef struct {
    float cpu_load;  /**< CPU load in percent */
    uint32_t uptime; /**< Uptime in seconds */
    uint16_t crc;    /**< CRC sent with the line */
    bool crc_valid;  /**< CRC sent matches the one worked out */
} adsb_receiver_system_stats_t;

/**
 * @brief Check whether the aircraft is on the ground
 *
 * @param[in] aircraft Decoded aircraft
 *
 * @return true if ::ADSB_RECEIVER_AIRCRAFT_FLAG_ON_GROUND is set
 */
static inline bool adsb_receiver_aircraft_on_ground(const adsb_receiver_aircraft_t *aircraft)
{
    return (aircraft->flags & ADSB_RECEIVER_AIRCRAFT_FLAG_ON_GROUND) != 0;
}

/**
 * @brief Check whether the aircraft is military
 *
 * @param[in] aircraft Decoded aircraft
 *
 * @return true if ::ADSB_RECEIVER_AIRCRAFT_FLAG_MILITARY is set
 */
static inline bool adsb_receiver_aircraft_is_military(const adsb_receiver_aircraft_t *aircraft)
{
    return (aircraft->flags & ADSB_RECEIVER_AIRCRAFT_FLAG_MILITARY) != 0;
}

/**
 * @brief Check a bit of the aircraft flags, such as which fields were updated
 *
 * @param[in] aircraft Decoded aircraft
 * @param[in] flag One of the ADSB_RECEIVER_AIRCRAFT_FLAG_ bits
 *
 * @return true if the bit is set
 */
static inline bool adsb_receiver_aircraft_has_flag(const adsb_receiver_aircraft_t *aircraft, uint16_t flag)
{
    return (aircraft->flags & flag) != 0;
}

// *****************************************************************************
// Section: MAVLink messages

/**
 * @brief One raw MAVLink v1 or v2 frame, whatever its message id
 *
 * `crc_valid` can only be true for message ids whose CRC_EXTRA byte is known,
 * HEARTBEAT and ADSB_VEHICLE. Any other message stays false even when the frame
 * is fine, since its checksum cannot be checked without that constant.
 */
typedef struct {
    uint8_t version;                                        /**< 1 or 2 */
    uint8_t sysid;                                          /**< System id of the sender */
    uint8_t compid;                                         /**< Component id of the sender */
    uint8_t seq;                                            /**< Sequence number */
    uint32_t msgid;                                         /**< Message id */
    uint8_t payload[ADSB_RECEIVER_MAVLINK_MAX_PAYLOAD_LEN]; /**< Payload as received */
    uint8_t payload_len;                                    /**< Bytes in payload */
    bool crc_valid;                                         /**< Checksum was checked and matched */
} adsb_receiver_mavlink_frame_t;

/**
 * @brief Values of the ADSB_VEHICLE altitude_type field
 */
typedef enum {
    ADSB_RECEIVER_ALTITUDE_TYPE_PRESSURE_QNH = 0, /**< Barometric altitude */
    ADSB_RECEIVER_ALTITUDE_TYPE_GEOMETRIC = 1,    /**< GNSS altitude */
} adsb_receiver_altitude_type_t;

/**
 * @brief Values of the ADSB_VEHICLE emitter_type field
 */
typedef enum {
    ADSB_RECEIVER_EMITTER_NO_INFO = 0,
    ADSB_RECEIVER_EMITTER_LIGHT = 1,
    ADSB_RECEIVER_EMITTER_SMALL = 2,
    ADSB_RECEIVER_EMITTER_LARGE = 3,
    ADSB_RECEIVER_EMITTER_HIGH_VORTEX_LARGE = 4,
    ADSB_RECEIVER_EMITTER_HEAVY = 5,
    ADSB_RECEIVER_EMITTER_HIGHLY_MANUV = 6,
    ADSB_RECEIVER_EMITTER_ROTORCRAFT = 7,
    ADSB_RECEIVER_EMITTER_UNASSIGNED = 8,
    ADSB_RECEIVER_EMITTER_GLIDER = 9,
    ADSB_RECEIVER_EMITTER_LIGHTER_AIR = 10,
    ADSB_RECEIVER_EMITTER_PARACHUTE = 11,
    ADSB_RECEIVER_EMITTER_ULTRA_LIGHT = 12,
    ADSB_RECEIVER_EMITTER_UNASSIGNED2 = 13,
    ADSB_RECEIVER_EMITTER_UAV = 14,
    ADSB_RECEIVER_EMITTER_SPACE = 15,
    ADSB_RECEIVER_EMITTER_UNASSIGNED3 = 16,
    ADSB_RECEIVER_EMITTER_EMERGENCY_SURFACE = 17,
    ADSB_RECEIVER_EMITTER_SERVICE_SURFACE = 18,
    ADSB_RECEIVER_EMITTER_POINT_OBSTACLE = 19,
} adsb_receiver_emitter_type_t;

/** Bits of the ADSB_VEHICLE flags field */
#define ADSB_RECEIVER_VEHICLE_FLAG_VALID_COORDS            0x0001 /**< lat and lon are valid */
#define ADSB_RECEIVER_VEHICLE_FLAG_VALID_ALTITUDE          0x0002 /**< altitude is valid */
#define ADSB_RECEIVER_VEHICLE_FLAG_VALID_HEADING           0x0004 /**< heading is valid */
#define ADSB_RECEIVER_VEHICLE_FLAG_VALID_VELOCITY          0x0008 /**< hor_velocity is valid */
#define ADSB_RECEIVER_VEHICLE_FLAG_VALID_CALLSIGN          0x0010 /**< callsign is valid */
#define ADSB_RECEIVER_VEHICLE_FLAG_VALID_SQUAWK            0x0020 /**< squawk is valid */
#define ADSB_RECEIVER_VEHICLE_FLAG_SIMULATED               0x0040 /**< Simulated aircraft */
#define ADSB_RECEIVER_VEHICLE_FLAG_VERTICAL_VELOCITY_VALID 0x0080 /**< ver_velocity is valid */
#define ADSB_RECEIVER_VEHICLE_FLAG_BARO_VALID              0x0100 /**< Barometric altitude is valid */
#define ADSB_RECEIVER_VEHICLE_FLAG_SOURCE_UAT              0x8000 /**< Received over UAT, not 1090ES */

/**
 * @brief Decoded MAVLink ADSB_VEHICLE message, one aircraft's state
 */
typedef struct {
    uint32_t icao_address;                        /**< ICAO 24-bit address */
    int32_t lat;                                  /**< Latitude in degrees * 1e7 */
    int32_t lon;                                  /**< Longitude in degrees * 1e7 */
    adsb_receiver_altitude_type_t altitude_type;  /**< What altitude is measured against */
    int32_t altitude;                             /**< Altitude in mm */
    uint16_t heading;                             /**< Heading in centidegrees */
    uint16_t hor_velocity;                        /**< Horizontal velocity in cm/s */
    int16_t ver_velocity;                         /**< Vertical velocity in cm/s, positive up */
    char callsign[ADSB_RECEIVER_CALLSIGN_SIZE];   /**< Callsign, NUL terminated */
    adsb_receiver_emitter_type_t emitter_type;    /**< Kind of aircraft */
    uint8_t tslc;                                 /**< Seconds since last contact */
    uint16_t flags;                               /**< ADSB_RECEIVER_VEHICLE_FLAG_ bits */
    uint16_t squawk;                              /**< Squawk code */
} adsb_receiver_adsb_vehicle_t;

/**
 * @brief Latitude of a vehicle in degrees
 *
 * @param[in] vehicle Decoded vehicle
 *
 * @return Latitude in degrees
 */
static inline double adsb_receiver_vehicle_lat_degrees(const adsb_receiver_adsb_vehicle_t *vehicle)
{
    return vehicle->lat / 1e7;
}

/**
 * @brief Longitude of a vehicle in degrees
 *
 * @param[in] vehicle Decoded vehicle
 *
 * @return Longitude in degrees
 */
static inline double adsb_receiver_vehicle_lon_degrees(const adsb_receiver_adsb_vehicle_t *vehicle)
{
    return vehicle->lon / 1e7;
}

/**
 * @brief Altitude of a vehicle in metres
 *
 * @param[in] vehicle Decoded vehicle
 *
 * @return Altitude in metres
 */
static inline double adsb_receiver_vehicle_altitude_meters(const adsb_receiver_adsb_vehicle_t *vehicle)
{
    return vehicle->altitude / 1000.0;
}

/**
 * @brief Check a bit of the vehicle flags
 *
 * @param[in] vehicle Decoded vehicle
 * @param[in] flag One of the ADSB_RECEIVER_VEHICLE_FLAG_ bits
 *
 * @return true if the bit is set
 */
static inline bool adsb_receiver_vehicle_has_flag(const adsb_receiver_adsb_vehicle_t *vehicle, uint16_t flag)
{
    return (vehicle->flags & flag) != 0;
}

#ifdef __cplusplus
}
#endif
