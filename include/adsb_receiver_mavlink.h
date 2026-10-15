/**
 * @file adsb_receiver_mavlink.h
 * @brief MAVLink frame parser state for the Aerobits TT-SC1b
 *
 * The module sends MAVLink v1 or v2 frames when its decoded output is set to
 * MAVLink. The parser here is fed one byte at a time by the driver; it is
 * exposed only because the driver handle holds one, and is used through the API
 * in soldered_adsb_receiver.h.
 *
 * @author Soldered Electronics
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "adsb_receiver_dfs.h"

/** Start byte of a MAVLink v1 frame */
#define ADSB_RECEIVER_MAVLINK_V1_STX 0xFE
/** Start byte of a MAVLink v2 frame */
#define ADSB_RECEIVER_MAVLINK_V2_STX 0xFD

/**
 * @brief Where the parser is within a frame
 */
typedef enum {
    ADSB_RECEIVER_MAVLINK_STATE_IDLE = 0, /**< Not inside a frame */
    ADSB_RECEIVER_MAVLINK_STATE_HEADER,   /**< Collecting the header after the start byte */
    ADSB_RECEIVER_MAVLINK_STATE_PAYLOAD,  /**< Collecting the payload */
    ADSB_RECEIVER_MAVLINK_STATE_CHECKSUM, /**< Collecting the two checksum bytes */
    ADSB_RECEIVER_MAVLINK_STATE_SIGNATURE /**< Skipping the 13 byte v2 signature */
} adsb_receiver_mavlink_state_t;

/**
 * @brief MAVLink frame parser
 */
typedef struct {
    adsb_receiver_mavlink_state_t state; /**< Where the parser is */
    adsb_receiver_mavlink_frame_t frame; /**< Frame being assembled */
    uint8_t header[9];                   /**< Header bytes after the start byte */
    uint8_t header_needed;               /**< Header length, 5 for v1 and 9 for v2 */
    uint8_t header_got;                  /**< Header bytes collected so far */
    uint8_t incompat_flags;              /**< v2 incompatibility flags */
    uint8_t payload_got;                 /**< Payload bytes collected so far */
    uint16_t crc_running;                /**< Checksum over everything after the start byte */
    uint8_t crc_buf[2];                  /**< Checksum bytes as received */
    uint8_t crc_got;                     /**< Checksum bytes collected so far */
    uint8_t signature_got;               /**< Signature bytes skipped so far */
} adsb_receiver_mavlink_parser_t;

/**
 * @brief Put the parser back into its idle state
 *
 * @param[out] parser Parser
 */
void adsb_receiver_mavlink_reset(adsb_receiver_mavlink_parser_t *parser);

/**
 * @brief Check whether the parser is inside a frame
 *
 * @param[in] parser Parser
 *
 * @return true if the next byte belongs to a frame
 */
bool adsb_receiver_mavlink_busy(const adsb_receiver_mavlink_parser_t *parser);

/**
 * @brief Start a frame on a received start byte
 *
 * @param[in,out] parser Parser
 * @param[in] start_byte ::ADSB_RECEIVER_MAVLINK_V1_STX or ::ADSB_RECEIVER_MAVLINK_V2_STX
 *
 * @return true if the byte was a start byte and a frame was started
 */
bool adsb_receiver_mavlink_start(adsb_receiver_mavlink_parser_t *parser, uint8_t start_byte);

/**
 * @brief Feed the next byte of a frame to the parser
 *
 * Returns true as soon as the checksum has been received, with the frame in
 * `parser->frame`. A v2 signature that follows is skipped by later calls.
 *
 * @param[in,out] parser Parser
 * @param[in] byte Received byte
 *
 * @return true when a whole frame has been received
 */
bool adsb_receiver_mavlink_feed(adsb_receiver_mavlink_parser_t *parser, uint8_t byte);

#ifdef __cplusplus
}
#endif
