/**
 * @file adsb_receiver_mavlink.c
 * @brief MAVLink frame parser and ADSB_VEHICLE decoder for the Aerobits TT-SC1b
 *
 * Framing follows the MAVLink v1 and v2 serialization described at
 * https://mavlink.io/en/guide/serialization.html
 *
 * @author Soldered Electronics
 */

#include <string.h>
#include "adsb_receiver_mavlink.h"
#include "soldered_adsb_receiver.h"

/* Length of the ADSB_VEHICLE payload with no trailing zeros trimmed off */
#define ADSB_VEHICLE_PAYLOAD_LEN 38

/* Length of the signature that may follow a v2 frame */
#define MAVLINK_SIGNATURE_LEN 13

/* v2 incompatibility flag announcing a signature */
#define MAVLINK_IFLAG_SIGNED 0x01

// *****************************************************************************
// Section: Small helpers

/**
 * @brief Add one byte to a CRC-16/MCRF4XX checksum, as used by MAVLink
 *
 * @param[in] data Byte to add
 * @param[in,out] crc Running checksum
 */
static void crc_accumulate(uint8_t data, uint16_t *crc)
{
    uint8_t tmp = data ^ (uint8_t)(*crc & 0xFF);
    tmp ^= (uint8_t)(tmp << 4);
    *crc = (uint16_t)((*crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ (tmp >> 4));
}

/**
 * @brief Look up the CRC_EXTRA byte of a message
 *
 * MAVLink's checksum needs a per-message CRC_EXTRA byte that is not part of the
 * wire format, so only the messages listed here can have their checksum checked.
 *
 * @param[in] msgid Message id
 * @param[out] extra CRC_EXTRA byte of the message
 *
 * @return true if the message is known
 */
static bool crc_extra_for(uint32_t msgid, uint8_t *extra)
{
    switch (msgid) {
    case ADSB_RECEIVER_MAVLINK_MSG_ID_HEARTBEAT: // Sent once a second alongside ADSB_VEHICLE
        *extra = 50;
        return true;
    case ADSB_RECEIVER_MAVLINK_MSG_ID_ADSB_VEHICLE: // The aircraft data the module sends
        *extra = 184;
        return true;
    default:
        return false;
    }
}

/**
 * @brief Read a little endian 16 bit value
 *
 * @param[in] p First byte
 *
 * @return Value read
 */
static uint16_t read_le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/**
 * @brief Read a little endian 32 bit value
 *
 * @param[in] p First byte
 *
 * @return Value read
 */
static uint32_t read_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/**
 * @brief Pull the fields out of a complete header
 *
 * @param[in,out] parser Parser holding a complete header
 */
static void decode_header(adsb_receiver_mavlink_parser_t *parser)
{
    adsb_receiver_mavlink_frame_t *frame = &parser->frame;
    const uint8_t *header = parser->header;

    if (frame->version == 1) {
        frame->payload_len = header[0];
        frame->seq = header[1];
        frame->sysid = header[2];
        frame->compid = header[3];
        frame->msgid = header[4];
        parser->incompat_flags = 0;
    } else {
        frame->payload_len = header[0];
        parser->incompat_flags = header[1];
        // header[2] holds the compat flags, which are not needed here
        frame->seq = header[3];
        frame->sysid = header[4];
        frame->compid = header[5];
        // The message id is 3 bytes, little endian
        frame->msgid = (uint32_t)header[6] | ((uint32_t)header[7] << 8) | ((uint32_t)header[8] << 16);
    }
}

// *****************************************************************************
// Section: Parser

void adsb_receiver_mavlink_reset(adsb_receiver_mavlink_parser_t *parser)
{
    parser->state = ADSB_RECEIVER_MAVLINK_STATE_IDLE;
}

bool adsb_receiver_mavlink_busy(const adsb_receiver_mavlink_parser_t *parser)
{
    return parser->state != ADSB_RECEIVER_MAVLINK_STATE_IDLE;
}

bool adsb_receiver_mavlink_start(adsb_receiver_mavlink_parser_t *parser, uint8_t start_byte)
{
    if (start_byte != ADSB_RECEIVER_MAVLINK_V1_STX && start_byte != ADSB_RECEIVER_MAVLINK_V2_STX) {
        return false;
    }

    parser->frame.version = (start_byte == ADSB_RECEIVER_MAVLINK_V1_STX) ? 1 : 2;
    parser->header_needed = (parser->frame.version == 1) ? 5 : 9;
    parser->header_got = 0;
    parser->crc_running = 0xFFFF;
    parser->state = ADSB_RECEIVER_MAVLINK_STATE_HEADER;

    return true;
}

bool adsb_receiver_mavlink_feed(adsb_receiver_mavlink_parser_t *parser, uint8_t byte)
{
    adsb_receiver_mavlink_frame_t *frame = &parser->frame;

    switch (parser->state) {
    case ADSB_RECEIVER_MAVLINK_STATE_HEADER:
        parser->header[parser->header_got++] = byte;
        crc_accumulate(byte, &parser->crc_running);
        if (parser->header_got < parser->header_needed) {
            break;
        }

        decode_header(parser);
        parser->payload_got = 0;
        parser->crc_got = 0;
        parser->state = (frame->payload_len == 0) ? ADSB_RECEIVER_MAVLINK_STATE_CHECKSUM :
                        ADSB_RECEIVER_MAVLINK_STATE_PAYLOAD;
        break;

    case ADSB_RECEIVER_MAVLINK_STATE_PAYLOAD:
        frame->payload[parser->payload_got++] = byte;
        crc_accumulate(byte, &parser->crc_running);
        if (parser->payload_got >= frame->payload_len) {
            parser->crc_got = 0;
            parser->state = ADSB_RECEIVER_MAVLINK_STATE_CHECKSUM;
        }
        break;

    case ADSB_RECEIVER_MAVLINK_STATE_CHECKSUM: {
        parser->crc_buf[parser->crc_got++] = byte;
        if (parser->crc_got < 2) {
            break;
        }

        uint16_t received = read_le16(parser->crc_buf);
        uint16_t computed = parser->crc_running;
        uint8_t extra;
        bool known_extra = crc_extra_for(frame->msgid, &extra);
        if (known_extra) {
            crc_accumulate(extra, &computed);
        }
        frame->crc_valid = known_extra && (computed == received);

        if (frame->version == 2 && (parser->incompat_flags & MAVLINK_IFLAG_SIGNED)) {
            parser->signature_got = 0;
            parser->state = ADSB_RECEIVER_MAVLINK_STATE_SIGNATURE;
        } else {
            parser->state = ADSB_RECEIVER_MAVLINK_STATE_IDLE;
        }
        return true;
    }

    case ADSB_RECEIVER_MAVLINK_STATE_SIGNATURE:
        if (++parser->signature_got >= MAVLINK_SIGNATURE_LEN) {
            parser->state = ADSB_RECEIVER_MAVLINK_STATE_IDLE;
        }
        break;

    case ADSB_RECEIVER_MAVLINK_STATE_IDLE:
    default:
        break;
    }

    return false;
}

// *****************************************************************************
// Section: Message decoders

esp_err_t adsb_receiver_parse_mavlink_adsb_vehicle(const adsb_receiver_mavlink_frame_t *frame,
                                                   adsb_receiver_adsb_vehicle_t *out)
{
    if (frame == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (frame->msgid != ADSB_RECEIVER_MAVLINK_MSG_ID_ADSB_VEHICLE) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* MAVLink 2 lets the sender trim trailing zero bytes off the payload, so pad
     * it back up to the full layout before pulling the fields out */
    uint8_t buf[ADSB_VEHICLE_PAYLOAD_LEN] = {0};
    size_t length = frame->payload_len < ADSB_VEHICLE_PAYLOAD_LEN ? frame->payload_len : ADSB_VEHICLE_PAYLOAD_LEN;
    memcpy(buf, frame->payload, length);

    memset(out, 0, sizeof(*out));
    out->icao_address = read_le32(&buf[0]);
    out->lat = (int32_t)read_le32(&buf[4]);
    out->lon = (int32_t)read_le32(&buf[8]);
    out->altitude = (int32_t)read_le32(&buf[12]);
    out->heading = read_le16(&buf[16]);
    out->hor_velocity = read_le16(&buf[18]);
    out->ver_velocity = (int16_t)read_le16(&buf[20]);
    out->flags = read_le16(&buf[22]);
    out->squawk = read_le16(&buf[24]);
    out->altitude_type = (adsb_receiver_altitude_type_t)buf[26];
    memcpy(out->callsign, &buf[27], 8);
    out->callsign[8] = '\0';
    out->emitter_type = (adsb_receiver_emitter_type_t)buf[36];
    out->tslc = buf[37];

    return ESP_OK;
}
