/**
 * @file adsb_receiver_csv.c
 * @brief CSV protocol decoders for the Aerobits TT-SC1b
 *
 * Every CSV line the module sends starts with a '#' prefix naming the message
 * and ends with a CRC16 field over everything before its last comma, as
 * described in the Aerobits OEM SC1/SC1-EXT datasheet.
 *
 * @author Soldered Electronics
 */

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "soldered_adsb_receiver.h"

/* Field count of each message, CRC included */
#define AIRCRAFT_FIELDS     17
#define ADSB_STATS_FIELDS   4
#define SYSTEM_STATS_FIELDS 3

// *****************************************************************************
// Section: Small helpers

/**
 * @brief Find a line with the whitespace at both ends taken off, without copying it
 *
 * @param[in] line Line
 * @param[out] length Length of the trimmed line
 *
 * @return Start of the trimmed line
 */
static const char *trim(const char *line, size_t *length)
{
    while (*line != '\0' && isspace((unsigned char) * line)) {
        line++;
    }

    size_t n = strlen(line);
    while (n > 0 && isspace((unsigned char)line[n - 1])) {
        n--;
    }

    *length = n;
    return line;
}

/**
 * @brief Copy the fields of a message, everything after its prefix, into a buffer to split
 *
 * @param[in] line Line as received
 * @param[in] prefix Message prefix, such as "#A:"
 * @param[out] out Buffer to copy the fields into, ::ADSB_RECEIVER_MAX_LINE_LEN + 1 long
 *
 * @return true if the line starts with the prefix
 */
static bool copy_fields(const char *line, const char *prefix, char *out)
{
    size_t length;
    const char *start = trim(line, &length);
    size_t prefix_len = strlen(prefix);

    if (length < prefix_len || strncmp(start, prefix, prefix_len) != 0) {
        return false;
    }

    start += prefix_len;
    length -= prefix_len;
    if (length > ADSB_RECEIVER_MAX_LINE_LEN) {
        length = ADSB_RECEIVER_MAX_LINE_LEN;
    }
    memcpy(out, start, length);
    out[length] = '\0';

    return true;
}

/**
 * @brief Split a line on commas, in place
 *
 * Stops after `max_out` fields, so anything past the last field kept is
 * dropped.
 *
 * @param[in,out] s Line to split, commas are overwritten with terminators
 * @param[out] out Pointers to the start of every field
 * @param[in] max_out Most fields to split off
 *
 * @return How many fields were split off
 */
static int split_csv(char *s, char **out, int max_out)
{
    int count = 0;
    char *start = s;

    for (char *p = s; count < max_out; p++) {
        if (*p == ',' || *p == '\0') {
            bool end = (*p == '\0');
            *p = '\0';
            out[count++] = start;
            if (end) {
                break;
            }
            start = p + 1;
        }
    }

    return count;
}

/**
 * @brief Copy a string field into a fixed size buffer, cutting it short if needed
 *
 * @param[out] out Buffer
 * @param[in] out_size Size of the buffer
 * @param[in] field Field to copy
 */
static void copy_field(char *out, size_t out_size, const char *field)
{
    strncpy(out, field, out_size - 1);
    out[out_size - 1] = '\0';
}

/**
 * @brief Read a hexadecimal field
 *
 * @param[in] field Field
 *
 * @return Value, 0 if the field holds no number
 */
static uint32_t hex_field(const char *field)
{
    return (uint32_t)strtoul(field, NULL, 16);
}

/**
 * @brief Read a decimal integer field
 *
 * @param[in] field Field
 *
 * @return Value, 0 if the field holds no number
 */
static long int_field(const char *field)
{
    return strtol(field, NULL, 10);
}

/**
 * @brief Check the CRC of a line
 *
 * @param[in] line Line as received
 * @param[out] crc_valid Whether the CRC matched, left alone if the line has no CRC
 */
static void check_crc(const char *line, bool *crc_valid)
{
    uint16_t computed, received;
    if (adsb_receiver_verify_csv_crc(line, &computed, &received)) {
        *crc_valid = (computed == received);
    }
}

// *****************************************************************************
// Section: Decoders

uint16_t adsb_receiver_crc16(const uint8_t *data, size_t length)
{
    uint8_t x;
    uint16_t crc = 0xFFFF;

    if (data == NULL) {
        return crc;
    }

    while (length--) {
        x = (uint8_t)((crc >> 8) ^ *data++);
        x ^= x >> 4;
        crc = (uint16_t)((crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x << 5)) ^ ((uint16_t)x));
    }

    // The module sends the CRC with its bytes swapped
    return (uint16_t)((crc >> 8) | (crc << 8));
}

bool adsb_receiver_verify_csv_crc(const char *line, uint16_t *computed, uint16_t *received)
{
    if (line == NULL || computed == NULL || received == NULL) {
        return false;
    }

    size_t length;
    const char *start = trim(line, &length);

    const char *last_comma = NULL;
    for (size_t i = 0; i < length; i++) {
        if (start[i] == ',') {
            last_comma = &start[i];
        }
    }
    if (last_comma == NULL) {
        return false;
    }

    // The CRC field is a handful of hex digits, anything beyond that is not a CRC
    char crc_part[16];
    size_t crc_len;
    const char *crc_start = trim(last_comma + 1, &crc_len);
    if (crc_start + crc_len > start + length) {
        crc_len = (size_t)(start + length - crc_start);
    }
    if (crc_len == 0) {
        return false;
    }
    if (crc_len >= sizeof(crc_part)) {
        crc_len = sizeof(crc_part) - 1;
    }
    memcpy(crc_part, crc_start, crc_len);
    crc_part[crc_len] = '\0';

    *computed = adsb_receiver_crc16((const uint8_t *)start, (size_t)(last_comma - start));
    *received = (uint16_t)hex_field(crc_part);

    return true;
}

esp_err_t adsb_receiver_parse_aircraft_line(const char *line, adsb_receiver_aircraft_t *out)
{
    if (line == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    char fields_buf[ADSB_RECEIVER_MAX_LINE_LEN + 1];
    if (!copy_fields(line, "#A:", fields_buf)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    char *f[AIRCRAFT_FIELDS];
    if (split_csv(fields_buf, f, AIRCRAFT_FIELDS) < AIRCRAFT_FIELDS) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    memset(out, 0, sizeof(*out));
    copy_field(out->icao, sizeof(out->icao), f[0]);
    if (f[1][0]) {
        out->flags = (uint16_t)hex_field(f[1]);
    }
    copy_field(out->callsign, sizeof(out->callsign), f[2]);
    copy_field(out->squawk, sizeof(out->squawk), f[3]);
    if (f[4][0] && f[5][0]) {
        out->has_position = true;
        out->lat = strtod(f[4], NULL);
        out->lon = strtod(f[5], NULL);
    }
    if (f[6][0]) {
        out->has_alt_baro = true;
        out->alt_baro = (int32_t)int_field(f[6]);
    }
    if (f[7][0]) {
        out->has_track = true;
        out->track = strtof(f[7], NULL);
    }
    if (f[8][0]) {
        out->has_vel_h = true;
        out->vel_h = strtof(f[8], NULL);
    }
    if (f[9][0]) {
        out->has_vel_v = true;
        out->vel_v = (int32_t)int_field(f[9]);
    }
    if (f[10][0]) {
        out->has_sig_s = true;
        out->sig_s = (int16_t)int_field(f[10]);
    }
    if (f[11][0]) {
        out->has_sig_q = true;
        out->sig_q = (uint8_t)int_field(f[11]);
    }
    out->fps = (uint16_t)int_field(f[12]);
    if (f[13][0]) {
        out->has_nic_nac = true;
        out->nic_nac = (uint16_t)hex_field(f[13]);
    }
    if (f[14][0]) {
        out->has_alt_geo = true;
        out->alt_geo = (int32_t)int_field(f[14]);
    }
    if (f[15][0]) {
        out->has_ecat = true;
        out->ecat = (uint8_t)int_field(f[15]);
    }
    if (f[16][0]) {
        out->crc = (uint16_t)hex_field(f[16]);
    }

    check_crc(line, &out->crc_valid);

    return ESP_OK;
}

esp_err_t adsb_receiver_parse_adsb_stats_line(const char *line, adsb_receiver_adsb_stats_t *out)
{
    if (line == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    char fields_buf[ADSB_RECEIVER_MAX_LINE_LEN + 1];
    if (!copy_fields(line, "#AS:", fields_buf)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    char *f[ADSB_STATS_FIELDS];
    if (split_csv(fields_buf, f, ADSB_STATS_FIELDS) < ADSB_STATS_FIELDS) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    memset(out, 0, sizeof(*out));
    out->fpss = (uint32_t)int_field(f[0]);
    out->fpsac = (uint32_t)int_field(f[1]);
    out->calib = strtof(f[2], NULL);
    out->crc = (uint16_t)hex_field(f[3]);

    check_crc(line, &out->crc_valid);

    return ESP_OK;
}

esp_err_t adsb_receiver_parse_system_stats_line(const char *line, adsb_receiver_system_stats_t *out)
{
    if (line == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    char fields_buf[ADSB_RECEIVER_MAX_LINE_LEN + 1];
    if (!copy_fields(line, "#S:", fields_buf)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    char *f[SYSTEM_STATS_FIELDS];
    if (split_csv(fields_buf, f, SYSTEM_STATS_FIELDS) < SYSTEM_STATS_FIELDS) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    memset(out, 0, sizeof(*out));
    out->cpu_load = strtof(f[0], NULL);
    out->uptime = (uint32_t)int_field(f[1]);
    out->crc = (uint16_t)hex_field(f[2]);

    check_crc(line, &out->crc_valid);

    return ESP_OK;
}
