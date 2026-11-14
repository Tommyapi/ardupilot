/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_ERDI_E2_ENABLED

#include "AP_RangeFinder_ERDI_E2.h"
#include <AP_HAL/AP_HAL.h>

#define ERDI_E2_HEADER1             0x55
#define ERDI_E2_HEADER2             0xAA
#define ERDI_E2_FRAME_LEN           8
#define ERDI_E2_FN_SINGLE           0x88
#define ERDI_E2_FN_CONTINUOUS       0x89
#define ERDI_E2_STA_OK              0x01
#define ERDI_E2_START_CMD_INTERVAL_MS 2000

extern const AP_HAL::HAL& hal;

void AP_RangeFinder_ERDI_E2::send_start_command()
{
    // "Continuous Ranging | Send | 55 AA 89 FF FF FF FF 85"
    static const uint8_t cmd[] { 0x55, 0xAA, ERDI_E2_FN_CONTINUOUS, 0xFF, 0xFF, 0xFF, 0xFF, 0x85 };
    uart->write(cmd, sizeof(cmd));
}

/*
  discard the first byte of the line buffer and move the buffer start
  to the next frame header candidate
 */
void AP_RangeFinder_ERDI_E2::resync()
{
    for (uint8_t i = 1; i < linebuf_len; i++) {
        if (linebuf[i] == ERDI_E2_HEADER1 &&
            (i + 1 == linebuf_len || linebuf[i+1] == ERDI_E2_HEADER2)) {
            memmove(&linebuf[0], &linebuf[i], linebuf_len - i);
            linebuf_len -= i;
            return;
        }
    }
    linebuf_len = 0;
}

/*
  read all available frames and return the average distance in metres
 */
bool AP_RangeFinder_ERDI_E2::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    // (re)start continuous ranging at boot and whenever the module
    // has gone quiet, e.g. after it was power cycled
    const uint32_t now_ms = AP_HAL::millis();
    if (!start_cmd_sent ||
        (now_ms - last_frame_ms > ERDI_E2_START_CMD_INTERVAL_MS &&
         now_ms - last_start_cmd_ms > ERDI_E2_START_CMD_INTERVAL_MS)) {
        send_start_command();
        start_cmd_sent = true;
        last_start_cmd_ms = now_ms;
    }

    float sum_m = 0;
    uint16_t count = 0;
    uint16_t count_out_of_range = 0;

    for (uint16_t j = 0; j < 1024; j++) {
        uint8_t c;
        if (!uart->read(c)) {
            break;
        }
        if (linebuf_len == 0 && c != ERDI_E2_HEADER1) {
            continue;
        }
        if (linebuf_len == 1 && c != ERDI_E2_HEADER2) {
            linebuf_len = (c == ERDI_E2_HEADER1) ? 1 : 0;
            continue;
        }
        linebuf[linebuf_len++] = c;
        if (linebuf_len < ERDI_E2_FRAME_LEN) {
            continue;
        }

        // module->host checksum covers the header too
        uint8_t checksum = 0;
        for (uint8_t i = 0; i < ERDI_E2_FRAME_LEN - 1; i++) {
            checksum += linebuf[i];
        }
        if (checksum != linebuf[ERDI_E2_FRAME_LEN - 1]) {
            resync();
            continue;
        }

        const uint8_t function = linebuf[2];
        const uint8_t sta = linebuf[3];
        const uint16_t raw = uint16_t(linebuf[5]) << 8 | linebuf[6];
        linebuf_len = 0;

        if (function != ERDI_E2_FN_CONTINUOUS && function != ERDI_E2_FN_SINGLE) {
            // e.g. the power-on self-test report or a stop reply
            continue;
        }
        last_frame_ms = now_ms;

        const float frame_m = raw * 0.1f;
        if (sta != ERDI_E2_STA_OK || frame_m > max_distance()) {
            // a failed measurement means no usable echo (no target,
            // target too far or too close)
            count_out_of_range++;
            continue;
        }
        sum_m += frame_m;
        count++;
    }

    if (count > 0) {
        reading_m = sum_m / count;
        return true;
    }

    if (count_out_of_range > 0) {
        // report just beyond the configured maximum so that the
        // status becomes OutOfRangeHigh
        reading_m = max_distance() + 1.0f;
        return true;
    }

    return false;
}

#endif  // AP_RANGEFINDER_ERDI_E2_ENABLED
