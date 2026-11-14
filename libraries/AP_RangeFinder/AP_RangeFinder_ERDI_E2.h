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

/*
  Driver for ERDI TECH 905nm long range laser rangefinders with the
  8-byte "55 AA" UART command/response protocol:

    E2 series: LR1000E2, LR1200E2, LR1500E2, LR2000E2

  Default 115200 8N1. The module only measures after it has been
  sent a command; this driver sends "continuous ranging" and the
  module then replies at 2-10Hz (adaptive):

    host -> module:  55 AA 89 FF FF FF FF 85
    module -> host:  55 AA 89 STA FF DIS_H DIS_L SUM

  STA = 1 means the measurement succeeded. Distance is big-endian in
  units of 0.1m. SUM is the low byte of the sum of the first 7 bytes
  (note the host->module checksum excludes the 55 AA header).

  The SW-SHOT (enable) pin on the module connector must be held high.
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_ERDI_E2_ENABLED

#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_ERDI_E2 : public AP_RangeFinder_Backend_Serial
{
public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params)
    {
        return NEW_NOTHROW AP_RangeFinder_ERDI_E2(_state, _params);
    }

protected:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    bool get_reading(float &reading_m) override;

    // factory default baud rate of the E2 series
    uint32_t initial_baudrate(uint8_t serial_instance) const override { return 115200; }

    // the module adapts its rate down to 2Hz at long range
    uint16_t read_timeout_ms() const override { return 1000; }

    // ask the module to start continuous ranging
    void send_start_command();

    // shift the line buffer to the next frame header after a bad frame
    void resync();

    uint8_t linebuf[8];
    uint8_t linebuf_len;
    uint32_t last_frame_ms;
    uint32_t last_start_cmd_ms;
    bool start_cmd_sent;
};

#endif  // AP_RANGEFINDER_ERDI_E2_ENABLED
