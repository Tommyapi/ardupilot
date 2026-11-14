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
  Simulator for ERDI TECH E2 series serial rangefinders
  (LR1000E2 / LR1200E2 / LR1500E2 / LR2000E2, 8-byte "55 AA" protocol)

  The simulated module stays silent until it receives the
  "continuous ranging" command, like the real module.

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:erdi_e2 --speedup=1

param set SERIAL5_PROTOCOL 9
param set SERIAL5_BAUD 115
param set RNGFND1_TYPE 51
param set RNGFND1_MIN 4
param set RNGFND1_MAX 1200
reboot

graph RANGEFINDER.distance
graph GLOBAL_POSITION_INT.relative_alt/1000-RANGEFINDER.distance

arm throttle
rc 3 1600
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_RF_ERDI_E2_ENABLED

#include "SIM_SerialRangeFinder.h"

namespace SITL {

class RF_ERDI_E2 : public SerialRangeFinder {
public:

    static SerialRangeFinder *create() { return NEW_NOTHROW RF_ERDI_E2(); }

    uint32_t packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen) override;

    // the real module adapts between 2Hz and 10Hz
    uint16_t reading_interval_ms() const override { return 100; }

private:

    // handle commands from the autopilot
    void process_input();

    bool streaming;
    uint8_t rxbuf[8];
    uint8_t rxlen;
};

}

#endif  // AP_SIM_RF_ERDI_E2_ENABLED
