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
*/

#include "SIM_config.h"

#if AP_SIM_RF_ERDI_E2_ENABLED

#include "SIM_RF_ERDI_E2.h"

using namespace SITL;

/*
  look for the 8-byte "continuous ranging" (0x89) and "stop
  ranging" (0x8E) commands. Host->module checksum is the low byte of
  the sum of bytes 2 to 6.
 */
void RF_ERDI_E2::process_input()
{
    char buf[32];
    const ssize_t n = read_from_autopilot(buf, sizeof(buf));
    for (ssize_t i = 0; i < n; i++) {
        const uint8_t c = buf[i];
        if (rxlen == 0 && c != 0x55) {
            continue;
        }
        if (rxlen == 1 && c != 0xAA) {
            rxlen = (c == 0x55) ? 1 : 0;
            continue;
        }
        rxbuf[rxlen++] = c;
        if (rxlen < sizeof(rxbuf)) {
            continue;
        }
        rxlen = 0;
        uint8_t sum = 0;
        for (uint8_t j = 2; j < 7; j++) {
            sum += rxbuf[j];
        }
        if (sum != rxbuf[7]) {
            continue;
        }
        switch (rxbuf[2]) {
        case 0x89:
            streaming = true;
            break;
        case 0x8E:
            streaming = false;
            break;
        default:
            break;
        }
    }
}

uint32_t RF_ERDI_E2::packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen)
{
    process_input();

    if (!streaming) {
        return 0;
    }

    const uint8_t PACKET_LEN = 8;
    if (buflen < PACKET_LEN) {
        abort();
    }

    // LR1200E2: 4m to 1200m in 0.1m steps. Outside that the module
    // reports a failed measurement (STA != 1).
    const bool ok = alt_m >= 4.0f && alt_m <= 1200.0f;
    const uint16_t raw = ok ? uint16_t(alt_m * 10) : 0;

    buffer[0] = 0x55;
    buffer[1] = 0xAA;
    buffer[2] = 0x89;
    buffer[3] = ok ? 0x01 : 0x00;
    buffer[4] = 0xFF;
    buffer[5] = raw >> 8;
    buffer[6] = raw & 0xFF;
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 7; i++) {
        sum += buffer[i];
    }
    buffer[7] = sum;

    return PACKET_LEN;
}

#endif  // AP_SIM_RF_ERDI_E2_ENABLED
