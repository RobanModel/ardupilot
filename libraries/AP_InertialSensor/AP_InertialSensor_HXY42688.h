/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_HXY42688 : public AP_InertialSensor_Backend
{
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;
    bool update() override;

    enum reg_hx42688 {
        WHO_AM_I     = 0x01,
        COM_CFG      = 0x05,
        LPF_CFG      = 0x08,
        SOFT_RST     = 0x4A,
        PWR_CTRL     = 0x7D,
        SEG_SEL      = 0x7F,
    };

private:
    AP_InertialSensor_HXY42688(AP_InertialSensor &imu,
                               AP_HAL::OwnPtr<AP_HAL::Device> dev,
                               enum Rotation rotation);

    /*
      initialise driver
     */
    bool init();

    void poll_data();

    bool block_read(uint8_t reg_addr, uint8_t *buf, uint32_t size);
    uint8_t read_register(uint8_t reg_addr);
    bool write_register(uint8_t reg_addr, uint8_t val, bool checked=false);

    void set_temperature(uint8_t instance, int16_t temper);

    bool _regs_check_out();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    uint8_t accel_instance;
    uint8_t gyro_instance;
    enum Rotation rotation;
    static const uint8_t COMM_REGS[][2];
};
