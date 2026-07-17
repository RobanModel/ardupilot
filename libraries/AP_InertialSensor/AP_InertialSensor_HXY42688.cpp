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

#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_InertialSensor_HXY42688.h"

#define HXY42688_BACKEND_SAMPLE_RATE       800

extern const AP_HAL::HAL& hal;

const uint8_t AP_InertialSensor_HXY42688::COMM_REGS[][2] = {
    {0x41, 0x03}, //ACC_RANGE  ±16G
    {0x40, 0x0B}, //ACC_CONF 0x0B=800Hz
    {0x43, 0x00}, //GYR_RANGE 2000dps
    {0x42, 0xEB}, //GYR_CONF 0x08=800Hz
    {0x05, 0x50}, //COM_CFG
    {0x08, 0x00}  //Disable DLPF
};

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
    return (int16_t)(((int16_t)msb << 8u) | lsb);
}

AP_InertialSensor_HXY42688::AP_InertialSensor_HXY42688(AP_InertialSensor &imu,
        AP_HAL::OwnPtr<AP_HAL::Device> dev,
        enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , rotation(_rotation)
{
}

AP_InertialSensor_Backend* AP_InertialSensor_HXY42688::probe(AP_InertialSensor &imu,
        AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
        enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    auto sensor = new AP_InertialSensor_HXY42688(imu, std::move(dev), rotation);

    if (!sensor) {
        return nullptr;
    }

    if (dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        dev->set_read_flag(0x80);
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_HXY42688::start()
{
    if (!_imu.register_accel(accel_instance, HXY42688_BACKEND_SAMPLE_RATE, _dev->get_bus_id_devtype(DEVTYPE_INS_HXY42688)) ||
        !_imu.register_gyro(gyro_instance,   HXY42688_BACKEND_SAMPLE_RATE, _dev->get_bus_id_devtype(DEVTYPE_INS_HXY42688))) {
        return;
    }

    // set backend rate
    uint16_t backend_rate_hz = HXY42688_BACKEND_SAMPLE_RATE;
    if (enable_fast_sampling(accel_instance) && get_fast_sampling_rate() > 1) {
        bool fast_sampling = _dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI;
        if (fast_sampling) {
            // constrain the gyro rate to be a 2^N multiple
            uint8_t fast_sampling_rate = constrain_int16(get_fast_sampling_rate(), 1, 4);
            // calculate rate we will be giving samples to the backend
            backend_rate_hz = constrain_int16(backend_rate_hz * fast_sampling_rate, backend_rate_hz, 2000);
        }
    }
    uint32_t backend_period_us = 1000000UL / backend_rate_hz;

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    // setup callbacks
    _dev->register_periodic_callback(backend_period_us, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_HXY42688::poll_data, void));
}

/*
  probe and initialise accelerometer
 */
bool AP_InertialSensor_HXY42688::init()
{
    uint8_t reg_value = 0;

    // power on and wait 50ms
    hal.scheduler->delay(100);

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    write_register(0x7F, 0x00);
    hal.scheduler->delay(2);

    // read ID and checked
    uint8_t id = read_register(0x01);
    if (id != 0x6A) {
        return false;
    }

    //POWER DOWN
    write_register(0x7D, 0x00);
    hal.scheduler->delay(200);
    reg_value = read_register(0x7D);
    if (reg_value != 0x00) {
        return false;
    }

    //SOFT_RESET
    write_register(0x7F, 0x00);
    hal.scheduler->delay(20);
    write_register(0x08, 0x87);//FLAG
    hal.scheduler->delay(10);
    write_register(0x05, 0x80);//BOOT
    write_register(0x4A, 0xA5);
    hal.scheduler->delay(200);

    // Select SPI
    write_register(0x7F, 0x00);//goto 0x90
    hal.scheduler->delay(5);
    write_register(0x4A, 0x66);
    hal.scheduler->delay(5);
    write_register(0x7F, 0x83);//goto 0x6F
    hal.scheduler->delay(5);
    write_register(0x6F, 0x04);//I2C disable
    write_register(0x7F, 0x00);//goto 0x6F
    hal.scheduler->delay(5);
    write_register(0x4A, 0x00);
    hal.scheduler->delay(5);


    for (int i = 0; i < 3; ++i) {
        // Power Off
        write_register(0x7F, 0x00);//goto 0x6F
        hal.scheduler->delay(10);
        write_register(0x7D, 0x0E);//PWR_CTRL ENABLE ACC+GYR+TEMP
        hal.scheduler->delay(200);

        write_register(0x41, 0x03); //ACC_RANGE  ±16G
        write_register(0x40, 0x80); //ACC_CONF 0x0B=800Hz
        write_register(0x40, 0x80); //ACC_CONF 0x0B=800Hz
        write_register(0x40, 0x8B); //ACC_CONF 0x0B=800Hz
        write_register(0x40, 0x8B); //ACC_CONF 0x0B=800Hz

        write_register(0x43, 0x00); //GYR_RANGE 2000dps
        write_register(0x42, 0x80); //GYR_CONF 0x08=800Hz
        write_register(0x42, 0x80); //GYR_CONF 0x08=800Hz
        write_register(0x42, 0xEB); //GYR_CONF 0x08=800Hz
        write_register(0x42, 0xEB); //GYR_CONF 0x08=800Hz

        write_register(0x05, 0x50); //COM_CFG
        write_register(0x08, 0x00); //Disable DLPF

        write_register(0x7D, 0x0E);//PWR_CTRL ENABLE ACC+GYR+TEMP
        hal.scheduler->delay(200);

        _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

        if (_regs_check_out()) {
            return true;
        }
    }

    // check ok
    return false;
}

bool AP_InertialSensor_HXY42688::_regs_check_out()
{
    write_register(0x7F, 0x00);//goto 0x00
    hal.scheduler->delay(1);

    for (int i = 0; i < sizeof(AP_InertialSensor_HXY42688::COMM_REGS)/sizeof(AP_InertialSensor_HXY42688::COMM_REGS[0]); ++i) {
        uint8_t reg = AP_InertialSensor_HXY42688::COMM_REGS[i][0];
        uint8_t value = read_register(reg);
        if (reg == 0x40 || reg == 0x42) {
            if ((value & 0x0B) == 0x0B) {
                continue;
            } else {
                return false;
            }
        }
        if (value != AP_InertialSensor_HXY42688::COMM_REGS[i][1]) {
            return false;
        }
    }
    return true;
}

void AP_InertialSensor_HXY42688::poll_data()
{
    uint8_t datas[6];
    uint8_t stat = 0;
    int16_t acc[3];
    int16_t gyro[3];

    stat = read_register(0x0B);

    if (stat & 0x30) {
        return;
    }

    if ((stat & 0x01) == 0x01) {
        if (block_read(0x0C, datas, 6)) {
            acc[0] = combine(datas[0], datas[1]);
            acc[1] = combine(datas[2], datas[3]);
            acc[2] = combine(datas[4], datas[5]);

            Vector3f accelVector(acc[0], acc[1], acc[2]);
            accelVector *= GRAVITY_MSS * (32.0f / 65536.0f);

            _rotate_and_correct_accel(accel_instance, accelVector);
            _notify_new_accel_raw_sample(accel_instance, accelVector);
        }
    }

    if ((stat & 0x02) == 0x02) {
        if (block_read(0x12, datas, 6)) {
            gyro[0] = combine(datas[0], datas[1]);
            gyro[1] = combine(datas[2], datas[3]);
            gyro[2] = combine(datas[4], datas[5]);

            Vector3f gyroVector(gyro[0], gyro[1], gyro[2]);
            gyroVector *= radians(4000.0f / 65536.0f);

            _rotate_and_correct_gyro(gyro_instance, gyroVector);
            _notify_new_gyro_raw_sample(gyro_instance, gyroVector);
        }
    }

    if ((stat & 0x04) == 0x04) {
        if (block_read(0x22, datas, 2)) {
            int16_t temp = combine(datas[0], datas[1]) / 512 + 23;
            set_temperature(accel_instance, temp);
        }
    }
}

void AP_InertialSensor_HXY42688::set_temperature(uint8_t instance, int16_t temper)
{
    const float temp_degc = temper * 0.00390625; // temper/256
    _publish_temperature(instance, temp_degc);
}

bool AP_InertialSensor_HXY42688::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

bool AP_InertialSensor_HXY42688::block_read(uint8_t reg_addr, uint8_t *buf, uint32_t size)
{
    reg_addr |= 0x80;

    return _dev->transfer(&reg_addr, 1, buf, size);
}

uint8_t AP_InertialSensor_HXY42688::read_register(uint8_t reg_addr)
{
    uint8_t val = 0;

    reg_addr |= 0x80;

    _dev->transfer(&reg_addr, 1, &val, 1);

    return val;
}

bool AP_InertialSensor_HXY42688::write_register(uint8_t reg_addr, uint8_t val, bool checked)
{
    uint8_t buf[2] = { reg_addr, val };
    if (checked) {
        _dev->set_checked_register(reg_addr, val);
    }
    bool result = _dev->transfer(buf, sizeof(buf), nullptr, 0);

    hal.scheduler->delay(5);

    return result;
}
