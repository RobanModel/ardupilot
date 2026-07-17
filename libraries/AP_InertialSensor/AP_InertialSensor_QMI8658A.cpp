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

#include "AP_InertialSensor_QMI8658A.h"

#define QMI8658A_BACKEND_SAMPLE_RATE       1000
#define QMI8658A_BACKEND_SAMPLE_MAX_RATE   2000

#define QMI8658A_ACCEL_G08              (0x02 << 4)
#define QMI8658A_ACCEL_G16              (0x03 << 4)
#define AMI8658A_ACCEL_SAMP_RATE_896HZ  (0x03)
#define QMI8658A_ACCEL_LPF              (0x03 << 1)
#define QMI8658A_ACCEL_LPF_EN           (0x01)

#define QMI8658A_GYRO_DPS_1024          (0x06 << 4)
#define QMI8658A_GYRO_DPS_2048          (0x07 << 4)
#define AMI8658A_GYRO_SAMP_RATE_896HZ   (0x03)
#define QMI8658A_GYRO_LPF               (0x03 << 5)
#define QMI8658A_GYRO_LPF_EN            (0x01 << 4)

extern const AP_HAL::HAL& hal;

const uint8_t AP_InertialSensor_QMI8658A::COMM_REGS[][2] = {
    {0x02, 0x60},
    {0x03, 0x33},
    {0x04, 0x63},
    {0x06, 0x55},
    {0x08, 0x03},
};

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
    return (int16_t)(((int16_t)msb << 8u) | lsb);
}

AP_InertialSensor_QMI8658A::AP_InertialSensor_QMI8658A(AP_InertialSensor &imu,
        AP_HAL::OwnPtr<AP_HAL::Device> dev,
        enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , rotation(_rotation)
{
}

AP_InertialSensor_Backend* AP_InertialSensor_QMI8658A::probe(AP_InertialSensor &imu,
        AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
        enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    auto sensor = new AP_InertialSensor_QMI8658A(imu, std::move(dev), rotation);

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

void AP_InertialSensor_QMI8658A::start()
{
    if (!_imu.register_accel(accel_instance, QMI8658A_BACKEND_SAMPLE_RATE, _dev->get_bus_id_devtype(DEVTYPE_INS_QMI8658A)) ||
        !_imu.register_gyro(gyro_instance,   QMI8658A_BACKEND_SAMPLE_RATE, _dev->get_bus_id_devtype(DEVTYPE_INS_QMI8658A))) {
        return;
    }

    // set backend rate
    uint16_t backend_rate_hz = QMI8658A_BACKEND_SAMPLE_RATE;
    if (enable_fast_sampling(accel_instance) && get_fast_sampling_rate() > 1) {
        bool fast_sampling = _dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI;
        if (fast_sampling) {
            // constrain the gyro rate to be a 2^N multiple
            uint8_t fast_sampling_rate = constrain_int16(get_fast_sampling_rate(), 1, 4);
            // calculate rate we will be giving samples to the backend
            backend_rate_hz = constrain_int16(backend_rate_hz * fast_sampling_rate, backend_rate_hz, QMI8658A_BACKEND_SAMPLE_MAX_RATE);
        }
    }
    uint32_t backend_period_us = 1000000UL / backend_rate_hz;

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    // setup callbacks
    _dev->register_periodic_callback(backend_period_us, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_QMI8658A::poll_data, void));
}

/*
  probe and initialise accelerometer
 */
bool AP_InertialSensor_QMI8658A::init()
{
    int retry = 5;

    WITH_SEMAPHORE(_dev->get_semaphore());

    // power on and wait 200ms
    hal.scheduler->delay(200);

    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    while (retry--) {
        // read ID and checked
        uint8_t id = read_register(WHO_AM_I);
        uint8_t version = read_register(REVISION_ID);
        if (!(id == 0x05 && version == 0x7C)) {
            return false;
        }

        // software reset and delay 100ms
        write_register(RESET, 0xB0);
        hal.scheduler->delay(200);

        // disable accel and gyro
        write_register(CTRL7, 0x00);
        hal.scheduler->delay(100);

        write_register(CTRL1, 0x60);

        // accel 16g and 896Hz
        write_register(CTRL2, QMI8658A_ACCEL_G16 | AMI8658A_ACCEL_SAMP_RATE_896HZ);
        // gyro 2048dps and 896Hz
        write_register(CTRL3, QMI8658A_GYRO_DPS_1024| AMI8658A_GYRO_SAMP_RATE_896HZ);
        // enable accel and gyro LPF filter
        write_register(CTRL5, 0x55);

        // enable accel and gyro
        write_register(CTRL7, 0x03);
        hal.scheduler->delay(500);

        _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
        if (regs_check_out()) {
            return true;
        }
    }

    // check ok
    return false;
}

bool AP_InertialSensor_QMI8658A::regs_check_out()
{
    for (int i = 0; i < sizeof(AP_InertialSensor_QMI8658A::COMM_REGS)/sizeof(AP_InertialSensor_QMI8658A::COMM_REGS[0]); ++i) {
        uint8_t reg = AP_InertialSensor_QMI8658A::COMM_REGS[i][0];
        uint8_t value = read_register(reg);
        if (value != AP_InertialSensor_QMI8658A::COMM_REGS[i][1]) {
            return false;
        }
    }
    return true;
}

void AP_InertialSensor_QMI8658A::poll_data()
{
    static unsigned int read_cnt = 0;

    uint8_t datas[6];
    uint8_t stat = 0;
    int16_t acc[3];
    int16_t gyro[3];

    stat = read_register(STATUS0);

    if ((stat & 0x01) == 0x01) {
        ++read_cnt;
        if (block_read(ACC_READ, datas, 6)) {
            acc[0] = combine(datas[1], datas[0]);
            acc[1] = combine(datas[3], datas[2]);
            acc[2] = combine(datas[5], datas[4]);

            Vector3f accelVector(acc[0], acc[1], acc[2]);
            accelVector *= GRAVITY_MSS * (1.0f / 2048.0f);

            _rotate_and_correct_accel(accel_instance, accelVector);
            _notify_new_accel_raw_sample(accel_instance, accelVector);
        }
    }

    if ((stat & 0x02) == 0x02) {
        if (block_read(GYRO_READ, datas, 6)) {
            gyro[0] = combine(datas[1], datas[0]);
            gyro[1] = combine(datas[3], datas[2]);
            gyro[2] = combine(datas[5], datas[4]);

            Vector3f gyroVector(gyro[0], gyro[1], gyro[2]);
            gyroVector *= radians(1.0f / 32.0f);

            _rotate_and_correct_gyro(gyro_instance, gyroVector);
            _notify_new_gyro_raw_sample(gyro_instance, gyroVector);
        }
    }

    if (read_cnt >= 100 && block_read(TEMP_READ, datas, 2)) {
        read_cnt = 0;
        int16_t temp = combine(datas[1], datas[0]);
        set_temperature(accel_instance, temp);
    }
}

void AP_InertialSensor_QMI8658A::set_temperature(uint8_t instance, int16_t temper)
{
    const float temp_degc = temper * 0.00390625; // temper/256
    _publish_temperature(instance, temp_degc);
}

bool AP_InertialSensor_QMI8658A::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

bool AP_InertialSensor_QMI8658A::block_read(uint8_t reg_addr, uint8_t *buf, uint32_t size)
{
    reg_addr |= 0x80;

    return _dev->transfer(&reg_addr, 1, buf, size);
}

uint8_t AP_InertialSensor_QMI8658A::read_register(uint8_t reg_addr)
{
    uint8_t val = 0;

    reg_addr |= 0x80;

    _dev->transfer(&reg_addr, 1, &val, 1);

    return val;
}

bool AP_InertialSensor_QMI8658A::write_register(uint8_t reg_addr, uint8_t val, bool checked)
{
    uint8_t buf[2] = { reg_addr, val };
    if (checked) {
        _dev->set_checked_register(reg_addr, val);
    }
    bool result = _dev->transfer(buf, sizeof(buf), nullptr, 0);

    hal.scheduler->delay(5);

    return result;
}
