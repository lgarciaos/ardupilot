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

#include "AP_Compass_AK09940A.h"
#include <cstdint>

#if AP_COMPASS_AK09940A_ENABLED

#include <assert.h>
#include <utility>

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

#define AK09940A_I2C_ADDR                               0x0C

#define AK09940A_WIA1                                   0x00
#       define      AK09940A_MFG_ID                     0x48
#define AK09940A_WIA2                                   0x00
#       define      AK09940A_DEV_ID                     0xA3

#define AK09940_ST                                      0x0F
#   define      AK09940A_ST_DOR                         0x2
#   define      AK09940A_ST_DRDY                        0x1

#define AK09940_ST1                                     0x10

#define AK09940A_HXL                                    0x11

/* bit definitions for AK09940A CNTL1 */
#define AK09940A_CNTL1                                  0x0A
#       define    AK09940A_DTSET                        0x20 // Set for external trigger

#define AK09940A_CNTL3                                  0x32
#        define    AK09940A_POWERDOWN_MODE              0x00
#        define    AK09940A_CONTINUOUS_MODE1            0x02 // 10 Hz
#        define    AK09940A_CONTINUOUS_MODE2            0x04 // 20 Hz
#        define    AK09940A_CONTINUOUS_MODE3            0x06 // 50 Hz
#        define    AK09940A_CONTINUOUS_MODE4            0x08 // 100 Hz
#        define    AK09940A_CONTINUOUS_MODE5            0x0A // 200 Hz
#        define    AK09940A_CONTINUOUS_MODE6            0x0C // 400 Hz
#        define    AK09940A_CONTINUOUS_MODE7            0x0E // 1 kHz
#        define    AK09940A_CONTINUOUS_MODE8            0x0F // 2.5 kHz
#        define    AK09940A_SELFTEST_MODE               0x10 
#        define    AK09940A_SENSORDRIVE_LP1             (0x00)
#        define    AK09940A_SENSORDRIVE_LP2             (0x01 << 5)
#        define    AK09940A_SENSORDRIVE_LN1             (0x10 << 5)
#        define    AK09940A_SENSORDRIVE_LN2             (0x11 << 5)
#        define    AK09940A_FIFO                        0x80

#define AK09940A_CNTL4                                  0x33
#        define AK09940A_RESET                          0x01

#define AK09940A_MILLIGAUSS_SCALE                       10.0f

extern const AP_HAL::HAL &hal;

AP_Compass_AK09940A::AP_Compass_AK09940A(AP_AK09940A_BusDriver *bus,
                                     enum Rotation rotation)
    : _bus(bus)
    , _rotation(rotation)
{
}

AP_Compass_AK09940A::~AP_Compass_AK09940A()
{
    delete _bus;
}

AP_Compass_Backend *AP_Compass_AK09940A::probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                             enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_AK09940A_BusDriver *bus = NEW_NOTHROW AP_AK09940A_BusDriver_HALDevice(std::move(dev));
    if (!bus) {
        return nullptr;
    }

    AP_Compass_AK09940A *sensor = NEW_NOTHROW AP_Compass_AK09940A(bus, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_Compass_AK09940A::init()
{
    AP_HAL::Semaphore *bus_sem = _bus->get_semaphore();

    if (!bus_sem) {
        return false;
    }
    _bus->get_semaphore()->take_blocking();

    if (!_bus->configure()) {
        DEV_PRINTF("AK09940A: Could not configure the bus\n");
        goto fail;
    }

    if (!_check_id()) {
        DEV_PRINTF("AK09940A: Wrong id\n");
        goto fail;
    }

    if (!_calibrate()) {
        DEV_PRINTF("AK09940A: Could not read calibration data\n");
        goto fail;
    }

    if (!_setup_mode()) {
        DEV_PRINTF("AK09940A: Could not setup mode\n");
        goto fail;
    }

    if (!_bus->start_measurements()) {
        DEV_PRINTF("AK09940A: Could not start measurements\n");
        goto fail;
    }

    _initialized = true;

    /* register the compass instance in the frontend */
    _bus->set_device_type(DEVTYPE_AK09940A);
    if (!register_compass(_bus->get_bus_id(), _compass_instance)) {
        goto fail;
    }
    set_dev_id(_compass_instance, _bus->get_bus_id());

    set_rotation(_compass_instance, _rotation);
    bus_sem->give();

    _bus->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_Compass_AK09940A::_update, void));

    return true;

fail:
    bus_sem->give();
    return false;
}

void AP_Compass_AK09940A::read()
{
    if (!_initialized) {
        return;
    }

    drain_accumulated_samples(_compass_instance);
}

void AP_Compass_AK09940A::_make_adc_sensitivity_adjustment(Vector3f& field) const
{
    static const float ADC_16BIT_RESOLUTION = 0.15f;

    field *= ADC_16BIT_RESOLUTION;
}

void AP_Compass_AK09940A::_make_factory_sensitivity_adjustment(Vector3f& field) const
{
    field.x *= _magnetometer_ASA[0];
    field.y *= _magnetometer_ASA[1];
    field.z *= _magnetometer_ASA[2];
}

void AP_Compass_AK09940A::_update()
{
    struct sample_regs regs;
    Vector3f raw_field;

    if (!_bus->block_read(AK09940A_HXL, (uint8_t *) &regs, sizeof(regs))) {
        return;
    }

    /* Check for overflow. See AK09940A's datasheet, section
     * 6.4.3.6 - Magnetic Sensor Overflow. */
    if ((regs.st2 & 0x08)) {
        return;
    }

    raw_field = Vector3f(regs.val[0], regs.val[1], regs.val[2]);

    if (is_zero(raw_field.x) && is_zero(raw_field.y) && is_zero(raw_field.z)) {
        return;
    }

    _make_factory_sensitivity_adjustment(raw_field);
    _make_adc_sensitivity_adjustment(raw_field);
    raw_field *= AK09940A_MILLIGAUSS_SCALE;

    accumulate_sample(raw_field, _compass_instance, 10);
}

bool AP_Compass_AK09940A::_check_id()
{
    for (int i = 0; i < 5; i++) {
        uint8_t deviceid = 0;

        /* Read AK09940A's id */
        if (_bus->register_read(AK09940A_WIA, &deviceid) &&
            deviceid == AK09940A_Device_ID) {
            return true;
        }
    }

    return false;
}

bool AP_Compass_AK09940A::_setup_mode(uint8_t mode) 
{
    return _bus->register_write(AK09940A_CNTL2, mode);
}

bool AP_Compass_AK09940A::_reset()
{
    return _bus->register_write(AK09940A_CNTL4, AK09940A_RESET);
}


bool AP_Compass_AK09940A::_calibrate()
{
    /* Enable FUSE-mode in order to be able to read calibration data */
    _bus->register_write(AK09940A_CNTL1, AK09940A_FUSE_MODE | AK09940A_16BIT_ADC);

    uint8_t response[3];

    _bus->block_read(AK09940A_ASAX, response, 3);

    for (int i = 0; i < 3; i++) {
        float data = response[i];
        _magnetometer_ASA[i] = ((data - 128) / 256 + 1);
    }

    return true;
}

/* AP_HAL::I2CDevice implementation of the AK09940A */
AP_AK09940A_BusDriver_HALDevice::AP_AK09940A_BusDriver_HALDevice(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : _dev(std::move(dev))
{
}

bool AP_AK09940A_BusDriver_HALDevice::block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    return _dev->read_registers(reg, buf, size);
}

bool AP_AK09940A_BusDriver_HALDevice::register_read(uint8_t reg, uint8_t *val)
{
    return _dev->read_registers(reg, val, 1);
}

bool AP_AK09940A_BusDriver_HALDevice::register_write(uint8_t reg, uint8_t val)
{
    return _dev->write_register(reg, val);
}

AP_HAL::Semaphore *AP_AK09940A_BusDriver_HALDevice::get_semaphore()
{
    return _dev->get_semaphore();
}

AP_HAL::Device::PeriodicHandle AP_AK09940A_BusDriver_HALDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return _dev->register_periodic_callback(period_usec, cb);
}

#endif  // AP_COMPASS_AK09940A_ENABLED
