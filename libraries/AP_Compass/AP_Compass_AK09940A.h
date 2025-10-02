#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_AK09940A_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

class AuxiliaryBus;
class AuxiliaryBusSlave;
class AP_InertialSensor;
class AP_AK09940A_BusDriver;

class AP_Compass_AK09940A : public AP_Compass_Backend
{
public:
    /* Probe for AK09940A standalone on I2C bus */
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                     enum Rotation rotation);

    static constexpr const char *name = "AK09940A";

    virtual ~AP_Compass_AK09940A();

    void read() override;

private:
    AP_Compass_AK09940A(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                      enum Rotation rotation);

    bool init();
    void _make_factory_sensitivity_adjustment(Vector3f &field) const;

    bool _reset();
    bool _setup_mode();
    bool _check_id();
    bool _calibrate();

    void _update();

    float _magnetometer_ASA[3] {0, 0, 0};

    uint8_t _compass_instance;
    bool _initialized;
    enum Rotation _rotation;
};

class AP_AK09940A_BusDriver
{
public:
    virtual ~AP_AK09940A_BusDriver() { }

    virtual bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) = 0;
    virtual bool register_read(uint8_t reg, uint8_t *val) = 0;
    virtual bool register_write(uint8_t reg, uint8_t val) = 0;

    virtual AP_HAL::Semaphore  *get_semaphore() = 0;

    virtual bool configure() { return true; }
    virtual bool start_measurements() { return true; }
    virtual AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t, AP_HAL::Device::PeriodicCb) = 0;

    // set device type within a device class
    virtual void set_device_type(uint8_t devtype) = 0;

    // return 24 bit bus identifier
    virtual uint32_t get_bus_id(void) const = 0;
};

   
#endif  // AP_COMPASS_AK09940A_ENABLED
