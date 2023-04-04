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
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>

// #include <AP_HAL/Util.h>
// #include <AP_Logger/AP_Logger.h>
// #include <stdio.h>

#include "AP_InertialSensor_IIM4623x.h"

/*
  device registers
 */
#define REG_PROD_ID         0x00
#define  PROD_ID_46234      0xEA
#define  PROD_ID_46230      0xE6

/*
  timings
 */
#define T_STALL_US   20U
#define T_RESET_MS   500U

#define TIMING_DEBUG 0
#if TIMING_DEBUG
#define DEBUG_SET_PIN(n,v) hal.gpio->write(52+n, v)
#define DEBUG_TOGGLE_PIN(n) hal.gpio->toggle(52+n)
#else
#define DEBUG_SET_PIN(n,v)
#define DEBUG_TOGGLE_PIN(n)
#endif

union iim4623x_serialnumber {
        uint16_t hdr;
        uint8_t length;
        uint8_t type;
        uint8_t pkt_number[2];
        uint8_t serial_number[16];
        uint16_t  checksum;
        uint16_t  footer;
    } sn_resp;

struct iim4623x_data {
        uint16_t hdr;
        uint8_t length;
        uint8_t type;
        uint8_t status;
        uint8_t sample_ctr;
        uint8_t timestamp[8];
        uint32_t  ax;
        uint32_t  ay;
        uint32_t  az;
        uint32_t  gx;
        uint32_t  gy;
        uint32_t  gz;
        uint32_t  temp;
        // uint32_t  dvx;
        // uint32_t  dvy;
        // uint32_t  dvz;
        // uint32_t  dax;
        // uint32_t  day;
        // uint32_t  daz;
        uint16_t  checksum;
        uint16_t  footer;
    };

extern const AP_HAL::HAL& hal;

const uint8_t cmd_get_serial_number[20]= {0x24, 0x24,
                                        0x08,
                                        0x26,
                                        0x00, 0x26,
                                        0x0D, 0x0A,
                                        0x00, 0x00, 0x00, 0x00, 
                                        0x00, 0x00, 0x00, 0x00, 
                                        0x00, 0x00, 0x00, 0x00}; 

AP_InertialSensor_IIM4623x::AP_InertialSensor_IIM4623x(AP_InertialSensor &imu,
                                                         AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                                         enum Rotation _rotation,
                                                         uint8_t drdy_gpio)
    : AP_InertialSensor_Backend(imu)
    , dev(std::move(_dev))
    , rotation(_rotation)
    , drdy_pin(drdy_gpio)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_IIM4623x::probe(AP_InertialSensor &imu,
                                   AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                   enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_InertialSensor_IIM4623x *sensor = new AP_InertialSensor_IIM4623x(imu, std::move(dev), rotation,60);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_IIM4623x::start()
{
    if (!_imu.register_accel(accel_instance, expected_sample_rate_hz, dev->get_bus_id_devtype(DEVTYPE_INS_IIM4623X)) ||
        !_imu.register_gyro(gyro_instance, expected_sample_rate_hz,   dev->get_bus_id_devtype(DEVTYPE_INS_IIM4623X))) {
        return;
    }

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    /*
      as the sensor does not have a FIFO we need to jump through some
      hoops to ensure we don't lose any samples. This creates a thread
      to do the capture, running at very high priority
     */
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_IIM4623x::loop, void),
                                      "IIM4623X",
                                      1024, AP_HAL::Scheduler::PRIORITY_BOOST, 1)) {
        AP_HAL::panic("Failed to create IIM4623X thread");
    }
}

/*
  check product ID
 */
bool AP_InertialSensor_IIM4623x::check_product_id(uint32_t &prod_id)
{
    read_word(REG_PROD_ID, prod_id);
    switch (prod_id) {
    case PROD_ID_46234:
        // can do up to 40G
        // accel_scale = 1.25 * GRAVITY_MSS * 0.001;
        _clip_limit = 31.5f * GRAVITY_MSS;
        gyro_scale = radians(0.1);
        expected_sample_rate_hz = 4000;
        return true;

    case PROD_ID_46230: {
        // can do up to 40G
        accel_scale = 1.25 * GRAVITY_MSS * 0.001;
        _clip_limit = 39.5f * GRAVITY_MSS;
        expected_sample_rate_hz = 2000;
        return true;

    }
    }
    return false;
}

uint16_t AP_InertialSensor_IIM4623x::calc_checksum(uint8_t *buff, uint32_t length)
{
	uint16_t sum = 0;
	for (uint32_t i = 0; i < length; i++) sum += (uint16_t)buff[i];
	return sum;
}

void AP_InertialSensor_IIM4623x::read_sensor()
{
    iim4623x_data data;
    // uint8_t zeros [46]= {0};

    //read once --pending verification
    if(hal.gpio->read(60)){
        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IIM_4623x: DRDY detected. ");
    }
    WITH_SEMAPHORE(dev->get_semaphore());
    dev->transfer((const uint8_t *)&data, sizeof(data), (uint8_t *)&data, sizeof(data));
    hal.scheduler->delay(1);
    // calc_checksum(&data[3],sizeof(data)-7);

    Vector3f accel{float((8*be32toh(data.ax)*GRAVITY_MSS)/pow(2,31)),
                  -float((8*be32toh(data.ay)*GRAVITY_MSS)/pow(2,31)),
                  -float((8*be32toh(data.az)*GRAVITY_MSS)/pow(2,31))};
    Vector3f gyro{float(500*be32toh(data.gx)/pow(2,31)),
                    -float(500*be32toh(data.gy)/pow(2,31)),
                    -float(500*be32toh(data.gz)/pow(2,31))};

    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel);

    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro);

    
    // Vector3f dvel{float(dvel_scale*int32_t(be16toh(data.dvx_low) | (be16toh(data.dvx_high)<<16))),
    //               -float(dvel_scale*int32_t(be16toh(data.dvy_low) | (be16toh(data.dvy_high)<<16))),
    //               -float(dvel_scale*int32_t(be16toh(data.dvz_low) | (be16toh(data.dvz_high)<<16)))};
    // Vector3f dangle{float(dangle_scale*int32_t(be16toh(data.dax_low) | (be16toh(data.dax_high)<<16))),
    //                 -float(dangle_scale*int32_t(be16toh(data.day_low) | (be16toh(data.day_high)<<16))),
    //                 -float(dangle_scale*int32_t(be16toh(data.daz_low) | (be16toh(data.daz_high)<<16)))};

    // // compensate for clock errors, see "DELTA ANGLES" in datasheet
    // dangle *= expected_sample_rate_hz / _gyro_raw_sample_rate(gyro_instance);
    // dvel *= expected_sample_rate_hz / _accel_raw_sample_rate(gyro_instance);

    // _notify_new_delta_velocity(accel_instance, dvel);
    // _notify_new_delta_angle(gyro_instance, dangle);

    /*
      publish average temperature at 20Hz
     */
    temp_sum += float(be32toh(data.temp)/126.8 +25);
    temp_count++;

    if (temp_count == 100) {
        _publish_temperature(accel_instance, temp_sum/temp_count);
        temp_sum = 0;
        temp_count = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IIM_4623x: Received %d \n", data.sample_ctr);
    }
}

bool AP_InertialSensor_IIM4623x::init()
{
    // hal.gpio->pinMode(drdy_pin, HAL_GPIO_INPUT);
    // hal.gpio->attach_interrupt(drdy_pin, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_IIM4623x::cb_reader, void, uint8_t, bool, uint32_t),AP_HAL::GPIO::INTERRUPT_RISING);
    _clip_limit = 7.5f * GRAVITY_MSS;
    gyro_scale = radians(0.1);
    expected_sample_rate_hz = 20;



    WITH_SEMAPHORE(dev->get_semaphore());
    uint8_t zeros[sizeof(sn_resp)];
    dev->transfer(cmd_get_serial_number,sizeof(cmd_get_serial_number),nullptr, 0);
    hal.scheduler->delay(2);
    //the expected sn packet is 0x1A in length
    // uint8_t response[sizeof(iim4623x_serialnumber)];
    // dev->transfer(response, sizeof(response),(uint8_t *)&response, sizeof(response));
    dev->transfer(zeros, sizeof(sn_resp),(uint8_t *)&sn_resp, sizeof(sn_resp));

    // sn_resp = (iim4623x_serialnumber) response;
    // dev->set_speed(AP_HAL::Device::SPEED_LOW);
    // uint8_t set_fixed_point[20] = {0x24,0x24,0x0D,0x12,0x00,0x01,0x19,0x00,0x01,0x00,0x2D,0x0D,0x0A,0x00,0,0,0,	0,	0,	0};
    // uint8_t zeros[20] = {0};
    
    // dev->transfer(set_fixed_point,sizeof(set_fixed_point),nullptr,0);
    // hal.scheduler->delay(2);
    // dev->transfer((uint8_t *)&zeros,20,(uint8_t *)&zeros,20);
    // hal.scheduler->delay(2);

    // uint8_t start_streaming[20] = {0x24,0x24,0x08,0x27,0,0x27,0x0D,0x0A,0,0,0,0,0,0,0,0,0,0,0,0};
    // dev->transfer(start_streaming,sizeof(start_streaming),start_streaming,sizeof(start_streaming));
    // hal.scheduler->delay(2);

    // uint8_t tries = 10;
    // uint32_t prod_id = 0;
    // do {
    //     hal.scheduler->delay(5);
    // } while (!check_product_id(prod_id) && --tries);
    // if (tries == 0) {
        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IIM_4623x: failed to fetch pid after %d", 10-tries);
    //     return false;
    // }
    

    // we need to use low speed for burst transfers
    

    return true;
}

// functions for 32 bit registers
bool AP_InertialSensor_IIM4623x::write_word(const uint8_t reg, const uint32_t data) const
{
    const uint8_t b[5] { reg,  uint8_t (data&0xff), uint8_t(data >> 8), uint8_t(data >> 16), uint8_t(data >> 24) };
    return dev->transfer(b, sizeof(b), nullptr, 0);
}

bool AP_InertialSensor_IIM4623x::read_word(const uint8_t reg, uint32_t& data) const
{
    return dev->read_registers(reg, (uint8_t *)&data, sizeof(data));
}

/*
  sensor read loop
 */
void AP_InertialSensor_IIM4623x::loop(void)
{
    while (true) {
        uint32_t tstart = AP_HAL::micros();
        // we deliberately set the period a bit fast to ensure we
        // don't lose a sample
        const uint32_t period_us = (1000000UL / expected_sample_rate_hz) - 20U;
        bool wait_ok = false;
        // uint8_t drdy_state = hal.gpio->read(60);
        // if (drdy_pin != 0) {
        //     // when we have a DRDY pin then wait for it to go high
        //     // DEBUG_SET_PIN(0, 1);
        //     wait_ok = hal.gpio->wait_pin(drdy_pin, AP_HAL::GPIO::INTERRUPT_RISING, 2100);
        //     // DEBUG_SET_PIN(0, 0);
        // }
        // if (opmode == OpMode::Delta32) {
        //     read_sensor32_delta();
        // } else if (opmode == OpMode::AG32) {
        //     read_sensor32();
        // } else {
        //     read_sensor16();
        // }
        //if (drdy_state){
            // read_sensor();
        //}
        WITH_SEMAPHORE(dev->get_semaphore());
        uint8_t zeros[sizeof(sn_resp)];
        dev->transfer(cmd_get_serial_number,sizeof(cmd_get_serial_number),nullptr, 0);
        hal.scheduler->delay(2);
        //the expected sn packet is 0x1A in length
        // uint8_t response[sizeof(iim4623x_serialnumber)];
        // dev->transfer(response, sizeof(response),(uint8_t *)&response, sizeof(response));
        dev->transfer(zeros, sizeof(sn_resp),(uint8_t *)&sn_resp, sizeof(sn_resp));
        hal.scheduler->delay(2);    
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IIM_4623x: Received %x pkts, SN %x \n", sn_resp.length,sn_resp.type);
        
        uint32_t dt = AP_HAL::micros() - tstart;
        if (dt < period_us) {
            uint32_t wait_us = period_us - dt;
            if (!wait_ok || wait_us > period_us/2) {
                hal.scheduler->delay_microseconds(wait_us);
            }
        }
    }
}

bool AP_InertialSensor_IIM4623x::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

// bool AP_InertialSensor_IIM4623x::write_register(uint8_t reg, uint8_t val, bool checked)
// {
//     uint8_t buf[2] = { reg, val };
//     if (checked) {
//         set_checked_register(reg, val);
//     }
//     bool result = transfer(buf, sizeof(buf), nullptr, 0);
//     if (AP_HAL::Device _register_rw_callback && result) {
//         _register_rw_callback(reg, &val, 1, true);
//     }
//     return result;
// }
