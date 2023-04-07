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



#define BYTE_HEADER_CMD	0x24
#define BYTE_HEADER_REP	0x23
#define BYTE_RESERVED	0x00
#define BYTE_FOOTER_1	0x0D
#define BYTE_FOOTER_2	0x0A
#define BYTE_PADDING	0x00

#define CMD_TYPE_SWITCH_TO_BOOTLOADER  0xF4
#define CMD_TYPE_UPGRADE_FIRMWARE      0xF5
#define CMD_TYPE_CLEAR_UPGRADE_FLAG    0xF6
#define CMD_TYPE_LENGTH_INFO           0xDA
#define CMD_TYPE_IMAGE_DATA            0xDA

#define CMD_TYPE_GET_VERSION		0x20
#define CMD_TYPE_GET_SERIAL_NUM		0x26
#define CMD_TYPE_READ_REG			0x11
#define CMD_TYPE_WRITE_REG			0x12
#define CMD_TYPE_SELF_TEST			0x2B
#define CMD_TYPE_SET_UTC_TIME		0x2D
#define CMD_TYPE_START_STREAMING	0x27
#define CMD_TYPE_STOP_STREAMING		0x28
#define CMD_TYPE_ENABLE_SENSORFT	0x2E
#define CMD_TYPE_DISABLE_SENSORFT	0x2F

#define SIZE_CMD_COMMON				8
#define SIZE_CMD_LENGTH_INFO        16
#define SIZE_CMD_SET_UTC_TIME		15
#define SIZE_CMD_READ_REGS			12
#define SIZE_CMD_WRITE_REGS_BASE	12

#define SIZE_RESP_ACK				10
#define SIZE_RESP_GET_SERIAL_NUM	26
#define SIZE_RESP_GET_VERSION		20
#define SIZE_RESP_IMU_SELF_TEST		16
#define SIZE_RESP_READ_REGS_BASE	16

#define SIZE_PACKET_CMD			20
#define SIZE_PACKET_CMD_FW		20
#define SIZE_PACKET_DATA_FW		523
#define SIZE_PACKET_BASE_DATA	18
#define SIZE_PACKET_FULL_DATA	70
#define SIZE_BUFF_RESP			26

#define SELF_TEST_RESULT_PASS	0x03
#define ERR_CODE_SUCCESS	0x00

#define BIT_SELECT_OUT_DATA_VEL		(0x10)
#define BIT_SELECT_OUT_DATA_ANG		(0x08)
#define BIT_SELECT_OUT_DATA_TEMP	(0x04)
#define BIT_SELECT_OUT_DATA_GYRO	(0x02)
#define BIT_SELECT_OUT_DATA_ACC		(0x01)

#define BIT_SAVE_ALL_CONFIG_CMD		(0x50)
#define BIT_SAVE_ALL_CONFIG_RESULT_IN_PROGRESS	(0x00)
#define BIT_SAVE_ALL_CONFIG_RESULT_SUCCESS		(0x01)
#define BIT_SAVE_ALL_CONFIG_RESULT_NOT_SAVED	(0x02)

#define BIT_ENABLE_EXT_ACCEL_BIAS		(0x01)
#define BIT_ENABLE_EXT_GYRO_BIAS		(0x02)
#define BIT_ENABLE_EXT_ACCEL_SENS		(0x04)
#define BIT_ENABLE_EXT_GYRO_SENS		(0x08)

#define IIM46230_WHO_AM_I	0xE6
#define IIM46234_WHO_AM_I	0xEA
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
enum PartNum {
	IIM46230 = 0,
	IIM46234
};

struct utc{
	uint16_t year;
	uint16_t month;
	uint16_t day;
	uint16_t hh;
	uint16_t mm;
	uint16_t ss;
};
enum IIM4623x_Intf {
	INTF_UART = 1,
	INTF_SPI
};

enum IIM4623x_Mode {
	COMMAND = 0,
	STREAMING
};

enum IIM4623x_OutDataForm {
	FLOATING = 0,	// 32-Bit IEEE 754 single-precision floating point (default)
	FIXED			// 32-Bit Fixed point 2's Complement representation
};

enum IIM4623x_DataOutPut {
	ACCEL = 0,
	GYRO,
	TEMP,
	DELTA_VEL,
	DELTA_ANG
};

enum IIM4623x_SampleRateDiv {
	ODR_1KHZ = 1,
	ODR_500HZ = 2,
	ODR_250HZ = 4,
	ODR_200HZ = 5,
	ODR_125HZ = 8,
	ODR_100HZ = 10,
	ODR_50HZ = 20,
	ODR_25HZ = 40,
	ODR_20HZ = 50,
	ODR_10HZ = 100 // 0x64
};

enum IIM4623x_UartBaudRate {
	BAUD_921600 = 0,
	BAUD_1500000 = 1,
	BAUD_3000000 = 3
};

enum IIM4623x_SyncConfig {
	DISABLE_SYNC = 0,
	SYNC_WITH_PPS = 1
};

enum IIM4623x_AccBwConfig {
	ACC_LPF_BW4 = 0x40,
	ACC_LPF_BW5 = 0x50,
	ACC_LPF_BW6 = 0x60,
	ACC_LPF_BW7 = 0x70
};

enum IIM4623x_GyroBwConfig {
	GYRO_LPF_BW4 = 0x4,
	GYRO_LPF_BW5 = 0x5,
	GYRO_LPF_BW6 = 0x6,
	GYRO_LPF_BW7 = 0x7
};

enum IIM4623x_AccelConfig0 {
	ACC_FSR_16G = 0x00,
	ACC_FSR_8G = 0x20,
	ACC_FSR_4G = 0x40,
	ACC_FSR_2G = 0x60
};

enum IIM4623x_GyroConfig0 {
	GYRO_FSR_2000DPS = 0x00,
	GYRO_FSR_1000DPS = 0x20,
	GYRO_FSR_500DPS = 0x40,
	GYRO_FSR_480DPS = 0x40,
	GYRO_FSR_250DPS = 0x60
};

enum IIM4623x_Axis {
	ACCEL_X = 0,
	ACCEL_Y,
	ACCEL_Z,
	GYRO_X,
	GYRO_Y,
	GYRO_Z
};

enum IIM4623x_CalibConfig {
	ACCEL_BIAS = 0,
	GYRO_BIAS,
	ACCEL_SENS,
	GYRO_SENS
};

enum IIM4623x_Mat_Index {
	X_1 = 0,
	X_2,
	X_3,
	Y_1,
	Y_2,
	Y_3,
	Z_1,
	Z_2,
	Z_3
};

typedef union{
	//uint8_t u8[4]; // Raw bytes received from SPI/UART
	float val; //Floating point value
	uint32_t u32; //Fixed point value
}float_uint_t;
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

struct IIM4623x_State {

	uint32_t fw_size;
	uint32_t fw_sent_data;
	bool fw_sending_done;

	enum IIM4623x_Mode mode;
	uint8_t fw_cmd_packet[SIZE_PACKET_DATA_FW];
	uint8_t cmd_packet[SIZE_PACKET_CMD];
	bool stp_cmd_in_streaming;
	bool utc_cmd_in_streaming;
	utc utc_time;
	PartNum part_num;
	enum IIM4623x_Intf streaming_intf;
	enum IIM4623x_OutDataForm data_form;
	enum IIM4623x_UartBaudRate baud_rate;
	enum IIM4623x_SyncConfig sync_config;
	uint8_t data_out;
	enum IIM4623x_SampleRateDiv rate;
	uint8_t lpf_bw;
	uint8_t accel_fsr;
	uint8_t gyro_fsr;
	uint8_t calib_config;
	float_uint_t acc_x;
	float_uint_t acc_y;
	float_uint_t acc_z;
	float_uint_t gyro_x;
	float_uint_t gyro_y;
	float_uint_t gyro_z;
	float_uint_t temp;
	float_uint_t d_vel_x;
	float_uint_t d_vel_y;
	float_uint_t d_vel_z;
	float_uint_t d_ang_x;
	float_uint_t d_ang_y;
	float_uint_t d_ang_z;	
} state;

extern const AP_HAL::HAL& hal;

uint8_t cmd_get_serial_number[20]= {0x24, 0x24,
                                        0x08,
                                        0x26,
                                        0x00, 0x26,
                                        0x0D, 0x0A,
                                        0x00, 0x00, 0x00, 0x00, 
                                        0x00, 0x00, 0x00, 0x00, 
                                        0x00, 0x00, 0x00, 0x00}; 

uint8_t cmd_set_output_to_fixed_point[20]= {36, 36,
                                        13,
                                        18,
                                        0, 1,
                                        25, 0,
                                        1, 0, 45, 13, 
                                        10, 0, 0, 0, 
                                        0, 0, 0, 0}; 

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
uint16_t calc_checksum(uint8_t *buff, uint32_t length)
{
	uint16_t sum = 0;
	for (uint32_t i = 0; i < length; i++) sum += (uint16_t)buff[i];
	return sum;
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
        expected_sample_rate_hz = 4000;
        return true;

    }
    }
    return false;
}

// uint16_t AP_InertialSensor_IIM4623x::calc_checksum(uint8_t *buff, uint32_t length)
// {
// 	uint16_t sum = 0;
// 	for (uint32_t i = 0; i < length; i++) sum += (uint16_t)buff[i];
// 	return sum;
// }
uint8_t rbuf_data[SIZE_PACKET_FULL_DATA];

void AP_InertialSensor_IIM4623x::read_sensor()
{   WITH_SEMAPHORE(dev->get_semaphore());

    state.stp_cmd_in_streaming = false;
        

    if (hal.gpio->read(drdy_pin))
    {
        dev->transfer(nullptr, 0,(uint8_t *)&rbuf_data, 46);
    }

    iim4623x_data data;
    
    memcpy((uint8_t *)&data, rbuf_data,sizeof(data));


    if (data.hdr ==0x2323 && data.footer == 0x0D0A ){
        uint16_t sum = 0;
	    for (uint32_t i = 3; i < sizeof(rbuf_data)-7; i++) sum += (uint16_t)rbuf_data[i];
        if (sum !=data.checksum){
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IIM_4623x: data is not valid");
           return; 
        }
        
    }
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IIM_4623x: data is valid");
   
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

int bus_type = 0;  // SPI
// int bus_type = 1;  // UART

uint32_t read_length = 0;




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
void IIM4623x_SetCMD_Common(uint8_t cmd_type)
{
	for (uint32_t i = 0; i < SIZE_PACKET_CMD; i++) state.cmd_packet[i] = 0x00;
	state.cmd_packet[0] = BYTE_HEADER_CMD;
	state.cmd_packet[1] = BYTE_HEADER_CMD;
	state.cmd_packet[2] = SIZE_CMD_COMMON;
	state.cmd_packet[3] = cmd_type;
	uint16_t checksum = calc_checksum(&state.cmd_packet[3], 1);
	state.cmd_packet[4] = (uint8_t)(checksum >> 8);
	state.cmd_packet[5] = (uint8_t)(checksum &= 0x00FF);
	state.cmd_packet[6] = BYTE_FOOTER_1;
	state.cmd_packet[7] = BYTE_FOOTER_2;
}
// uint8_t cmd_packet[20];

void IIM4623x_SetCMD_ReadRegister()
{
	for (uint32_t i = 0; i < SIZE_PACKET_CMD; i++) state.cmd_packet[i] = 0x00;
	state.cmd_packet[0] = BYTE_HEADER_CMD;
	state.cmd_packet[1] = BYTE_HEADER_CMD;
	state.cmd_packet[2] = SIZE_CMD_READ_REGS;
	state.cmd_packet[3] = CMD_TYPE_READ_REG;
	state.cmd_packet[4] = BYTE_RESERVED;
	state.cmd_packet[5] = 1;//user_reg.length;
	state.cmd_packet[6] = 0x00;//user_reg.first_addr;
	state.cmd_packet[7] = 0;//user_reg.page_id;
	uint16_t checksum = calc_checksum(&state.cmd_packet[3], 5);
	state.cmd_packet[8] = (uint8_t)(checksum >> 8);
	state.cmd_packet[9] = (uint8_t)(checksum &= 0x00FF);
	state.cmd_packet[10] = BYTE_FOOTER_1;
	state.cmd_packet[11] = BYTE_FOOTER_2;
}
float_uint_t* u32_sample[13];
uint32_t g_ul_out_num = 0;

void IIM4623x_Set_ODR( uint8_t *value)
{
	for (uint32_t i = 0; i < SIZE_PACKET_CMD; i++) state.cmd_packet[i] = 0x00;
	state.cmd_packet[0] = BYTE_HEADER_CMD;
	state.cmd_packet[1] = BYTE_HEADER_CMD;
	state.cmd_packet[2] = SIZE_CMD_WRITE_REGS_BASE + 2;
	state.cmd_packet[3] = CMD_TYPE_WRITE_REG;
	state.cmd_packet[4] = BYTE_RESERVED;
	state.cmd_packet[5] = 2;//user_reg.length;
	state.cmd_packet[6] = 0x1A;//user_reg.first_addr;
	state.cmd_packet[7] = 0;//user_reg.page_id;
    state.cmd_packet[8] = (uint8_t)((*value) >> 8);
    state.cmd_packet[9] = (uint8_t)((*value) &= 0x00FF);
	uint16_t checksum = calc_checksum(&state.cmd_packet[3], 5+2);
	state.cmd_packet[8+2] = (uint8_t)(checksum >> 8);
	state.cmd_packet[9+2] = (uint8_t)(checksum &= 0x00FF);	
	state.cmd_packet[10+2] = BYTE_FOOTER_1;
	state.cmd_packet[11+2] = BYTE_FOOTER_2;
}

void set_read_length(uint32_t *length)
{
	*length = SIZE_PACKET_BASE_DATA + 4*g_ul_out_num;
}
uint8_t rec_packet[20];
int count;
void AP_InertialSensor_IIM4623x::transfer_packet(uint8_t out_packet[20], int rec_pkt_len){
    WITH_SEMAPHORE(dev->get_semaphore());
    
    
    dev->transfer(out_packet,20,nullptr, 0);
    count =0;
    while (!hal.gpio->read(drdy_pin)){
        count = count+1;
        if (count>2000){
            break;
        }
        
    }
    count =0;
    dev->transfer(nullptr, 0,(uint8_t *)&rec_packet, rec_pkt_len);
    hal.scheduler->delay(1);
}
// uint8_t drdy_state;
bool AP_InertialSensor_IIM4623x::init()
{
    _clip_limit = 7.5f * GRAVITY_MSS;
    gyro_scale = radians(0.1);
    expected_sample_rate_hz = 4000;
    // drdy_state = hal.gpio->read(drdy_pin);//drdy_pin=60

    WITH_SEMAPHORE(dev->get_semaphore());
    //read whoami
    IIM4623x_SetCMD_ReadRegister();
    transfer_packet(state.cmd_packet,17);
    transfer_packet(cmd_set_output_to_fixed_point,10);

    uint8_t odr_rate = 1;
    IIM4623x_Set_ODR(&odr_rate);
    transfer_packet(state.cmd_packet,10);

    IIM4623x_SetCMD_Common(CMD_TYPE_START_STREAMING);
    dev->transfer(state.cmd_packet,8,nullptr, 0);



    


    return true;
}

/*
  sensor read loop
 */

void AP_InertialSensor_IIM4623x::loop(void)
{
    while (true) {
        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IIM_4623x: whoami ");
        uint32_t tstart = AP_HAL::micros();
        // we deliberately set the period a bit fast to ensure we
        // don't lose a sample
        const uint32_t period_us = (1000000UL / expected_sample_rate_hz) - 20U;
        bool wait_ok = false;

        WITH_SEMAPHORE(dev->get_semaphore());
    

        // wait_ok = hal.gpio->wait_pin(drdy_pin, AP_HAL::GPIO::INTERRUPT_RISING, 2100);
        // streaming data
        read_sensor();
        

        
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
