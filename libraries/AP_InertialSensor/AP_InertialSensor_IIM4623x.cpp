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


/*
* The atmel development kit, reference driver can be downloaded from the following link
* https://invensense.tdk.com/developers/download/iim-4623x-evaluation-kit-3-6/
* additional features can be implemented similar to the atmel driver
*/

#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <fstream>
#include <chrono>
#include <iomanip>

//#include <iostream>
//#include <cstdint>
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
/*
Note:

Exact number of bytes should be read from the response buffer,
otherwise the DRDY does not work( The DMA raises exception inside the IIM module)
*/

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
//the size is 48 not 46 rounding off the bytes


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

typedef struct IIM4623x_reg{
	uint8_t first_addr;
	uint8_t length;
	uint8_t page_id;
} reg;

reg WHO_AM_I			= {0x00, 1 , 0};
reg SERIAL_NUM 			= {0x01, 16, 0};
reg FIRMWARE_REV		= {0x11, 2 , 0};
reg BOOTLOADER_REV		= {0x13, 2 , 0};
reg FLASH_ENDURANCE		= {0x15, 4 , 0};
reg OUT_DATA_FORM		= {0x19, 1 , 0};
reg SAMPLE_RATE_DIV		= {0x1A, 2 , 0};
reg SELECT_OUT_DATA		= {0x1C, 1 , 0};
reg UART_IF_CONFIG		= {0x1D, 1 , 0};
reg SYNC_CONFIG			= {0x1E, 1 , 0};
reg USER_SCRATCH1		= {0x1F, 8 , 0};
reg USER_SCRATCH2		= {0x27, 8 , 0};
reg SAVE_ALL_CONFIG		= {0x2F, 1 , 0};
reg BW_CONFIG			= {0x30, 1 , 0};
reg ACCEL_CONFIG0       = {0x33, 1 , 0};
reg GYRO_CONFIG0        = {0x34, 1 , 0};
reg EXT_CALIB_CONFIG    = {0x3F, 1 , 0};
reg EXT_ACCEL_X_BIAS    = {0x40, 4 , 0};
reg EXT_ACCEL_Y_BIAS    = {0x44, 4 , 0};
reg EXT_ACCEL_Z_BIAS    = {0x48, 4 , 0};
reg EXT_GYRO_X_BIAS     = {0x4C, 4 , 0};
reg EXT_GYRO_Y_BIAS     = {0x50, 4 , 0};
reg EXT_GYRO_Z_BIAS     = {0x54, 4 , 0};

reg EXT_ACC_SENS_MAT11  = {0x58, 4 , 0};
reg EXT_ACC_SENS_MAT12  = {0x5C, 4 , 0};
reg EXT_ACC_SENS_MAT13  = {0x60, 4 , 0};
reg EXT_ACC_SENS_MAT21  = {0x64, 4 , 0};
reg EXT_ACC_SENS_MAT22  = {0x68, 4 , 0};
reg EXT_ACC_SENS_MAT23  = {0x6C, 4 , 0};
reg EXT_ACC_SENS_MAT31  = {0x70, 4 , 0};
reg EXT_ACC_SENS_MAT32  = {0x74, 4 , 0};
reg EXT_ACC_SENS_MAT33  = {0x78, 4 , 0};

reg EXT_GYR_SENS_MAT11  = {0x7C, 4 , 0};
reg EXT_GYR_SENS_MAT12  = {0x80, 4 , 0};
reg EXT_GYR_SENS_MAT13  = {0x84, 4 , 0};
reg EXT_GYR_SENS_MAT21  = {0x88, 4 , 0};
reg EXT_GYR_SENS_MAT22  = {0x8C, 4 , 0};
reg EXT_GYR_SENS_MAT23  = {0x90, 4 , 0};
reg EXT_GYR_SENS_MAT31  = {0x94, 4 , 0};
reg EXT_GYR_SENS_MAT32  = {0x98, 4 , 0};
reg EXT_GYR_SENS_MAT33  = {0x9C, 4 , 0};

reg DATA_READY_STATUS	= {0x00, 1 , 1};
reg TIMESTAMP_OUT		= {0x03, 8 , 1};
reg ACCEL_X_OUTPUT		= {0x0B, 4 , 1};
reg ACCEL_Y_OUTPUT		= {0x0F, 4 , 1};
reg ACCEL_Z_OUTPUT		= {0x13, 4 , 1};
reg GYRO_X_OUTPUT		= {0x17, 4 , 1};
reg GYRO_Y_OUTPUT		= {0x1B, 4 , 1};
reg GYRO_Z_OUTPUT		= {0x1F, 4 , 1};
reg TEMPERA_OUTPUT		= {0x23, 4 , 1};
reg DELTA_VEL_X			= {0x27, 4 , 1};
reg DELTA_VEL_Y			= {0x2B, 4 , 1};
reg DELTA_VEL_Z			= {0x2F, 4 , 1};
reg DELTA_ANGLE_X		= {0x33, 4 , 1};
reg DELTA_ANGLE_Y		= {0x37, 4 , 1};
reg DELTA_ANGLE_Z		= {0x3B, 4 , 1};
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

// void set_read_length(uint32_t *length)
// {
// 	*length = SIZE_PACKET_BASE_DATA + 4*g_ul_out_num;
// }

uint16_t calc_checksum(uint8_t *buff, uint32_t length)
{
	uint16_t sum = 0;
	for (uint32_t i = 0; i < length; i++) sum += (uint16_t)buff[i];
	return sum;
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


void IIM4623x_SetCMD_WriteRegister(reg user_reg, uint8_t *value)
{
	for (uint32_t i = 0; i < SIZE_PACKET_CMD; i++) state.cmd_packet[i] = 0x00;
	state.cmd_packet[0] = BYTE_HEADER_CMD;
	state.cmd_packet[1] = BYTE_HEADER_CMD;
	state.cmd_packet[2] = SIZE_CMD_WRITE_REGS_BASE + user_reg.length;
	state.cmd_packet[3] = CMD_TYPE_WRITE_REG;
	state.cmd_packet[4] = BYTE_RESERVED;
	state.cmd_packet[5] = user_reg.length;
	state.cmd_packet[6] = user_reg.first_addr;
	state.cmd_packet[7] = user_reg.page_id;
	if (user_reg.length == 1)
		state.cmd_packet[8] = *value;
	else if (user_reg.length == 2) {
		state.cmd_packet[8] = (uint8_t)((*value) >> 8);
		state.cmd_packet[9] = (uint8_t)((*value) &= 0x00FF);
	}
	else if (user_reg.length == 4) {
		state.cmd_packet[8] = *value;
		state.cmd_packet[9] = *(value+1);
		state.cmd_packet[10] = *(value+2);
		state.cmd_packet[11] = *(value+3);	
	}	
	
	uint16_t checksum = calc_checksum(&state.cmd_packet[3], 5+user_reg.length);
	state.cmd_packet[8+user_reg.length] = (uint8_t)(checksum >> 8);
	state.cmd_packet[9+user_reg.length] = (uint8_t)(checksum &= 0x00FF);	
	state.cmd_packet[10+user_reg.length] = BYTE_FOOTER_1;
	state.cmd_packet[11+user_reg.length] = BYTE_FOOTER_2;
}


float_uint_t* u32_sample[13];
uint32_t g_ul_out_num = 0;

void set_read_length(uint32_t *length)
{
	*length = SIZE_PACKET_BASE_DATA + 4*g_ul_out_num;
}

void IIM4623x_Start_Streaming(void)
{

    u32_sample[g_ul_out_num] = &state.acc_x;
    g_ul_out_num++;
    u32_sample[g_ul_out_num] = &state.acc_y;
    g_ul_out_num++;
    u32_sample[g_ul_out_num] = &state.acc_z;
    g_ul_out_num++;			

    u32_sample[g_ul_out_num] = &state.gyro_x;
    g_ul_out_num++;
    u32_sample[g_ul_out_num] = &state.gyro_y;
    g_ul_out_num++;
    u32_sample[g_ul_out_num] = &state.gyro_z;
    g_ul_out_num++;			



    u32_sample[g_ul_out_num] = &state.temp;
    g_ul_out_num++;
   


    IIM4623x_SetCMD_Common(CMD_TYPE_START_STREAMING);
    

}
uint8_t rec_packet[20];
int count;

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

AP_InertialSensor_Backend *AP_InertialSensor_IIM4623x::probe(AP_InertialSensor &imu,
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

inline uint32_t read_u32_data(uint8_t *p_buf)
{
	uint32_t value;
	value = ((uint32_t)(*p_buf)<<24 | 
		(uint32_t)(*(p_buf+1))<<16 | 
		(uint32_t)(*(p_buf+2))<<8 | 
		(uint32_t)(*(p_buf+3)));

	return value;
}
void AP_InertialSensor_IIM4623x::read_sensor()
{   WITH_SEMAPHORE(dev->get_semaphore());

    state.stp_cmd_in_streaming = false;
        
    iim4623x_data imu_data;

    if (hal.gpio->read(drdy_pin))
    {
        dev->transfer(nullptr, 0,(uint8_t *)&rbuf_data, 46);
    }

    // uint32_t sample_val;
    for (uint32_t i=0; i<g_ul_out_num; i++) {
        // sample_val = read_u32_data(&rbuf_data[14+4*i]);
        // u32_sample[i]->val = *((float*)&sample_val);
        u32_sample[i]->u32 = read_u32_data(&rbuf_data[14+4*i]);
    }


    int32_t ax_int = int32_t(state.acc_x.u32); 
    int32_t ay_int = int32_t(state.acc_y.u32); 
    int32_t az_int = int32_t(state.acc_z.u32); 
    int32_t gx_int = int32_t(state.gyro_x.u32); 
    int32_t gy_int = int32_t(state.gyro_y.u32); 
    int32_t gz_int = int32_t(state.gyro_z.u32); 
    int32_t temp_int = int32_t(state.temp.u32);

    float accel_factor = 16.0/pow(2.0,31.0);
    float gyro_factor = 2000.0/pow(2.0,31.0);

    float ax = accel_factor * ax_int;
    float ay = accel_factor * ay_int;
    float az = accel_factor * az_int;
    float gx = gyro_factor * gx_int;
    float gy = gyro_factor * gy_int;
    float gz = gyro_factor * gz_int;

    Vector3f accel{ax, -ay, -az};
    Vector3f gyro{gx, -gy, -gz};
    // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "accel: %ld, %ld, %ld gyro: %ld, %ld, %ld", state.acc_x.u32, state.acc_y.u32, state.acc_z.u32, state.gyro_x.u32, state.gyro_y.u32, state.gyro_z.u32);
    memcpy((uint8_t *) &imu_data, rbuf_data,sizeof(imu_data));//sizeof(data)
    
    


    // accel *= accel_factor;
    // gyro *= gyro_factor;
    // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Gyro raw: %ld Gyro scaled: (%f)", state.gyro_x.u32,  gyro.x);
    // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Accel raw: %ld Accel scaled: (%f)", state.acc_x.u32,  accel.x);
    if (imu_data.hdr ==0x2323 && imu_data.footer == 0x0D0A ){
        uint16_t sum = 0;
	    for (uint32_t i = 3; i < sizeof(rbuf_data)-7; i++) sum += (uint16_t)rbuf_data[i];
        if (sum !=imu_data.checksum){
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IIM_4623x: data is not valid");
           return; 
        }
        
    }

    

    // uint32_t ax_int = (rbuf_data[13] << 24) | (rbuf_data[12] << 16) | (rbuf_data[11] << 8) | rbuf_data[10];
    // uint32_t ay_int = (rbuf_data[17] << 24) | (rbuf_data[16] << 16) | (rbuf_data[15] << 8) | rbuf_data[14];
    // uint32_t az_int = (rbuf_data[21] << 24) | (rbuf_data[20] << 16) | (rbuf_data[19] << 8) | rbuf_data[18];
    // uint32_t gx_int = (rbuf_data[25] << 24) | (rbuf_data[24] << 16) | (rbuf_data[23] << 8) | rbuf_data[22];
    // uint32_t gy_int = (rbuf_data[29] << 24) | (rbuf_data[28] << 16) | (rbuf_data[27] << 8) | rbuf_data[26];
    // uint32_t gz_int = (rbuf_data[33] << 24) | (rbuf_data[32] << 16) | (rbuf_data[31] << 8) | rbuf_data[30];

   
    // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Accel raw: (%ld, %ld,%ld) Accel: (%f, %f,%f)", state.acc_x.u32, state.acc_y.u32, state.acc_z.u32, accel.x, accel.y, accel.z);
    // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Accel: (%f, %f,%f) Gyro: (%f, %f,%f)", accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);
    // // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Gyro raw: (%lu, %lu,%lu) Gyro: (%.2f, %.2f,%.2f)", imu_data.gx, imu_data.gy, imu_data.gz, gyro.x, gyro.y, gyro.z);
    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel);

    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro);

    
    // Vector3f dvel{};
    // Vector3f dangle{};

    // // compensate for clock errors, see "DELTA ANGLES" in datasheet
    // dangle *= expected_sample_rate_hz / _gyro_raw_sample_rate(gyro_instance);
    // dvel *= expected_sample_rate_hz / _accel_raw_sample_rate(gyro_instance);

    // _notify_new_delta_velocity(accel_instance, dvel);
    // _notify_new_delta_angle(gyro_instance, dangle);

    /*
      publish average temperature at 20Hz
     */
    temp_sum += float(temp_int/126.8 +25.0);
    temp_count++;

    if (temp_count == 100) {
        _publish_temperature(accel_instance, temp_sum/temp_count);
        temp_sum = 0;
        temp_count = 0;
        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IIM_4623x: Received %d \n", imu_data.sample_ctr);
    }
}

int bus_type = 0;  // SPI
// int bus_type = 1;  // UART

uint32_t read_length = 0;




// functions for 32 bit registers
// bool AP_InertialSensor_IIM4623x::write_word(const uint8_t reg, const uint32_t data) const
// {
//     const uint8_t b[5] { reg,  uint8_t (data&0xff), uint8_t(data >> 8), uint8_t(data >> 16), uint8_t(data >> 24) };
//     return dev->transfer(b, sizeof(b), nullptr, 0);
// }

// bool AP_InertialSensor_IIM4623x::read_word(const uint8_t reg, uint32_t& data) const
// {
//     return dev->read_registers(reg, (uint8_t *)&data, sizeof(data));
// }
void IIM4623x_Set_AccelConfig0(enum IIM4623x_AccelConfig0 fsr)
{

	state.accel_fsr &= 0x1f;
	state.accel_fsr |= fsr;
	IIM4623x_SetCMD_WriteRegister(ACCEL_CONFIG0, (uint8_t *)&state.accel_fsr);

}
void IIM4623x_Set_GyroConfig0(enum IIM4623x_GyroConfig0 fsr)
{
	state.gyro_fsr &= 0x1f;
	state.gyro_fsr |= fsr;
	IIM4623x_SetCMD_WriteRegister(GYRO_CONFIG0, (uint8_t *)&state.gyro_fsr);

}


void AP_InertialSensor_IIM4623x::transfer_packet(uint8_t out_packet[20], int rec_pkt_len){
    WITH_SEMAPHORE(dev->get_semaphore());
    
    
    dev->transfer(out_packet,SIZE_PACKET_CMD,nullptr, 0);
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
    dev->set_device_type(DEVTYPE_INS_IIM4623X);
    //read whoami
    IIM4623x_SetCMD_ReadRegister();
    transfer_packet(state.cmd_packet,SIZE_RESP_READ_REGS_BASE+state.cmd_packet[5]);

    // set output to fixed point
    transfer_packet(cmd_set_output_to_fixed_point,SIZE_RESP_ACK);

    // OUT_DATA_FORM  set output to floating point
    // uint8_t data_form = 0;
    // IIM4623x_SetCMD_WriteRegister(OUT_DATA_FORM, &data_form);
    // transfer_packet(state.cmd_packet,SIZE_RESP_ACK);

    //set odr rate to 1kHz
    uint8_t odr_rate = 1;
    IIM4623x_SetCMD_WriteRegister(SAMPLE_RATE_DIV, &odr_rate);
    transfer_packet(state.cmd_packet,SIZE_RESP_ACK);

    //set accel fsr to 16g
    IIM4623x_Set_AccelConfig0(ACC_FSR_16G);
    transfer_packet(state.cmd_packet,SIZE_RESP_ACK);

    //set gyro fsr to 2000dps
    IIM4623x_Set_GyroConfig0(GYRO_FSR_2000DPS);
    transfer_packet(state.cmd_packet,SIZE_RESP_ACK);



    IIM4623x_Start_Streaming();
    dev->transfer(state.cmd_packet,20,nullptr, 0);
    



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
