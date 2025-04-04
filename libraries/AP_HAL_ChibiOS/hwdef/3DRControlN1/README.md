# 3DR Control N1 

The Control N1 is a flight controller produced by [3DR](https://store.3dr.com/control-n1/), launched on May 2025.

![3DR Control N1](https://vddwxegfxugwzpfnrrlp.supabase.co/storage/v1/object/public/Website-CDN/autopilot-img/control_n1.png)
![3DR Control N1 - Mounted](https://vddwxegfxugwzpfnrrlp.supabase.co/storage/v1/object/public/Website-CDN/autopilot-img/CZOEM_revG_back.JPG?t=2024-03-08T20%3A18%3A49.140Z)

## Features

 - Processor
	 - STM32H743 32-bit Arm Cortex M7 core, double-precision FPU
Sensors
	 - 2x ICM56686 Accel, Gyro
	 - ICT15312 Magnetometer
	 - DPS368 Barometer
 - Power
	 - External Power Supply
	 - Logic level IO at 3.3V
 - Interfaces
	 
	 - 11x PWM / IO - (DMA capability TBC)
	 - 3x GPIO 
	 - 7x UARTs (3x w/hardware flow control)
	 - 2x CAN
	 - 1x SPI
	 - 3x I2C
	 - uSD card socket
 - Memory
	 - 1Mb F-RAM
 - Miscellaneous
	 - Onboard 3 color LED (WS2812B Type)
	 - Mini Buzzer
- Connectors: 2x Front DF40 30-pin plug type connector, 1x rear DF40 80-pin plug type connector.


###  Physical parameters

 - Weight: TBD (oz)
 - Width:  17.6mm (.69in)
 - Length: 28mm (1.1in)


## Changelog

### R0024A.0 Initial release

## Pinout

![Control N1](https://vddwxegfxugwzpfnrrlp.supabase.co/storage/v1/object/public/Website-CDN/pinouts/czoem_pinout_revG_topview.png)

## UART Mapping

- SERIAL0 -> USB1, for GCS connection

- SERIAL1 -> USART2 (TELEM 1) DMA Enabled

- SERIAL2 -> UART4 (TELEM 2) DMA Enabled

- SERIAL3 -> USART3 (GPS) DMA Enabled

- SERIAL4 -> USART6 (GPS 2) DMA Enabled

- SERIAL5 -> UART8 

- SERIAL6 -> UART7

- SERIAL7 -> USART1 

- SERIAL7 -> USB2, MAVLink interface

## RC Input

Default serial port .
All serial-based protocols supported, any serial port may be repurposed by setting SERIALx_PROTOCOL to 23

Spektrum DSM / DSM2 / DSM-X® Satellite compatible input and binding. 
Futaba S.BUS® & S.BUS2® compatible input.  (No need for inverter)
Graupner SUMD. Yuneec ST24.

## Analog Inputs

The Control N1 has 3 ADC inputs:

- ADC1 Pin10 -> Battery Voltage
- ADC1 Pin18 -> Battery Current
- ADC1 Pin8 -> Internal 5V sensor

## PWM Output

The Control N1 supports 11 PWM outputs.  All DShot and BiDirDShot capable (TBC)

The PWM outputs are distributed in 3 groups:

- PWM 1-4 in group 1
- PWM 5-8 in group 2
- PWM 9-11 in group 4

Channels within the same group must use only one output rate. If any channel is using DShot or BiDirDShot the rest of the group will use the said output type.

## Addressable LED

The RGB LED onboard is a WS2812B addressable type LED, connected to PWM channel 12 (TIM3-CH2).
The LED's data out pin is exposed as ADDR_LED.

## Power Supply

This board requires a 5V, 500mA power supply

## Battery Monitoring

This board has a built-in voltage and current sensors. The following settings need to be present already on the board to work with a Power Zero Module (M10077):

- BATT_MONITOR 4
- BATT_VOLT_PIN 10
- BATT_CURR_PIN 18
- BATT_VOLT_SCALE 15.3
- BATT_CURR_SCALE 50.0

*Other Power Module needs to be adjusted accordingly*

## Build

`./waf configure --board=3DRControlN1`

`./waf copter` (check ArduPilot's docs for more info about the available targets)

The compiled binary will be located in `build/3DRControlZeroG/bin/arducopter.apj`.

## Uploading Firmware

Any Control N1 has a preloaded Ardupilot bootloader, which allows the user to use a compatible Ground Station software to upload the `.apj` file.  
