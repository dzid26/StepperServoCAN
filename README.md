# Stepper CAN Actuator firmware
- The currently targetted hardware is S42Bv2 board or similar. 

## Hardware (BTT S42Bv2, S57Bv2)
- STMicroelectronics' 32-bit MCU, STM32F103C8T6 ARM 32-bit, Cortex™-M3 CPU Core, 72MHz maximum frequency, 20k RAM, 64k Flash (but really 128k ??).
- TLE5012 15bit magnetic (GMR) angle sensor. Also has temperature sensor.
- A4950 current drivers (integrated mosfets) - **IMPORTANT** - do not back drive the motor without the [board modification](https://github.com/dzid26/RetroPilot-SERVO42B/wiki/Board-BEMF-protection-mod)
### Tutorials
- Attaching magnet with the glue [youtube](https://youtu.be/mQyXR3hITy0?t=41)
- Finding S42Bv2 may not be possible anymore, but there are still some S57Bv2 available. [click here](https://www.aliexpress.com/item/1005001863622626.html). Others, like S42C are not compatible. No CAN, different CPU, etc. 

## Firmware 
- The frmware is compatible with Bigtreetech S42Bv2 and S57Bv2 boards.
- Dip switches, step, dir, enable pin, USART are currenlty NOT used in the firmware
- Uses ST's old Standard Peripheral Library (src/lib) for registers configuration
- Buttons order on the board is set as follows: 
    `Next` -- `Menu` -- `Enter`
- CAN handling c-code is generated from dbc file using cantools. See `generate_Msg.sh` 

### Build and upload
- Specify your programming dongle in `firmware/platformio.ini` - default is stlink
- The easiest way to build is to load `firmware` folder into VScode workspace and loading Platformio IDE extension. [Quick start](https://docs.platformio.org/en/latest/integration/ide/vscode.html).
- Alternatively [Platformio Core](https://docs.platformio.org/en/latest/core/installation/index.html) can be used to build from command line:
```
    cd /firmware
    platformio run
```
- Upload firmware to the board by pressing Upload arrow at the status bar in VScode
- Eeprom is not erased when flashing the firmware - any future calibration will not be lost.

## Calibration and first run
- On first start defualt parameters are loaded and then calibrated and stored in eeprom.
- During first start two phases are briefly actuated and `motorParams.motorWiring` state is automatically determined based on angle sensor movement. (you will know if this parameter is incorrect, if phases will be audibly actuated in wrong order). This corresponds to 
- Next the display will prompt to calibrate the motor. Press `Enter` to start calibration. The motor will be calibrated and values stored. Press `Enter` again to exit calibration mode.
- Actuator parameters need to be specified for CANbus signal units to be converted from actuator output domain to the motor domain. Change gearbox and final gear ratios in `firmware/actuator_config.h` file: `rated_current`, `rated_torque`, `motor_gearbox_ratio`, `final_drive_ratio`. Rebuild firmware and upload to the board.
- Depending on mounting orientation and gearing the motor rotation direction might be reversed. If this is the case, reverse the motor direction by navigating on display menu to `Rotation` and changing the parameter. This corresponds to `SystemParams.dirRotation` in the code.
## CAN interface
Actuator accepts commands via CANbus as defined by `dbc` file in [Retropilot/Opendbc/ocelot_controls.dbc](https://github.com/RetroPilot/opendbc/blob/Ocelot-steering-dev/ocelot_controls.dbc)

CAN Command - expect rate is 10ms
- 0x22E (0558) STEERING_COMMAND
    - STEER_TORQUE (Nm)
    - STEER_ANGLE (deg)
    - STEER_MODE
        - 0 - "Off" - instant 0 torque
        - 1 - "TorqueControl" - uses STEER_TORQUE signal to control torque
        - 2 - "RelativeControl" **DEPRICATED** - uses STEER_ANGLE signal to control relative angle using PID and STEER_TORQUE as feedforward
        - 3 - "SoftOff" - ramp torque to 0 in 1s - meant to be used for coommunication error safe mode
    - COUNTER
    - CHECKSUM

Actuator will report back its status every 10ms:
- 0x22F (0559) STEERING_STATUS
    - STEERING_ANGLE (deg)
    - STEERING_SPEED (rev/s) - 
    - STEERING_TORQUE (Nm)
    - CONTROL_STATUS
    - COUNTER
    - CHECKSUM
    - TEMPERATURE (C)
    - DEBUG_STATES

## BSP Firmware License 
- The firmware is based on Misfittech project which is based on [nano_stepper](https://github.com/Misfittech/nano_stepper) project and it inherited GPL V3 license
- I continue [GPL v3 license](https://github.com/dzid26/RetroPilot-SERVO42B/blob/openpilot_S42B/LICENSE) scheme as required for the software derivatives
- Derivatives of this software under GPLv3 license will also need to remain open source if distributed commercially.
