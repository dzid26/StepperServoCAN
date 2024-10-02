[![Sonarcloud Status](https://sonarcloud.io/api/project_badges/measure?project=dzid26_RetroPilot-SERVO42B&metric=alert_status)](https://sonarcloud.io/dashboard?id=dzid26_RetroPilot-SERVO42B)
[![CodeFactor](https://www.codefactor.io/repository/github/dzid26/StepperServoCAN/badge)](https://www.codefactor.io/repository/github/dzid26/StepperServoCAN)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/fce99e1ae7d149a0a6768b91afb259d7)](https://www.codacy.com/gh/dzid26/StepperServoCAN/dashboard)
# StepperServoCAN - firmware
- This firmware is for two phase stepper motor actuator controller. 
- It allows for smooth (close-loop) torque control by reading shaft angle from a magnetic sensor
- it support CANbus communication. It can interpret values based on specified motor gearing

## Hardware
- PCB Schematics repo [here](https://github.com/dzid26/StepperServo-hardware)
- STMicroelectronics' 32-bit MCU, STM32F103C8T6 ARM 32-bit, Cortex™-M3 CPU Core, 72MHz maximum frequency, 20k RAM, 64k Flash (but really 128k ??).
- TLE5012 15bit magnetic (GMR) angle sensor. Also provides temperature sensing.
- A4950 drivers with integrated mosfets and cureent limit 

## Firmware
- Uses ST's (old) Standard Peripheral Library (src/lib) for registers configuration
- CAN handling c-code is generated from dbc file using cantools. See `generate_Msg.sh` 
- It uses Platformio to build, upload and debug the program
- User can interract with Function buttons (F1 and F2) and Reset button (Rst) or via debug virtual serial, aka semihosting

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

### Configuration
- In `firmware/src/BSP/actuator_config.c` set:
    - `rated_current` (single phase) and `rated_torque` from motor spec or measurement. Note, this is just a datapoint and will be extrapolated up to 3.3A. Choose motor wisely.
    - `motor_gearbox_ratio` - gearbox attached to the motor
    - `final_drive_ratio` - any additional gearing - separate parameter for convenience
- Depending on mounting orientation and gearing the motor rotation direction may be reversed. You can change the direction by setting `motor_gearbox_ratio` or `final_drive_ratio` to a negative value.

### LED indicators
BLUE LED (Function):
 - short single blink after user long presses `F1` button indicating button can be released
 - solid - waiting for user confirmation [of calibration] (either with `F1` button or Enter in Platformio OpenOCD debugger virtual serial console)

RED LED (Error):
 - solid/dim/flickering - Motion task CPU overrun (shall not happen)
 - slowly blinking every 1s- encoder initialization error
 - solid ~~with short interruption every 1s~~ - waiting for power supply voltage to be above 8V (checks voltage every 1s)
### Calibration and first run
1. On first start default parameters are loaded to be later stored in Flash.
2. During first start two phases are briefly actuated and based on angle sensor movement `motorParams.motorWiring` is determined automatically.
3. Next the controller automatically waits (blue LED on) for the user to confirm sensor calibration. Press `F1` button to start calibration. The motor will be calibrated and values stored in Flash. Calibration can be repeated any time by long pressing `F1` button until first short blink of the blue LED. 
4. Actuator physical values (gearing, torque, current, etc) need to be specified `firmware/actuator_config.h`. It affectes signal values read from CANbus to internal control. CANbus values are represented in actuator domain (i.e. considering motor gearbox). Change gearbox and final gear ratios in `firmware/actuator_config.h` file. Available parameters are `rated_current`, `rated_torque`, `motor_gearbox_ratio`, `final_drive_ratio`.
5. Additionally, one can extract sensor calibration values (point 3) from the Flash using `readCalibration.py`:

    <img width="488" alt="image" src="https://github.com/dzid26/StepperServoCAN/assets/841061/5316b32f-8268-41e9-b313-c464d13543d5">
    
    ([wiki](https://github.com/dzid26/StepperServoCAN/wiki/Calibration))

## CAN interface
Actuator accepts commands via CANbus as defined by `dbc` file in [Retropilot/Opendbc/ocelot_controls.dbc](https://github.com/RetroPilot/opendbc/blob/Ocelot-steering-dev/ocelot_controls.dbc)

CAN Command - expect rate is 10ms
- 0x22E (0558) STEERING_COMMAND
    - STEER_TORQUE (Nm)
    - STEER_ANGLE (deg)
    - STEER_MODE
        - 0 - "Off" - instant 0 torque
        - 1 - "TorqueControl" - uses STEER_TORQUE signal to control torque
        - 2 - "AngleControl"- uses STEER_ANGLE signal to control absolute angle using PID close-loop and STEER_TORQUE as feedforward
        - 3 - "SoftOff" - ramp torque to 0 in 1s - meant to be used for coommunication error safe mode
    - COUNTER
    - CHECKSUM

Actuator will report back its status every 10ms:
- 0x22F (0559) STEERING_STATUS
    - STEERING_ANGLE (deg)
    - STEERING_SPEED (rev/s)
    - STEERING_TORQUE (Nm)
    - CONTROL_STATUS
    - COUNTER
    - CHECKSUM
    - TEMPERATURE (C)
    - DEBUG_STATES

### Interfacing with Openpilot
Reference implementation can be found in my bmw openpilot [repo](https://github.com/dzid26/openpilot-for-BMW-E8x-E9x/commit/51c692dd7e5940be8e6e8ddbfb46321120918d4e):
- `opendbc/` - make sure [ocelot_controls.dbc](https://github.com/RetroPilot/opendbc/blob/Ocelot-steering-dev/ocelot_controls.dbc#L77-L92) is copied here
- `selfdrive/car/xxx/xxxcan.py` - [sending CAN message](https://github.com/dzid26/openpilot-for-BMW-E8x-E9x/blob/0.74_openactuator_testdev/selfdrive/car/bmw/bmwcan.py#L4-L20)
- `selfdrive/car/xxx/carstate.py` - [parsing CAN message](https://github.com/dzid26/openpilot-for-BMW-E8x-E9x/blob/51c692dd7e5940be8e6e8ddbfb46321120918d4e/selfdrive/car/bmw/carstate.py#L235-L247)
- `panda/board/safety/safety_xxx.h`- [CAN tx filter `558`](https://github.com/dzid26/panda_bmw/blob/openactuator_dev/board/safety/safety_bmw.h#L7), some [safety](https://github.com/dzid26/panda_bmw/blob/bbdb0ad5ea7c3ce55032f3875b8b2ee431889106/board/safety/safety_bmw.h#L192-L201), and [safety testing](https://github.com/dzid26/panda_bmw/blob/openactuator_dev/tests/safety/test_bmw.py#L75-L91)

### Interfacing from a PC
Use this `StepperServoCANtester.py` GUI program to quickly test the StepperServoCAN motor. 
#### Requirements
```
pip install tkinter cantools python-can

git submodule init opendbc
git submodule update opendbc
```
#### Usage
1. Connect the StepperServoCAN motor to the computer via CAN interface supported by `python-can`.

2. Run the program:
    ```
    python StepperServoCANtester.py
    ```
3. Upon startup, the program will automatically attempt to connect to the default CAN interface, which is `pcan`. If you are using a different interface, you can select the desired CAN interface from the dropdown menu in the program. Once you have selected the interface, the program will establish a connection and start sending the 0x22E STEERING_COMMAND message with initial values of 0 for torque and angle.
4. In addition to the CAN interface dropdown menu, the program also provides two input fields where you can enter the desired motor torque and angle values. Note that torque values can be entered as floats.
5. You can update the injected values to the motor by clicking the "Update Torque/Angle Value" button or by pressing the "Return" key on your keyboard or changing Steer mode.
6. After selecting either "TorqueControl" and providing a "Steer Torque" value, or selecting "AngleControl" and providing a "Steer Angle" value, the StepperServoCAN motor will start spinning accordingly.
7. To exit the program, you can click the "Exit" button, press the "ESC" key, or simply close the window.

#### Notes
- The torque value should be between -16 and 15.875, and the angle value should be between -4096 and 4096.
- To use the program with SocketCAN on Linux, you can set up a virtual CAN interface with the following commands:
  ```
  sudo modprobe vcan
  sudo ip link add dev vcan0 type vcan
  sudo ip link set up vcan0
  ip -details -brief link | grep can | grep UP
  ```
If SocketCAN interfaces are not found, the program will abort. 

## Contributing
- Develop using MISRA C:2012 standard and analyzed using [Cppcheck](https://cppcheck.sourceforge.io/). Project is preconfigured with (`misra.json`) for [C/C++ Advanced Lint](https://marketplace.visualstudio.com/items?itemName=jbenden.c-cpp-flylint) and [SonarLint](https://marketplace.visualstudio.com/items?itemName=SonarSource.sonarlint-vscode) to highlight violations in VScode. Ask priv about the `misra_rules_set_cppcheck.txt` for cppcheck.

## BSP Firmware License 
- The firmware is based on Misfittech project which is based on [nano_stepper](https://github.com/Misfittech/nano_stepper) project and it inherited GPL V3 license
- I continue [GPL v3 license](https://github.com/dzid26/RetroPilot-SERVO42B/blob/openpilot_S42B/LICENSE) scheme as required for the software derivatives
- Derivatives of this software under GPLv3 license will also need to remain open source if distributed commercially.

## Atributions
For initial software and hardware:
- [Makerbase](https://makerbase.com.cn/)
- [Misfittech](https://misfittech.net/)
- [Bigtreetech](https://bigtree-tech.com/)
