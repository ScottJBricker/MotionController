The "MotionController" is designed for real-time control using serial communication to a microcontroller to manipulate precision motors. This API efficiently manages RAM to satisfy low resource systems.

## Firmware for MotionController

This repository contains the firmware for MotionController, which includes the TMC2130Stepper library. The firmware is licensed under the GNU Affero General Public License (AGPL) version 3.

## License and Third-Party Libraries

This project uses the following third-party libraries, each with its own respective license:

### 1. SerialCommands Library
- **License:** BSD 3-Clause License
- **Description:** This library is used to facilitate serial communication commands. It is licensed under the BSD 3-Clause License.

### 2. Teensy_PWM Library
- **License:** MIT License
- **Description:** This library is used to manage PWM (Pulse Width Modulation) on Teensy microcontrollers. It is licensed under the MIT License.

### 3. Pololu Corporation Library
- **License:** MIT License
- **Description:** This library is provided by Pololu Corporation for use with their products. It is licensed under the MIT License.

### 4. TMC2130Stepper (using AGPL-3.0)
- **License:** GNU Affero General Public License (AGPL-3.0)
- **Description:** This library is licensed under the AGPL-3.0, which requires that the source code be made available when the software is distributed, especially if it is used over a network.

### 5. AccelStepper
- **License:** GPL v3 or Commercial
- **Description:** This library by Mike McCauley is available under either the GPL v3 or a commercial license. For open source use, the GPL v3 applies, which requires that you distribute your application's source code under the same license when distributing the application. For proprietary use, a commercial license must be obtained.

For more details on these licenses, please see the `LICENSES.txt` file included with this project. Long licenses are stored in the `Long Licenses` directory.

### Source Code

The complete source code for this firmware is available in the `src` directory of this repository. Users who download or interact with this firmware are entitled to receive the source code as required by the AGPL-3.0.

For more information, please refer to the license terms provided in this repository.