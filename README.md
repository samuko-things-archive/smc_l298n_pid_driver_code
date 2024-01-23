# smc_arduino_driver_code
This is a child project of the Samuko Motor Control (SMC) project. It contains the code to be uploaded into the arduino board (NANO or UNO) attached to the smc_driver module shield. Without this code, the sheild is as good as useless for the velocity PID control and cannot connect to the [**smc_app**](https://github.com/samuko-things/smc_app) for the velocity PID setup.


## How to Use the Driver Code
- Ensure you have the smc_driver module shield with a preffered arduino board of your choice (NANO or UNO)

- Ensure you have the Arduino IDE up and running on your PC

- Download (by clicking on the green Code button above) or clone the repo into your PC

- Open the smc_arduino_driver_code.ino file in your ArduinoIDE

- select the board (Arduino nano or uno). 
  > **Note:** if you are using nano select processor as "Atmega328P (Old Bootloader)"

- verify and upload the code.

- You can start using the smc_driver module shield with the **smc_app** for PID setup and in your project for velocity PID control. 
