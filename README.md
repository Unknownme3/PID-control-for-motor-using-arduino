FOR PID CONTROL USING RMCS2303 LIBRARY ON NO-LOAD.
COMPONENTS USED
1.MOTOR RMCS5023
2.MOTOR DRIVER USED RMCS2303
3.INA219 CURRENT-POWER-VOLTAGE SENSOR.
4.MICROCONTROLLER USE ARDUINO UNO R3.

Ensure the RMCS-2303 library is installed in the Arduino IDE.
Connect the motor controllers to Arduino as described in the datasheet.
Assign unique slave IDs for each motor .

Motors are configured to run in Digital Speed Control Mode using Enable_Digital_Mode().
Current RPM is obtained using the Speed_Feedback() function.
PID logic adjusts the speed command to maintain the target RPM.
Current RPM, target RPM, and speed command values are printed to the Serial Monitor for debugging.

FOR MORE DETAILED EXPLANATION OF PINOUTS AND OTHER HELP IVE UPLOADED RMC2303 DATASHEET

