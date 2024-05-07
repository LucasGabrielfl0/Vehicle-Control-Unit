# Integrated-VCU
Integration of the VCU, the car's safety sensors(APPS, BSE, ...), the Electronic differential and CAN 2.0

## Getting Started!

To decrease the complexity of the main code and improve it's safety, 2 Header files were created:

-  angle_sensors 
-  motor_can

### Angle Sensors
The class has methods to get and validate data from Pedals and other angle related sensors (APPS, BSE, Steering Wheel) 
```
angle_sensors.h
```

### Motor Can
motor_can has methods to facilitate the communication with the motor controllers  
```
motor_can.h 
```
