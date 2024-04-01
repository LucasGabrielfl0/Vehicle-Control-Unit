# Integrated-VCU
Integration of the VCU, the car's safety sensors(APPS, BSE, ...), the Electronic differential and CAN 2.0

## Getting Started!

### Prerequisites

To decrease the complexity of the main code and improve it's safety, 2 Header files were created:

motor_can has methods to send/receive data from the motor controllers  
```
motor_can.h 
```

Angle Sensors class has methods to get and validate data from Pedals and angle related sensors (APPS, BSE, Steering Wheel) 
```
angle_sensors.h
```
