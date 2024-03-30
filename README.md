# Integrated-VCU
Integration of the VCU: the car's safety sensors(APPS, BSE, ...), the Eletronic differential and CAN 2.0

## Getting Started!

### Prerequisites

To decrease the complexity of the code and improve it's safety, we've created 3 Header files with all the methods used:

CAN 2.0 has methods to get and send data to both Inverters  
```
CAN_2.h 
```

Electronic Differential has methods calculate the angular speed of each wheel
```
Electronic_Differential.h
```

VCU_Sensors class has methods to get and validate data from sensors (APPS, BSE, Steering Wheel) 
```
VCU_Sensors.h
```
