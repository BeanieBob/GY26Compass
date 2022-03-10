# GY26Compass

**GY26Compass** is a library which facilitates communication with the **GY-26** Compass module. Serial and I2C communication protocols are supported by separate classes, **GY26_Uart_Compass** and **GY26_I2C_Compass**. 

- Author: Tony Brophy
- Version 0.5.0

## Tests

- The I2C and Software-Serial implementations have been tested successfully on Arduino Uno
- The I2C and Hardware-Serial implementations have been tested successfully on ESP32

## Dependencies
- **GY26_I2C_Compass**: The Arduino **Wire** library is required for I2C
- **GY26_Uart_Compass**: The Arduino **SoftwareSerial** libary is required for the serial implementation only if a Hardware port is not being used

## Usage

Simply instantiate the appropriate GY26Compass object corresponding to the desired protocol.

For **I2C**, initialize the Wire object, then call the desired methods as shown below:   
```Arduino
#include <Wire.h>
#include <GY26Compass.h>

GY26_I2C_Compass compass(0x70);

void setup(){
    Wire.begin();  // Start I2C connectivity
}

void loop(){
    compass.setDeclinationAngle(-2.4);
    float compassTemperature = compass.getTemperatureCelsius();
    float compassAngle = compass.getCompassAngle();
    /* Use above values ... */
}
```
The following sample code is relevant for a **Hardware Serial** port, in this case **Serial2** (not available on Uno)
```Arduino
#include <GY26Compass.h>

GY26_Uart_Compass compass(&Serial2);

void setup(){    
    Serial2.begin(9600); // Start Serial Connectivity 
}

void loop(){
    compass.setDeclinationAngle(-2.4);
    float compassTemperature = compass.getTemperatureCelsius();
    float compassAngle = compass.getCompassAngle();
    /* Use above values ... */
}
```

If a **SoftwareSerial** library is installed for your board, you may use it as shown below 
```Arduino
#include <GY26Compass.h>
#include <SoftwareSerial.h>

const uint8_t softRxPin = 10;
const uint8_t softTxPin = 11;

SoftwareSerial serial(softRxPin, softTxPin);
GY26_Uart_Compass compass(&serial);

void setup(){
    serial.begin(9600); // Start serial connectivity 
}

void loop(){
    compass.setDeclinationAngle(-2.5);
    float compassTemperature = compass.getTemperatureCelsius();
    float compassAngle = compass.getCompassAngle();
    /* Use above values ... */
}
```

See the examples for more details and instructions on calibration. 

---
The library source headers **GY26_I2C_Compass.h** and  **GY26_Uart_Compass.h** contain documentation which provide further insight into the available functionality  