/*
    GY26_I2C_ReadCompass

    Purpose:
      Using I2C, provide periodic compass readings. This code allows
      initialization with the local declination angle. Compass readings
      are then written to the Serial Monitor along with Temperature.

    Wiring Connections as follows:
      * Connect Power (3.3V/5.0V) and GND from the microcontroller board to the GY-26 board
      * Connect I2C lines (SDA and SCL) from the microcontroller board to the
        GY-26 board. Use the appropriate hardware I2C pins for your particular
        case - For Arduino boards see https://www.arduino.cc/en/reference/wire .

    Instructions:
      * To achieve accurate compass angles, search online for the appropriate 
        declination angle according to your location, and substitute in the code.  
      * Open Serial Monitor
      * Upload the sketch
      * Observe the readings on the Serial Monitor

    2022-03-06    Version 0.5.0   Tony Brophy   Initial sketch

*/

#include <Wire.h>
#include <GY26Compass.h>

/*
 * This is the compass interface. 
 * The default I2C address is 0x70
 */
GY26_I2C_Compass compass(0x70);

/*
 * Set this to the correct declination angle 
 * for your location
 */
float localDeclinationAngle = 0.0; 

float compassTemperature;
float compassAngle;

void setup(){
    Serial.begin(9600); // Start serial monitor
    Wire.begin();       // Start I2C connectivity
    compass.setDeclinationAngle(localDeclinationAngle);
}

void loop(){
  compassTemperature = compass.getTemperatureCelsius();
  compassAngle = compass.getCompassAngle();
  Serial.println("=========================================");
  Serial.print("TEMPERATURE (C): ");Serial.println(compassTemperature);
  Serial.print("COMPASS ANGLE:  ");Serial.println(compassAngle);
  Serial.println("=========================================");
  Serial.println();
  delay(1000);
}
