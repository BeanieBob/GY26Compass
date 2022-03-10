/*
    GY26_Serial_ReadCompass

    Purpose:
      Using a Serial connection, provide periodic compass readings. This
      code allows initialization with the local declination angle. Compass
      readings are then written to the Serial Monitor along with Temperature.

    Wiring Connections as follows:
      * Connect Power (3.3V/5.0V) and GND from the microcontroller board to the GY-26 board
      * Make a crossover connection with serial lines (RX and TX) from the
        microcontroller board to the GY-26 board (i.e. connect RX to TX, and
        connect TX to RX). The RX and TX pins numbers will be particular to the
        spare Serial port (Serial1, Serial2 etc) that you choose to use, as well as
        the microcontroller board being used. For Arduino boards, see
        https://www.arduino.cc/reference/en/language/functions/communication/serial/ .
        Do not use the first serial port (Serial) as this is used for program upload, 
        and the serial monitor. If you wish to test on an Uno, or any other board with
        no spare Serial hardware, you can use the Software Serial implementation and
        connect using I/O pins as commented in the provided code below.

    Instructions:
      * Ensure the unused code for either Hardware Serial or Software Serial
        is commented out below as appropriate. One of these must be commented.
      * To achieve accurate compass angles, search online for the appropriate 
        declination angle according to your location, and substitute in the code.  
      * Open Serial Monitor
      * Upload the sketch
      * Observe the readings on the Serial Monitor

    2022-03-06    Version 0.5.0   Tony Brophy   Initial sketch

*/

#include <GY26Compass.h>


// SOFTWARE SERIAL ---------------------------------------------------
// If NOT using Software Serial port, comment out the lines below
#include <SoftwareSerial.h>
const uint8_t softRxPin = 10;
const uint8_t softTxPin = 11;
SoftwareSerial compassSerial(softRxPin, softTxPin);
//---------------------------------------------------------------------


// HARDWARE SERIAL ---------------------------------------------------
// If NOT using Hardware Serial port, comment out the line below
// HardwareSerial& compassSerial = Serial2;
//---------------------------------------------------------------------


/*
 * This is the compass interface.
 * This implementation supports Serial communication.
 */
GY26_Uart_Compass compass(&compassSerial);

/*
 * Set this to the correct declination angle 
 * for your location
 */
float localDeclinationAngle = 0.0; 

float compassTemperature;
float compassAngle;

void setup(){
    Serial.begin(9600); // Start serial monitor
    compassSerial.begin(9600); // Start serial connectivity with compass
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
