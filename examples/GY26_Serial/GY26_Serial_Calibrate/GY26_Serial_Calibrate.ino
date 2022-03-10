/*
    GY26_Serial_Calibrate

    Purpose:
      Using a Serial connection, perform the calibration routine on the GY-26
      Compass module. This procedure is required to improve accuracy whenever
      the location of the compass module has changed.

    Wiring Connections as follows:
      * Connect Power (3.3V/5.0V) and GND from the microcontroller board to the GY-26 board
      * Make a crossover connection with serial lines (RX and TX) from the
        microcontroller board to the GY-26 board (i.e. connect RX to TX, and
        connect TX to RX). The RX and TX pins numbers will be particular to the
        spare Serial port (Serial1, Serial2 etc) that you choose to use, as well as
        the microcontroller board being used. For Arduino boards, see
        https://www.arduino.cc/reference/en/language/functions/communication/serial/ .
        Do not use the first serial port (Serial) as this is used for program upload 
        and monitor. If you wish to test on an Uno, or any other board with
        no spare Serial hardware, you can use the Software Serial implementation and
        connect using I/O pins as commented in the provided code below.


    Instructions:
      * Ensure the unused code for either Hardware Serial or Software Serial
        is commented out below as appropriate. One of these must be commented.
      * Open Serial Monitor
      * Upload the sketch
      * Follow instructions shown on Serial Monitor
      * Improve Calibration Level by editting the
        calibrationPeriod variable and running the
        calibration procedure again.

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
//HardwareSerial& compassSerial = Serial2;
//---------------------------------------------------------------------

/*
 * This is the compass interface.
 * This implementation supports Serial communication.
 */
GY26_Uart_Compass compass(&compassSerial);

/*
 * How many seconds to spend performing calibration.
 * For best results, extend to 60 seconds or more
 * to achieve a calibration value of 9.
 */
unsigned long calibrationPeriod = 30;

void setup()
{
    Serial.begin(9600);        // Start serial monitor
    compassSerial.begin(9600); // Start serial connectivity with compass
    delay(1000);
    Serial.println("When the green LED lights, keep the GY-26 board");
    Serial.println("horizontal while rotating slowly through at least");
    Serial.println("one full revolution until the LED turns off ... ");
    delay(5000);
    compass.startCalibration();
    unsigned long delayMs = calibrationPeriod * 1000;
    delay(delayMs);
    compass.endCalibration();
    uint8_t calibrationLevel = compass.getCalibrationLevel();
    Serial.println();
    Serial.print("Calibration completed to level: ");
    Serial.println(calibrationLevel);
    Serial.println();
}

void loop()
{
    // No code
}
