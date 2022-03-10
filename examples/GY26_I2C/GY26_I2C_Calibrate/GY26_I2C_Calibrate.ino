/*
    GY26_I2C_Calibrate

    Purpose:
      Using I2C, perform the calibration routine on the GY-26 Compass
      module. This procedure is required to improve accuracy whenever
      the location of the compass module has changed.

    Wiring Connections as follows:
      * Connect Power (3.3V/5.0V) and GND from the microcontroller board to the GY-26 board
      * Connect I2C lines (SDA and SCL) from the microcontroller board to the
        GY-26 board. Use the appropriate hardware I2C pins for your particular
        case - For Arduino boards see https://www.arduino.cc/en/reference/wire .

    Instructions:
      * Open Serial Monitor
      * Upload the sketch
      * Follow instructions shown on Serial Monitor
      * Improve Calibration Level by editing the
        calibrationPeriod variable and running the 
        calibration procedure again.

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
 * How many seconds to spend performing calibration.
 * For best results, extend to 60 seconds or more
 * to achieve a calibration value of 9.
 */
unsigned long calibrationPeriod = 30;

void setup()
{
  Serial.begin(9600); // Start serial monitor
  Wire.begin();       // Start I2C connectivity
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
