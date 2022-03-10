/**
 * @file GY26_UART_Compass.h
 * @author Tony Brophy
 * @brief Facilitates Serial Communication with GY-26 Compass Module
 * @version 0.5
 * @date 2022-03-06
 *
 */

#ifndef GY26_UART_COMPASS_H
#define GY26_UART_COMPASS_H

#include <Arduino.h>
#include "GY26_Common.h"

// Default sync delay
#define UART_SYNC_DELAY_MS 40
// Calibration sync delay
#define UART_SYNC_DELAY_CAL_MS 200

/**
 * @brief Magic number to signify comms issue
 *
 */
#define UART_ERROR_CODE 2000

/**
 * @brief Communicate with the GY-26 Compass module over serial UART.
 *
 */
class GY26_Uart_Compass
{
public:
    /**
     * @brief Constructor
     * @param serial Serial interface object.
     *
     * @note Serial interface can use HardwareSerial or SoftwareSerial implementation
     * @note Preferably use harware serial (Serial1, Serial2 ...) for reliability
     */
    GY26_Uart_Compass(Stream *serial);

    /**
     * @brief Get the temperature measured on the GY-26
     *
     * @return Temperature on GY-26 in Celsius to one decimal place
     *
     * @note If there are comms errors, the return value is 0.0
     */
    float getTemperatureCelsius();

    /**
     * @brief Get the compass angle measured on the GY-26
     *
     * @return Compass angle in degrees, to one decimal place. This measurement
     * is the angle between the line of direction of the arrow printed on the
     * GY-26 and detected North, as measured clockwise. Values are always
     * positive and range from 0.0 to 359.9
     *
     * @note If there are comms errors, the return value is 0.0
     * @note The angle returned from this method is compensated according
     * to the delination angle
     * @see GY26_Uart_Compass::setDeclinationAngle
     */
    float getCompassAngle();

    /**
     * @brief Set the Declination Angle. This should be set to compensate for the
     * deviation between magnetic North and geographical North. Search online for
     * the correct declination angle according to your location
     *
     * @param declinationAngle The required declination Angle. One decimal place is
     * recognized, and valid range is 0.0 - 359.9. Negative angles up to -179.9 are
     * also accepted. However, for negative angles, getDeclinationAngle()
     * subsequently returns the appropriate complementary angle.
     *
     * @return true if the declination angle was set successfully
     * @return false if an error was detected
     *
     * @note If an invalid angle is supplied ( < -180.0 OR >= 360.0 ), the Declination
     * angle is reset to 0.0
     */
    bool setDeclinationAngle(float declinationAngle);

    /**
     * @brief Get most recent Calibration Level value
     *
     * @return Calibration Level. Values range from 0 to 9 where 9
     * indicates the most complete calibration. If the
     * startCalibration() method has not been previously called,
     * this method returns 0.0
     */
    uint8_t getCalibrationLevel();

    /**
     * @brief Start Calibration Mode. The GY-26 LED should go on
     *
     * @return true if the Calibration starts successfully
     * @return false if an error is detected
     */
    bool startCalibration();

    /**
     * @brief End Calibration Mode. The GY-26 LED should go off
     *
     * @return true if the Calibration ends successfully
     * @return false if an error is detected
     */
    bool endCalibration();

    /**
     * @brief Set a new I2C device address on the GY-26 module.
     *
     * @param newAddressForI2C The new I2C device address
     * @return true if the new address is in range, and command is successful
     * @return false if the new address is not valid or the command failed
     *
     * @note Invalid I2C addresses are ignored. Left-justified values are accepted.
     * @note If successful, the new I2C address is used in future communication
     * by the current object, but newly instantiated GY_I2C_Compass objects
     * need to be properly initialized to use the updated I2C address as expected
     * by the GY-26 module
     */
    bool setAddressForI2C(uint8_t newAddressForI2C);

    /**
     * @brief Perform factory reset. This clears internal calibration parameters
     * and sets the Declination Angle to 0.0. I2C address is not affected.
     *
     * @return true if Factory Reset is successful
     * @return false if an error is detected
     */
    bool doFactoryReset();

protected:
    /**
     * @brief Send a command to the device
     *
     * @param commandByte The command byte to send to the device
     * @param syncDelayMs Time to wait between command and response
     * @return The response value as a number of the form ###.#
     *
     * @note If an error is detected UART_ERROR_CODE is returned
     * @see UART_ERROR_CODE
     */
    float sendCommandSerial(uint8_t commandByte, unsigned long syncDelayMs = UART_SYNC_DELAY_MS);

    /**
     * @brief Read and validate the response from the device.
     *
     * @return Numerical value of the form ###.#
     *
     *
     * @note If an error is detected, UART_ERROR_CODE is returned
     * @see UART_ERROR_CODE
     */
    float receiveResponseSerial();

    /**
     * @brief Check the validity of the supplied I2C address
     *
     * @param address The proposed I2C address to validate
     * @return The right-justified representation of the address
     * being validated. If the supplied value is invalid, return
     * value is 0.
     */
    uint8_t validateI2CAddress(uint8_t address);

private:
    Stream *_serialStream;
    int8_t _cachedCalibrationLevel = 0;
};

#endif // GY26_UART_COMPASS_H