/**
 * @file GY26_I2C_Compass.h
 * @author Tony Brophy
 * @brief Facilitates I2C Communication with GY-26 Compass Module
 * @version 0.5
 * @date 2022-03-06
 *
 */

#ifndef GY26_I2C_COMPASS_H
#define GY26_I2C_COMPASS_H

#include <Arduino.h>
#include <Wire.h>
#include "GY26_Common.h"

// I2C_SYNC_DELAY_MS -
// This value works. Can be reduced if required
#define I2C_SYNC_DELAY_MS 55

#define I2C_DEFAULT_ADDRESS 0x70
#define REG_COMMAND 0x00
#define REG_COMPASS 0x01
#define REG_TEMPERATURE 0x05
#define REG_CALIBRATION_LEVEL 0x07

/**
 * @brief Communicate with the GY-26 Compass module over I2C.
 *
 */
class GY26_I2C_Compass
{
public:
    /**
     * @brief Constructor
     * @param addressI2C I2C address of GY-26 device. Defaults to 0x70.
     * @param wire TwoWire I2C bus interface. Defaults to global Wire object.
     *
     * @note Wire should be initialized with Wire.begin() before use.
     * @note If supplied, the I2C address value may be passed in left-adjusted
     * or right-adjusted form. The acceptable values are
     *
     * @note LEFT-JUSTIFIED:  0x70, 0x71, 0x72 ... 0x7F
     * @note RIGHT-JUSTIFIED: 0xE0, 0xE2, 0xE4 ... 0xFE
     *
     * @note An invalid address results in the default address 0x70 being used
     */
    GY26_I2C_Compass(uint8_t addressI2C = I2C_DEFAULT_ADDRESS, TwoWire *wire = &Wire);

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
     * @see GY26_I2C_Compass::setDeclinationAngle
     */
    float getCompassAngle();

    /**
     * @brief Get the current declination angle
     *
     * @return Current Declination Angle, to one decimal place.
     * This value is always positive in the range 0.0 to 359.9
     *
     */
    float getDeclinationAngle();

    /**
     * @brief Set the Declination Angle. This should be set to compensate for the
     * deviation between magnetic North and geographical North. Search online for
     * the correct declination angle according to your location
     *
     * @param declinationAngle The required declination Angle. One decimal place is
     * recognized, and valid range is 0.0 to 359.9. Negative angles up to -179.9 are
     * also accepted. However, for negative angles, getDeclinationAngle()
     * subsequently returns the appropriate complementary angle.
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
     * indicates the most complete calibration.
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
     * @brief Get I2C address currently used to communicate with GY-26
     *
     * @return Right-justified I2C address, range 0x70 - 0x7F
     */
    uint8_t getAddressForI2C();

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
     * @brief Update cached value for calibration level
     *
     */
    void refreshCalibrationLevel();

    /**
     * @brief Get one or two bytes from register
     *
     * @param registerAddress Start address to read data
     * @param returnSingleByte true to return single right-justified byte. false to return a two
     * byte integer MSB first. Default is false.
     *
     * @return 16-bit or zero-padded 8-bit data from register
     *
     * @note returns 0 if an error is detected
     */
    uint16_t readIntegerI2C(uint8_t registerAddress, bool returnSingleByte = false);

    /**
     * @brief Send a recognized command to the module
     *
     * @param commandByte The byte code representing the desired command
     * @return true if the command is sent successfully
     * @return false if an error is detected
     */
    bool sendCommandI2C(uint8_t commandByte);

    /**
     * @brief Set the value of a specified register address
     *
     * @param registerAddress The register address to update
     * @param registerValue The value to place in the specified register
     * @return true if the register address is updated successfully
     * @return false if an error is detected
     */
    bool setRegisterI2C(uint8_t registerAddress, uint8_t registerValue);

    /**
     * @brief Perform I2C communication and send specified bytes to the device
     *
     * @param dataBytes Array of byte to be sent to the device
     * @param byteCount Number of bytes to be sent to the device
     * @return true if the data is sent successfully
     * @return false if an error is detected
     */
    bool sendBytesI2C(uint8_t *dataBytes, uint8_t byteCount = 1);

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
    uint8_t _addressI2C = I2C_DEFAULT_ADDRESS;
    TwoWire *_wire;
    int8_t _cachedCalibrationLevel = -1;
};

#endif // GY26_I2C_COMPASS_H