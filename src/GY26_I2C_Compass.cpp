/**
 * @file GY26_I2C_Compass.cpp
 * @author Tony Brophy
 * @brief Facilitates I2C Communication with GY-26 Compass Module
 * @version 0.5
 * @date 2022-03-06
 *
 */

#include <Arduino.h>
#include "GY26_I2C_Compass.h"

/*
 * Uncomment line below to display diagnostics on the specified serial port.
 * The Serial port must be initialized with Serial.begin(baudRate) before use.
 */
//#define SERIAL_LOG Serial

#define LOG_TAG "[GY-26] " // Leading text for any diagnostics output

GY26_I2C_Compass::GY26_I2C_Compass(uint8_t addressI2C /*= I2C_DEFAULT_ADDRESS*/, TwoWire *wire /*= &Wire*/)
{
  _addressI2C = validateI2CAddress(addressI2C);
  if (_addressI2C == 0)
    _addressI2C = I2C_DEFAULT_ADDRESS;
  _wire = wire;
}

float GY26_I2C_Compass::getTemperatureCelsius()
{
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.println("----- getTemperatureCelsius() -----");
#endif
  if (sendCommandI2C(CMD_GET_TEMPERATURE))
    return (float)readIntegerI2C(REG_TEMPERATURE) / 10;
  return 0.0;
}

float GY26_I2C_Compass::getCompassAngle()
{
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.println("----- getCompassAngle() -----");
#endif
  if (sendCommandI2C(CMD_GET_COMPASS))
    return (float)readIntegerI2C(REG_COMPASS) / 10;
  return 0.0;
}

float GY26_I2C_Compass::getDeclinationAngle()
{
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.println("----- getDeclinationAngle() -----");
#endif
  return (float)readIntegerI2C(REG_DECLINATION) / 10;
}

bool GY26_I2C_Compass::setDeclinationAngle(float declinationAngle)
{
  if ((declinationAngle < -180.0) || (declinationAngle >= 360.0))
    declinationAngle = 0.0;
  if (declinationAngle < 0)
    declinationAngle += 360.0;
  // 0.06 Offset provided to facilitate rounding
  uint16_t integerValue = (uint16_t)((declinationAngle + 0.06) * 10);
  declinationAngle = float(integerValue) / 10;
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.println("----- setDeclinationAngle() -----");
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.print("New declination angle will be set to ");
  SERIAL_LOG.println(declinationAngle);
#endif
  // Integer Value stored is 10 * (Declination Angle to one decimal place)
  uint8_t loByte = (uint8_t)integerValue;
  uint8_t hiByte = (uint8_t)(integerValue >> 8);
  if (!setRegisterI2C(REG_DECLINATION, hiByte) || !setRegisterI2C(REG_DECLINATION + 1, loByte))
  {
#ifdef SERIAL_LOG
    SERIAL_LOG.print(LOG_TAG);
    SERIAL_LOG.println("!!!!! COMMS ERROR: Cannot write declination angle");
#endif
    return false;
  }
  return true;
}

uint8_t GY26_I2C_Compass::getCalibrationLevel()
{
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.println("----- getCalibrationLevel() -----");
#endif
  if (_cachedCalibrationLevel < 0)
    refreshCalibrationLevel();
  return _cachedCalibrationLevel;
}

bool GY26_I2C_Compass::startCalibration()
{
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.println("----- startCalibration() -----");
#endif
  if (sendCommandI2C(CMD_START_CALIBRATION))
  {
    // Invalidate the cached value
    _cachedCalibrationLevel = -1;
    return true;
  }
  return false;
}

bool GY26_I2C_Compass::endCalibration()
{
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.println("----- endCalibration() -----");
#endif
  bool success = sendCommandI2C(CMD_END_CALIBRATION);
  // This suppplemental delay IS required for stability
  delay(I2C_SYNC_DELAY_MS);
  return success;
}

uint8_t GY26_I2C_Compass::getAddressForI2C()
{
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.println("----- getAddressForI2C() -----");
#endif
  return _addressI2C;
}

bool GY26_I2C_Compass::setAddressForI2C(uint8_t newAddressForI2C)
{
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.println("----- setAddressForI2C() -----");
#endif
  uint8_t rightJustifiedAddress = validateI2CAddress(newAddressForI2C);
  if (rightJustifiedAddress == 0)
    return false;
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.print("New right-justified I2C Address is being set to 0x");
  SERIAL_LOG.println(rightJustifiedAddress, HEX);
#endif
  bool success = sendCommandI2C(CMD_RESET_CODE_1);
  success = success && sendCommandI2C(CMD_RESET_CODE_2);
  success = success && sendCommandI2C(CMD_RESET_CODE_3);
  // Must shift left to get accepted value for register
  success = success && sendCommandI2C(rightJustifiedAddress << 1);
  if (success)
    _addressI2C = rightJustifiedAddress;
  return success;
}

bool GY26_I2C_Compass::doFactoryReset()
{
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.println("----- doFactoryReset() -----");
#endif
  bool success = sendCommandI2C(CMD_RESET_CODE_1);
  success = success && sendCommandI2C(CMD_RESET_CODE_2);
  success = success && sendCommandI2C(CMD_RESET_CODE_3);
  success = success && sendCommandI2C(CMD_FACTORY_RESET);
  return success;
}

// protected ---------------------------------------------

void GY26_I2C_Compass::refreshCalibrationLevel()
{
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.println("Refreshing cached value for Calibration Level");
#endif
  _cachedCalibrationLevel = (uint8_t)(readIntegerI2C(REG_CALIBRATION_LEVEL, true));
}

uint16_t GY26_I2C_Compass::readIntegerI2C(uint8_t registerAddress, bool returnSingleByte /*= false*/)
{
  uint8_t byteCount = returnSingleByte ? 1 : 2;
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.print("Reading ");
  SERIAL_LOG.print(byteCount);
  SERIAL_LOG.print(" byte(s) from address 0x");
  SERIAL_LOG.println(registerAddress, HEX);
#endif
  _wire->beginTransmission(_addressI2C);
  _wire->write(registerAddress);
  uint8_t errorCode = _wire->endTransmission(false);
  if (errorCode > 0)
  {
#ifdef SERIAL_LOG
    SERIAL_LOG.print(LOG_TAG);
    SERIAL_LOG.print("!!!!! COMMS ERROR: ");
    SERIAL_LOG.print("Transmission Exit Code ");
    SERIAL_LOG.println(errorCode);
#endif
    return 0;
  }
  _wire->requestFrom(_addressI2C, byteCount);
  if (_wire->available() != (byteCount))
  {
#ifdef SERIAL_LOG
    SERIAL_LOG.print(LOG_TAG);
    SERIAL_LOG.println("!!!!! ERROR: Cannot read byte(s) from I2C");
#endif
    return 0;
  }
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.print("Receiving byte(s) [ ");
#endif
  uint8_t byteArray[2] = {0x00, 0x00};
  for (uint8_t i = 0; i < byteCount; i++)
  {
    byteArray[i] = _wire->read();
#ifdef SERIAL_LOG
    SERIAL_LOG.print("0x");
    SERIAL_LOG.print(byteArray[i], HEX);
    SERIAL_LOG.print(" ");
#endif
  }
#ifdef SERIAL_LOG
  SERIAL_LOG.println("]");
#endif
  uint16_t result = byteArray[0];
  if (!returnSingleByte)
  {
    result <<= 8;
    result += byteArray[1];
  }
  return result;
}

bool GY26_I2C_Compass::sendCommandI2C(uint8_t commandByte)
{
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.print("Preparing to send I2C COMMAND 0x00 0x");
  SERIAL_LOG.println(commandByte, HEX);
#endif
  uint8_t byteArray[2] = {REG_COMMAND, commandByte};
  return sendBytesI2C(byteArray, 2);
}

bool GY26_I2C_Compass::setRegisterI2C(uint8_t registerAddress, uint8_t registerValue)
{
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.print("Setting register address 0x");
  SERIAL_LOG.print(registerAddress, HEX);
  SERIAL_LOG.print(" value to 0x");
  SERIAL_LOG.println(registerValue, HEX);
#endif
  uint8_t byteArray[2] = {registerAddress, registerValue};
  return sendBytesI2C(byteArray, 2);
}

bool GY26_I2C_Compass::sendBytesI2C(uint8_t *dataBytes, uint8_t byteCount /*= 1*/)
{
  _wire->beginTransmission(_addressI2C);
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.print("Transmitting bytes: [ ");
#endif
  for (uint8_t i = 0; i < byteCount; i++)
  {
    _wire->write(dataBytes[i]);
#ifdef SERIAL_LOG
    SERIAL_LOG.print("0x");
    SERIAL_LOG.print(dataBytes[i], HEX);
    SERIAL_LOG.print(" ");
#endif
  }
  uint8_t errorCode = _wire->endTransmission();
#ifdef SERIAL_LOG
  SERIAL_LOG.println("]");
  SERIAL_LOG.print(LOG_TAG);
  if (errorCode > 0)
    SERIAL_LOG.print("!!!!! COMMS ERROR: ");
  SERIAL_LOG.print("Transmission Exit Code ");
  SERIAL_LOG.println(errorCode);
#endif
  // Delay required for reliability
  delay(I2C_SYNC_DELAY_MS);
  return (errorCode == 0);
}

uint8_t GY26_I2C_Compass::validateI2CAddress(uint8_t address)
{
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.println("----- validateI2CAddress() -----");
#endif
  uint8_t newAddress = address;
  if ((newAddress >= I2C_JUSTIFYLEFT_LO) && (!(newAddress % 2)))
    newAddress >>= 1;
  bool validAddress = ((newAddress >= I2C_JUSTIFYRIGHT_LO) && (newAddress <= I2C_JUSTIFYRIGHT_HI));
  if (validAddress)
    return newAddress;
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.print("Proposed I2C Address INVALID. Rejected 0x");
  SERIAL_LOG.println(address, HEX);
#endif
  return 0;
}