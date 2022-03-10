/**
 * @file GY26_UART_Compass.cpp
 * @author Tony Brophy
 * @brief Facilitates Serial Communication with GY-26 Compass Module
 * @version 0.5
 * @date 2022-03-06
 *
 */

#include <Arduino.h>
#include "GY26_Uart_Compass.h"

/*
 * Uncomment line below to display diagnostics on the specified serial port.
 * The Serial port must be initialized with Serial.begin(baudRate) before use.
 * It cannot be the same Serial port used to pass commands to the compass
 */
//#define SERIAL_LOG Serial

#define LOG_TAG "[GY-26] " // Leading text for any diagnostics output

GY26_Uart_Compass::GY26_Uart_Compass(Stream *serial)
{
  _serialStream = serial;
}

float GY26_Uart_Compass::getTemperatureCelsius()
{
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.println("----- getTemperatureCelsius() -----");
#endif
  float responseValue = sendCommandSerial(CMD_GET_TEMPERATURE);
  if (responseValue == UART_ERROR_CODE)
    return 0.0;
  return responseValue;
}

float GY26_Uart_Compass::getCompassAngle()
{
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.println("----- getCompassAngle() -----");
#endif
  float responseValue = sendCommandSerial(CMD_GET_COMPASS);
  if (responseValue == UART_ERROR_CODE)
    return 0.0;
  return responseValue;
}

bool GY26_Uart_Compass::setDeclinationAngle(float declinationAngle)
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
  float responseCode = -1;
  sendCommandSerial(REG_DECLINATION);        // No response is expected
  responseCode = sendCommandSerial(hiByte);  // 0.0 is expected
  sendCommandSerial(REG_DECLINATION + 1);    // No response is expected
  responseCode += sendCommandSerial(loByte); // 0.0 is expected
  return (responseCode == 0.0);
}

uint8_t GY26_Uart_Compass::getCalibrationLevel()
{
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.println("----- getCalibrationLevel() -----");
#endif
  return _cachedCalibrationLevel;
}

bool GY26_Uart_Compass::startCalibration()
{
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.println("----- startCalibration() -----");
#endif
  if (sendCommandSerial(CMD_START_CALIBRATION, UART_SYNC_DELAY_CAL_MS) == 0.0)
  {
    // Invalidate the cached value
    _cachedCalibrationLevel = 0;
    return true;
  }
  return false;
}

bool GY26_Uart_Compass::endCalibration()
{
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.println("----- endCalibration() -----");
#endif
  float responseCode = sendCommandSerial(CMD_END_CALIBRATION, UART_SYNC_DELAY_CAL_MS);
  if (responseCode != UART_ERROR_CODE)
  {
    _cachedCalibrationLevel = uint8_t(responseCode * 10);
    delay(UART_SYNC_DELAY_MS);
    return true;
  }
  return false;
}

bool GY26_Uart_Compass::setAddressForI2C(uint8_t newAddressForI2C)
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
  // Reset codes return "1000.0" which is ignored
  sendCommandSerial(CMD_RESET_CODE_1);
  sendCommandSerial(CMD_RESET_CODE_2);
  sendCommandSerial(CMD_RESET_CODE_3);
  // Address must be left-justified in register
  float responseCode = sendCommandSerial(rightJustifiedAddress << 1);
  return (responseCode == 0.0);
}

bool GY26_Uart_Compass::doFactoryReset()
{
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.println("----- doFactoryReset() -----");
#endif
  // Reset codes return "1000.0" which is ignored
  sendCommandSerial(CMD_RESET_CODE_1);
  sendCommandSerial(CMD_RESET_CODE_2);
  sendCommandSerial(CMD_RESET_CODE_3);
  float responseCode = sendCommandSerial(CMD_FACTORY_RESET);
  return (responseCode == 0.0);
}

// protected ---------------------------------------------

float GY26_Uart_Compass::sendCommandSerial(uint8_t dataByte, unsigned long syncDelayMs /*=UART_SYNC_DELAY_MS*/)
{
#ifdef SERIAL_LOG
  SERIAL_LOG.print(LOG_TAG);
  SERIAL_LOG.print("Sending Serial Byte 0x");
  SERIAL_LOG.println(dataByte, HEX);
#endif
  _serialStream->write(dataByte);
  delay(syncDelayMs);
  return receiveResponseSerial();
}

float GY26_Uart_Compass::receiveResponseSerial()
{
  uint8_t checkSum = 0;
  char receivedString[8];
  uint8_t byteNo = 0;
  while (_serialStream->available())
  {
    receivedString[byteNo] = _serialStream->read();
#ifdef SERIAL_LOG
    SERIAL_LOG.print(LOG_TAG);
    SERIAL_LOG.print("Received Serial Byte 0x");
    SERIAL_LOG.println(receivedString[byteNo], HEX);
#endif
    if (byteNo < 7)
      checkSum += receivedString[byteNo];
    byteNo++;
  }
  // Signify that response string was not received
  if (byteNo != 8)
  {
#ifdef SERIAL_LOG
    SERIAL_LOG.print(LOG_TAG);
    SERIAL_LOG.println("Incomplete or no response from Serial Port");
#endif
    return UART_ERROR_CODE;
  }
  if (receivedString[7] != checkSum)
  {
#ifdef SERIAL_LOG
    SERIAL_LOG.print(LOG_TAG);
    SERIAL_LOG.print("!!!!! COMMS ERROR: CHECKSUM NOT VALID");
#endif
    return UART_ERROR_CODE;
  }
  float measurement = 0.0;
  // Convert from ascii "###.#" to float
  measurement += (receivedString[2] - 48) * 100;
  measurement += (receivedString[3] - 48) * 10;
  measurement += (receivedString[4] - 48);
  measurement += float(receivedString[6] - 48) / 10;
  return measurement;
}

uint8_t GY26_Uart_Compass::validateI2CAddress(uint8_t address)
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