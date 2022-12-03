#include "FX29K.h"
#include <Wire.h>

/**
   @brief Construct a FX29K object instance.

   Check your load cell's part number:

    FX29Kx-xxxx-xxxx-x
    |____|      |___||_ unit
 	 addr       range

   @param addr FX29 I2C address.
   @param range FX29 load range.
*/
FX29K::FX29K(uint8_t addr, uint8_t range) {
  _i2cAddr = addr;
  _range = range;
}

/**
   @brief Destructor.
*/
FX29K::~FX29K(void) {}

/**
   @brief Tare the scale with the average of 10 samples.
*/
uint16_t FX29K::tare(void) {
  requestMeasurement();
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 10; i++) {
    sum += getRawBridgeData();
    delay(10);
  }
  _tare = sum / 10;
  return _tare;
}

/**
   @brief Tare the scale with the average of n samples.
   @param samples Number of samples to take.
*/
uint16_t FX29K::tare(uint16_t samples) {
  requestMeasurement();
  uint32_t sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += getRawBridgeData();
  }
  _tare = sum / samples;
  return _tare;
}

/**
  @brief Get the raw bridge data at tare weight.
  @return Bridge data as a 14-bit number.
 */
uint16_t FX29K::getTare(void){
  return _tare;
}

/**
   @brief Device to start a measuremnet.
*/
void FX29K::requestMeasurement(void) {
  uint8_t temp = 0;
  write(_i2cPtr, _i2cAddr, &temp, 0);
}

/**
   @brief Device to return raw bridge data.
   @return Bridge data as a 14-bit number.
*/
uint16_t FX29K::getRawBridgeData(void) {
  uint8_t bridgeData[2] = {0};
  read(_i2cPtr, _i2cAddr, bridgeData, 2);
  return (bridgeData[0] << 8 | bridgeData[1]) & 0x3fff;
}

/**
   @brief Get weight in pounds (lbs).
   @return Weight, in pounds.
*/
float FX29K::getPounds(void) {
  uint16_t bridgeData = getRawBridgeData();
  uint32_t net = 0;
  int8_t sign = 1;
  if (bridgeData >= _tare) {
    net = bridgeData - _tare;
    sign = 1;
  }
  else {
    net = _tare - bridgeData;
    sign = -1;
  }
  return (net * _range / 14000.0) * sign;
}

/**
   @brief Get weight in kilograms (kg).
   @return Weight, in kilograms.
*/
float FX29K::getKilograms(void) {
  return getPounds() * 0.453592;
}

/**
   @brief Get weight in grams (g).
   @return Weight, in grams.
*/
float FX29K::getGrams(void) {
  return getPounds() * 453.592;
}

/**
   @brief I2C write function.
   @param i2cPtr TwoWire object pointer.
   @param i2cAddr I2C device address to write to.
   @param arr Array containing data bytes to write.
   @param byteCount Number of bytes to write.
*/
void FX29K::write(TwoWire* i2cPtr, uint8_t i2cAddr, uint8_t* arr, uint8_t byteCount) {
  i2cPtr->beginTransmission(i2cAddr);
  for (uint8_t i = 0; i < byteCount; i++) {
    i2cPtr->write(*(arr + i));
  }
  i2cPtr->endTransmission();
}

/**
   @brief I2C read function.
   @param i2cPtr TwoWire object pointer.
   @param i2cAddr I2C device address to read from.
   @param arr Array to write fetched data bytes to.
   @param byteCount Number of bytes to read.
*/
void FX29K::read(TwoWire* i2cPtr, uint8_t i2cAddr, uint8_t* arr, uint8_t byteCount) {
  i2cPtr->requestFrom(i2cAddr, byteCount);
  for (uint8_t i = 0; i < byteCount; i++) {
    *(arr + i) = i2cPtr->read();
  }
  i2cPtr->endTransmission();
}
