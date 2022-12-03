#ifndef FX29K_H
#define FX29K_H
#include <Arduino.h>
#include <Wire.h>

#define FX29K0 0x28
#define FX29K1 0x36
#define FX29K2 0x46
#define FX29K3 0x48
#define FX29K4 0x51

class FX29K{

  private:
    uint8_t _i2cAddr = FX29K0;
    uint8_t _range = 0010;
    uint16_t _tare = 0;
    TwoWire* _i2cPtr = &Wire;
  
  public:
    FX29K(uint8_t addr, uint8_t range);
    ~FX29K(void);

    void setWire(TwoWire* i2cPtr) {_i2cPtr = i2cPtr;}
   
    uint16_t tare(void);
    uint16_t tare(uint16_t samples);
  
    uint16_t getTare(void);
  
    void requestMeasurement(void);
  
    uint16_t getRawBridgeData(void);
  
    float getPounds(void);
    float getKilograms(void);
    float getGrams(void);
  
    void write(TwoWire* i2cPtr, uint8_t i2cAddr, uint8_t* arr, uint8_t byteCount);
    void read(TwoWire* i2cPtr, uint8_t i2cAddr, uint8_t* arr, uint8_t byteCount);
  
};

#endif
