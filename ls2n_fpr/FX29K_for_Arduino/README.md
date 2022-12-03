# FX29K_for_Arduino
An Arduino library for the FX29K digital load cells.

## Compatibility
Supported models:
* `FX29Kx-xxxx-xxxx-L` (Tested on an FX29K0-040B-0010-L)
* `FX29Jx-xxxx-xxxx-L` (Not tested)

FX292 and FX293 analog load cells are *NOT* supported.

## Quick start
### Minimal setup
```ruby
   RED ---------- VCC (2.7-5.5V)
YELLOW ---------- SCL
 WHITE ---------- SDA
 BLACK ---------- GND
```
### Installation
* Download this repository as a `.zip` file.
* In Arduino IDE, navigate to `Sketch` > `Include library` > `Add .zip library`.
* Select the downloaded `.zip` file.
### Main Program
#### Include
* In your `.ino` file, include [`FX29K.h`](./FX29K.h):
```ruby
#include "FX29K.h"
```
#### Declaration
* Declare your `FX29K` object globally, with constructors:
```ruby  
FX29K::FX29K(uint8_t addr, uint8_t range);
FX29K::FX29K(uint8_t addr, uint8_t range, TwoWire* i2cPtr);
```
* For example, for `FX29K0-040B-0010-L`:
```ruby
FX29K fx29k(FX29K0, 0010, &Wire);
```

#### Initialization
* Tare load cell with:
```ruby
void FX29K::tare(void);
void FX29K::tare(uint16_t samples);
```

#### Measurement
* Read raw wheatstone bridge data with:
```ruby
uint16_t FX29K::getRawBridgeData(void);
```
* Get one measurement:
```ruby
float FX29K::getPounds(void);
float FX29K::getKilograms(void);
float FX29K::getGrams(void);
```

### Misc
* Refer to [`./FX29K_for_Arduino.ino`](./FX29_for_Arduino.ino) and [datasheets](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FFX29%7FA5%7Fpdf%7FEnglish%7FENG_DS_FX29_A5.pdf%7FCAT-FSE0006) for further information.
