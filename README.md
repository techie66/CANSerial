# CANSerial

Use CAN bus as physical layer for serial/uart communication.
Aheres to Stream interface, so it can be used exactly like a regular Serial port. Supports multiple concurrent objects when used in cunjunction with included Buffered_MCP_CAN class. Designed to interface with https://github.com/bondus/CanSerial/

## History
One day I found myself needing an additional serial port on my Arduino nano. Because it's based on the atmega328p it only has one hardware port. Normally, I'd just use SoftwareSerial, but I was already using that for something else. I also happened to already have my device connected to a CAN bus and I was debugging the messages being sent out.

It then occured to me that instead of trying to make another SoftwareSerial and adding more wires to the mix (and another uart adapter on the computer), I could just pipe those debug messages over the CAN bus too.

After looking around online, I eventuially found https://github.com/bondus/CanSerial/ which claims to be obsolete. I took a look anyways and figured that it would work fine on the computer side of things. I just needed to write the Arduino side "client" code, and that's how we got here.

## Usage
This library is actually two libraries in one. CANSerial is the star of the show, but Buffered_MCP_CAN is in there too.
CANSerial can be used in one of two ways; either directly with an MCP_CAN object or using an intermediary Buffered_MCP_CAN.

Direct usage of MCP_CAN has the following problems:
 - Unable to have more than one CANSerial Object
 - Unable to read from the CAN bus with any other part of the program.
 
These problems are fixed by wrapping access to the CAN object in Buffered_MCP_CAN, examples for each are included in the `examples/` folder.

#### Requirements
##### Seeed_Arduino_CAN
I cannot possibly know what hardware you might be using and in what configuration, so I won't attempt to explain how to get your CAN interface working. This library is designed to work with Seeed_Arduino_CAN and either an mcp2515 or mcp2518fd.
You will probably have something like the following to have a working CAN interface:
```
// CAN Constants
const int SPI_CS_PIN = 10;
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
```
...
```
void setup() {
  // CAN Setup
  while (CAN_OK != CAN.begin(CAN_500KBPS,MCP_12MHz)) {             // init can bus : baudrate = 500k
    Serial.println("CAN ERROR");
    delay(100);
  }
}
```
##### Setup
To add CANSerial to the above, include the header:
```
#include <CANSerial.h>
```

---
Declare an object, passing in a "base address" and your CAN object:
```
CANSerial CS(0x6E0,CAN);
```
Or, if using Buffered_MCP_CAN:
```
Buffered_MCP_CAN bufCAN(CAN);
CANSerial CS(0x6E0,bufCAN);
```
Notice that `CAN` is passed to Buffered_MCP_CAN and then the Buffered_MCP_CAN object is passed to CANSerial.

---
The "base address" is what lets this client library and the PC server software mentioned above talk to each other. It needs to match `PKT_ID_UUID_FILTER` from `cansock.h` in the [CanSerial utility program](https://github.com/bondus/CanSerial/).

UUIDs for this are a maximum of 6 bytes, they need to be unique on your network for each device, not necessary to be pseudo-random.
In `setup()` add:
```
  static unsigned char UUID[_CS_UUID_SIZE] = {0xCE, 0xF2, 0x35, 0x2C, 0xE1, 0xB3};
  CS.begin(UUID,_CS_UUID_SIZE);
```

---
Finally, use it:
```
loop(
  CS.println("Hello");
  delay(1000);
)
```

## Issues
This library has been tested to work in several configurations. It probably has edge cases that completely break it. Future updates to Seeed_Arduino_CAN could also break it.
If you find an issue, [Report it](https://github.com/techie66/CANSerial/issues/new)

## Contributing
All pull requests welcome!

## License
Licensed under GPLv3, see LICENSE file for details.
