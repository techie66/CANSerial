# CANSerial

Use CAN bus as physical layer for serial/uart communication.
Aheres to Stream interface, so it can be used exactly like a regular Serial port. Supports multiple concurrent objects when used in cunjunction with included Buffered_MCP_CAN class. Designed to interface with https://github.com/bondus/CanSerial/

## History
One day I found myself needing an additional serial port on my Arduino nano. Because it's based on the atmega328p it only has one hardware port. Normally, I'd just use SoftwareSerial, but I was already using that for something else. I also happened to already have my device connected to a CAN bus and I was debugging the messages being sent out.

It then occured to me that instead of trying to make another SoftwareSerial and adding more wires to the mix (and another uart adapter on the computer), I could just pipe those debug messages over the CAN bus too.

After looking around online, I eventuially found https://github.com/bondus/CanSerial/ which claims to be obsolete. I took a look anyways and figured that it would work fine on the computer side of things. I just needed to write the Arduino side "client" code, and that's how we got here.

After some initial testing that worked really well it became apparent that the ability to have multiple objects on a device would be great and I still needed a way to still read the CAN bus from my project. This led to the creation of the Buffered_MCP_CAN class. It buffers received CAN frames and exposes the ability for a buffer-aware client to read them while keeping them in the buffer for other clients to read.

## ...Ok, but WHY?
In one word, convenience.
Imagine this example. You have a project based on an Arduino nano. It has a GPS module connected to SoftwareSerial and sends GPS data out over a CAN bus. It also has a bluetooth to serail module connected so it can be controlled via a phone app. As you work on this project, something isn't working. Normally you might start doing some `Serial.println("debug message");` at different lines in your code to see whats going on, but alas, you don't have a spare Serial port. 
With CANSerial, you just create a new CANSerial object and `println` your debug output there.
One other handy thing. By using CANSerial, you can have your debug messages always available on the bus without needing to physically connect more wires to your MCU. It's also trivial to keep debug messages separate from "real" communication data without extra wires.

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
  // Periodic calls to .available(), one of the .print() functions or .read() etc
  //   are necessary for the library to perform address negotiation and heartbeat
  CS.println("Hello");
  delay(1000);
)
```

---
##### Utility program
[CanSerial utility program](https://github.com/bondus/CanSerial/)
Clone the repository. Edit cansock.h Build it, run it.
It prints out handy messages when something connects and tells you the virtual tty device name to open.

#### Memory Usage
A quick note about memory usage.
Buffered_MCP_CAN buffers CAN frames. Each frame requires 13 bytes of RAM, so the buffer size is critical for RAM usage. By default it is set to 16 which seems to work well with a couple objects using it. Depending on your program structure, you may need to increase `_CAN_MAX_RX_BUF` to a larger value to prevent buffer rollovers where data gets lost because it is overwritten before it gets read. Be careful, when testing, I set `_CAN_MAX_RX_BUF` to 64 and RAM was almost full causing strange errors because local variables started getting written to wrong places. I recommend going as large as you can while keeping static RAM usage under about 80% to give the best chance of error-free operation.

## Issues
This library has been tested to work in several configurations. It probably has edge cases that completely break it. Future updates to Seeed_Arduino_CAN could also break it.
If you find an issue, [Report it](https://github.com/techie66/CANSerial/issues/new)

## Contributing
All pull requests welcome!

## License
Licensed under GPLv3, see LICENSE file for details.
