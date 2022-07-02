#include <mcp2515_can.h>
#include <CANSerial.h>

// CAN Constants
const int SPI_CS_PIN = 10;
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin

CANSerial CS(0x6E0,CAN);

void setup() {
  Serial.begin(115200);
  Serial.println("Startup initiated!");

  // CAN Setup
  while (CAN_OK != CAN.begin(CAN_500KBPS,MCP_12MHz)) {             // init can bus : baudrate = 500k
	  Serial.println("CAN ERROR");
      delay(100);
  }

  // Uncomment the following line if hardware is setup to enable transceiver from MCP PIO
  CAN.mcpPinMode(MCP_RX0BF,MCP_PIN_OUT);  

	static unsigned char UUID[_CS_UUID_SIZE] = {0xCE, 0xF2, 0x35, 0x2C, 0xE1, 0xB3};
  CS.begin(UUID,_CS_UUID_SIZE);
}

void loop() {
  CS.println("Hello");
  delay(1000);
}
