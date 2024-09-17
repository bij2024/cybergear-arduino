#include <mcp_can.h>
#include <SPI.h>
#include "xiaomi_cybergear_driver.h"

// Pins used to connect to MCP2515 CAN bus transceiver:
#define CAN_CS_PIN 10 // Chip Select pin for MCP2515
#define CAN_INT_PIN 2 // Interrupt pin for MCP2515

MCP_CAN CAN(CAN_CS_PIN);  // Set CS pin

// Interval:
#define TRANSMIT_RATE_MS 1000
#define POLLING_RATE_MS 1000

static bool driver_installed = false;
unsigned long previousMillis = 0;  // will store last time a message was sent

uint8_t CYBERGEAR_CAN_ID = 0x7F;
uint8_t MASTER_CAN_ID = 0x00;
XiaomiCyberGearDriver cybergear = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID, MASTER_CAN_ID);

void setup() {
  Serial.begin(115200);

  // Initialize MCP2515 CAN interface
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP2515...");
    while (1);
  }
  CAN.setMode(MCP_NORMAL);  // Set operation mode to normal

  // Set up the motor parameters
  cybergear.init_motor(MODE_POSITION);
  cybergear.set_limit_speed(10.0f); // set maximum speed of the motor
  cybergear.set_limit_current(5.0); // current limit for faster operation
  cybergear.enable_motor();         // turn on the motor
  cybergear.set_position_ref(0.0);  // set initial rotor position

  driver_installed = true;
}

static void handle_rx_message(unsigned long id, uint8_t len, uint8_t* buf) {
  // Check if the received message is for the CyberGear
  if (((id & 0xFF00) >> 8) == CYBERGEAR_CAN_ID) {
    twai_message_t message;
    message.identifier = id;
    message.data_length_code = len;
    memcpy(message.data, buf, len);

    cybergear.process_message(message);
  }

  // Debug print received message
  Serial.print("ID: ");
  Serial.print(id, HEX);
  Serial.print(" Data: ");
  for (int i = 0; i < len; i++) {
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void check_alerts() {
  // Check for received CAN messages
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    unsigned long id;
    uint8_t len;
    uint8_t buf[8];

    // Read CAN message
    CAN.readMsgBuf(&id, &len, buf);
    handle_rx_message(id, len, buf);
  }
}

void loop() {
  if (!driver_installed) {
    delay(1000);
    return;
  }

  delay(1000);

  static float pos = 0.0;
  static float inc_val = 1;
  pos += inc_val;
  if (pos > 10.0) inc_val = -1;
  if (pos < -10.0) inc_val = 1;
  cybergear.set_position_ref(pos);

  check_alerts();

  XiaomiCyberGearStatus cybergear_status = cybergear.get_status();
  Serial.printf("POS:%f V:%f T:%f temp:%d\n", cybergear_status.position, cybergear_status.speed, cybergear_status.torque, cybergear_status.temperature);

  // send a request to the cybergear to receive motor status (position, speed, torque, temperature)
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
    previousMillis = currentMillis;
    cybergear.request_status();
  }
}
