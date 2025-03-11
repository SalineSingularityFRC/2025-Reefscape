/*
 * Teensy 4.0 Triple CAN Demo - Modified for Unique CAN Addresses per Message
 *
 * Sends the 8 rows from the 8x8 sensor array as 8 different CAN messages.
 * Each message contains 8 values (one per column in that row) and has a unique CAN ID.
 * Also prints the entire sensor array to the serial terminal.
 *
 * For use with:
 * http://skpang.co.uk/catalog/teensy-40-triple-can-board-include-teensy-40-p-1575.html
 *
 * can1 and can2 are CAN2.0B
 * can3 is CAN FD
 *
 * Ensure FlexCAN_T4 is installed:
 * https://github.com/tonton81/FlexCAN_T4
 *
 * www.skpang.co.uk October 2019 
 */

#include <FlexCAN_T4.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> // http://librarymanager/All#SparkFun_VL53L5CX

// CAN interface objects
FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> FD;  // CAN FD port (can3)
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;    // Standard CAN port (can2)
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;    // Standard CAN port (can1)

// Sensor objects
SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Structure to hold ranging data

int imageResolution = 0; // Should be 64 for an 8x8 sensor
int imageWidth = 0;      // Should be 8

int led = 13;
IntervalTimer timer;
uint8_t d = 0;

// This function sends one CAN message for a given row.
// Each row's data (8 sensor readings) is sent as an 8-byte CAN message.
// The message ID is computed uniquely for each row: 
// e.g., row 0 gets id 0x15555550, row 1 gets id 0x15555551, etc.
void sendRow(uint8_t row) {
  CAN_message_t msg;
  msg.id = 0x15555550 + row; // Unique CAN ID for each row
  msg.len = 8;
  msg.seq = 0;
  
  // For each column in the row, retrieve the sensor value.
  // The sensor data is stored in a 1D array (64 elements),
  // with row data starting at index (row * imageWidth).
  for (uint8_t col = 0; col < imageWidth; col++) {
    int index = row * imageWidth + col;
    int value = measurementData.distance_mm[index];
    // Scale the value down to 8 bits (shift right by 1 and clamp to 255).
    int temp = value >> 2;
    if (temp > 255) {
      temp = 255;
    }
    msg.buf[col] = (uint8_t)temp;
  }
  
  can1.write(msg);  // Send the message on can1
  
  Serial.print(row);
  Serial.print(": ");
  for (uint8_t col = 0; col < 8; col++) {
      Serial.print(msg.buf[col]);
      Serial.print(" ");
    }
  Serial.println();
}

// This function prints the entire 8x8 sensor array to the serial terminal.
void printSensorArray() {
  Serial.println("Sensor Array:");
  for (uint8_t row = 0; row < imageWidth; row++) {
    for (uint8_t col = 0; col < imageWidth; col++) {
      int index = row * imageWidth + col;
      Serial.print(measurementData.distance_mm[index]);
      Serial.print("\t");
    }
    Serial.println();
  }
  Serial.println();
}

void setup(void) {
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  Serial.begin(115200);
  delay(1000);
  Serial.println("Teensy 4.0 Triple CAN test - Row Mode with Unique Addresses");
  digitalWrite(led, LOW);

  // Initialize I²C for the sensor
  Wire.begin();
  Wire.setClock(400000);

  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  if (!myImager.begin()) {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1);
  }

  myImager.setResolution(8 * 8); // Use all 64 zones (8x8)
  myImager.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS);
  myImager.setRangingFrequency(15);

  imageResolution = myImager.getResolution(); // e.g., 64 for an 8x8 grid
  imageWidth = sqrt(imageResolution);           // e.g., 8

  myImager.startRanging();

  // Initialize CAN FD (can3)
  FD.begin();
  CANFD_timings_t config;
  config.clock = CLK_24MHz;
  config.baudrate =   1000000;  // 1000kbps arbitration rate
  config.baudrateFD = 2000000;  // 2000kbps data rate
  config.propdelay = 190;
  config.bus_length = 1;
  config.sample = 75;
  FD.setRegions(64);
  FD.setBaudRateAdvanced(config, 1, 1);
  FD.onReceive(canSniff);
  FD.setMBFilter(ACCEPT_ALL);
  FD.setMBFilter(MB13, 0x1);
  FD.setMBFilter(MB12, 0x1, 0x3);
  FD.setMBFilterRange(MB8, 0x1, 0x04);
  FD.enableMBInterrupt(MB8);
  FD.enableMBInterrupt(MB12);
  FD.enableMBInterrupt(MB13);
  FD.enhanceFilter(MB8);
  FD.enhanceFilter(MB10);
  FD.distribute();
  FD.mailboxStatus();

  // Initialize CAN2 (standard CAN)
  can2.begin();
  can2.setBaudRate(500000);  // 500kbps data rate
  can2.enableFIFO();
  can2.enableFIFOInterrupt();
  can2.onReceive(FIFO, canSniff20);
  can2.mailboxStatus();

  // Initialize CAN1 (standard CAN)
  can1.begin();
  can1.setBaudRate(1000000);  // 1Mbps data rate
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  // Optionally set up a callback for CAN1 if needed.
  can1.mailboxStatus();
}

void canSniff(const CANFD_message_t &msg) {
  Serial.print("ISR - MB ");
  Serial.print(msg.mb);
  Serial.print("  OVERRUN: ");
  Serial.print(msg.flags.overrun);
  Serial.print("  LEN: ");
  Serial.print(msg.len);
  Serial.print(" EXT: ");
  Serial.print(msg.flags.extended);
  Serial.print(" TS: ");
  Serial.print(msg.timestamp);
  Serial.print(" ID: ");
  Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for (uint8_t i = 0; i < msg.len; i++) {
    Serial.print(msg.buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void canSniff20(const CAN_message_t &msg) { // Global callback for standard CAN messages
  Serial.print("T4: ");
  Serial.print("MB ");
  Serial.print(msg.mb);
  Serial.print(" OVERRUN: ");
  Serial.print(msg.flags.overrun);
  Serial.print(" BUS ");
  Serial.print(msg.bus);
  Serial.print(" LEN: ");
  Serial.print(msg.len);
  Serial.print(" EXT: ");
  Serial.print(msg.flags.extended);
  Serial.print(" REMOTE: ");
  Serial.print(msg.flags.remote);
  Serial.print(" TS: ");
  Serial.print(msg.timestamp);
  Serial.print(" ID: ");
  Serial.print(msg.id, HEX);
  Serial.print(" IDHIT: ");
  Serial.print(msg.idhit);
  Serial.print(" Buffer: ");
  for (uint8_t i = 0; i < msg.len; i++) {
    Serial.print(msg.buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void loop() {
  // Check if sensor data is ready
  if (myImager.isDataReady()) {
    measurementData = VL53L5CX_ResultsData();
    if (myImager.getRangingData(&measurementData)) {
      // Print the entire sensor array to the terminal
      //printSensorArray();

      // For each row, send a CAN message with that row's data.
      for (uint8_t row = 0; row < imageWidth; row++) {
        sendRow(row);
      }
    }
  }

  // Process any CAN events on can1.
  can1.events();
  
  // Check for and print any incoming CAN messages on can1.
  CAN_message_t msg;
  if (can1.readMB(msg)) {
    Serial.print("MB: ");
    Serial.print(msg.mb);
    Serial.print("  OVERRUN: ");
    Serial.print(msg.flags.overrun);
    Serial.print("  ID: 0x");
    Serial.print(msg.id, HEX);
    Serial.print("  EXT: ");
    Serial.print(msg.flags.extended);
    Serial.print("  LEN: ");
    Serial.print(msg.len);
    Serial.print(" DATA: ");
    for (uint8_t i = 0; i < msg.len; i++) {
      Serial.print(msg.buf[i]);
      Serial.print(" ");
    }
    Serial.print("  TS: ");
    Serial.println(msg.timestamp);
  }
  
  // Short delay to pace the loop.
  delay(15);
}
