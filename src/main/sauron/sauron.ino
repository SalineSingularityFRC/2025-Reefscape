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
  // with row data starting at index (row * imageWidth)
  for (uint8_t col = 0; col < imageWidth; col++) {
    int index = row * imageWidth + col;
    int16_t value = filter_distance_mm(measurementData.distance_mm[index], measurementData.target_status[index]);
    // Scale the value down to 8 bits (shift right by 1 and clamp to 255).
    int16_t temp = value >> 2;
    if (temp > 255) {
      temp = 255;
    }
    msg.buf[col] = (uint8_t)temp;
  }
  
  can1.write(msg);  // Send the message on can1
  
  // if (0) {
  Serial.print(row);
  Serial.print(": ");
  for (uint8_t col = 0; col < 8; col++) {
      Serial.print((msg.buf[col] << 2) + 1);
      Serial.print(" ");
    }
  Serial.println();
  // }
}

/**
Target status Description
0 Ranging data are not updated
1 Signal rate too low on SPAD array
2 Target phase
3 Sigma estimator too high
4 Target consistency failed
5 Range valid
6 Wrap around not performed (Typically the first range)
7 Rate consistency failed
8 Signal rate too low for the current target
9 Range valid with large pulse (may be due to a merged target)
10 Range valid, but no target detected at previous range
11 Measurement consistency failed
12 Target blurred by another one, due to sharpener
13 Target detected but inconsistent data. Frequently happens for secondary targets.
255 No target detected (only if number of target detected is enabled)
*/
int16_t filter_distance_mm(int16_t distance_mm, int target_status) {
  // if (target_status != 5 && target_status != 9 && target_status != 6 && target_status != 10 && target_status != 13) {
  //   return 0;
  // }
  
  // if (distance_mm < 30) {
  //   return 0;
  // }

  return distance_mm;
}

// This function prints the entire 8x8 sensor array to the serial terminal.
void printSensorArray() {
  Serial.println("Sensor Array:");
  for (uint8_t row = 0; row < imageWidth; row++) {
    for (uint8_t col = 0; col < imageWidth; col++) {
      int index = row * imageWidth + col;
      Serial.print(measurementData.target_status[index]);
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
  Serial.println("5066 Singularity - Eye of Sauron");
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
  // myImager.setSharpenerPercent(100);
  myImager.setPowerMode(SF_VL53L5CX_POWER_MODE::WAKEUP);
  myImager.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS);
  myImager.setTargetOrder(SF_VL53L5CX_TARGET_ORDER::STRONGEST);
  myImager.setWireMaxPacketSize(128);
  myImager.setRangingFrequency(10);

  imageResolution = myImager.getResolution(); // e.g., 64 for an 8x8 grid
  imageWidth = sqrt(imageResolution);           // e.g., 8

  myImager.startRanging();

  // Initialize CAN1 (standard CAN)
  can1.begin();
  can1.setBaudRate(1000000);  // 1Mbps data rate
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  // Optionally set up a callback for CAN1 if needed.
  can1.mailboxStatus();
}

void loop() {
  // Check if sensor data is ready
  if (myImager.isDataReady()) {
    measurementData = VL53L5CX_ResultsData();
    if (myImager.getRangingData(&measurementData)) {
      // Print the entire sensor array to the terminal
      // printSensorArray();

      // For each row, send a CAN message with that row's data.
      for (uint8_t row = 0; row < imageWidth; row++) {
        sendRow(row);
      }
    }
  }

  // Process any CAN events on can1.
  can1.events();
}
