/*
 * Teensy 4.0 Triple CAN Demo
 * 
 * For use with:
 * http://skpang.co.uk/catalog/teensy-40-triple-can-board-include-teensy-40-p-1575.html
 * 
 * can1 and can2 are CAN2.0B
 * can3 is CAN FD
 * 
 * Ensure FlexCAN_T4 is installed first
 * https://github.com/tonton81/FlexCAN_T4
 * 
 * www.skpang.co.uk October 2019 
 * 
 */
#include <FlexCAN_T4.h>

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> // http://librarymanager/All#SparkFun_VL53L5CX


FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> FD;  // can3 port
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;  // can2 port
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;  // can1 port 

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Structure to hold ranging data


int imageResolution = 0; // Total number of distance values (64 for 8x8)
int imageWidth = 0;      // Image width (8 for 8x8)

int led = 13;
IntervalTimer timer;
uint8_t d=0;


/*
  Returns a pointer to a static array containing the minimum distance value
  for each printed column.
  
  Parameters:
    - data: The VL53L5CX measurement data containing the distance array.
    - width: The number of columns/rows (e.g., 8).
  
  Note: Printed column 0 corresponds to sensor column (width - 1)
        and printed column (width - 1) corresponds to sensor column 0.
*/

int* getMinValuesPerColumn(const VL53L5CX_ResultsData &data, int width)
{
  static int minValues[8]; // Static so the array persists after the function returns
  for (int printedCol = 0; printedCol < width; printedCol++)
  {
    int sensorCol = width - 1 - printedCol;  // Map printed column to sensor column
    int minVal = 0xFFFF; // Start with a large number

    // Loop over each row in this column
    for (int row = 0; row < width; row++)
    {
      int index = row * width + sensorCol; // Calculate the index in the 1D array
      int distance = data.distance_mm[index];
      if (distance < minVal)
      {
        minVal = distance;
      }
    }
    minValues[printedCol] = minVal;
  }
  return minValues;
}


/*
  Returns a pointer to a static array containing the average distance value
  for each printed column.
  
  Parameters:
    - data: The VL53L5CX measurement data containing the distance array.
    - width: The number of columns/rows (e.g., 8).
  
  Note: Printed column 0 corresponds to sensor column (width - 1)
        and printed column (width - 1) corresponds to sensor column 0.
*/
int* getAvgValuesPerColumn(const VL53L5CX_ResultsData &data, int width)
{
  static int avgValues[8]; // Static so the array persists after the function returns
  for (int printedCol = 0; printedCol < width; printedCol++)
  {
    int sensorCol = width - 1 - printedCol;
    long sum = 0;
    // Loop over each row in this column
    for (int row = 0; row < width; row++)
    {
      int index = row * width + sensorCol;
      sum += data.distance_mm[index];
    }
    avgValues[printedCol] = (int)(sum / (float)width);
  }
  return avgValues;
}


void setup(void) {
  pinMode(led, OUTPUT);   
  digitalWrite(led,HIGH);
  Serial.begin(115200); 
  delay(1000);
  Serial.println("Teensy 4.0 Triple CAN test");
  digitalWrite(led,LOW);


  // Init distanceArray stuff
  Wire.begin();
  Wire.setClock(400000);

  
  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  if (myImager.begin() == false)
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1);
  }

  
  myImager.setResolution(8 * 8); // Use all 64 zones (8x8)
  myImager.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS);
  myImager.setRangingFrequency(15);
  
  imageResolution = myImager.getResolution(); // e.g., 64 for 8x8
  imageWidth = sqrt(imageResolution);           // e.g., 8

  myImager.startRanging();

//

  FD.begin();
  CANFD_timings_t config;
  config.clock = CLK_24MHz;
  config.baudrate =   1000000;       // 1000kbps arbitration rate
  config.baudrateFD = 2000000;      // 2000kbps data rate
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
  
  

  can2.begin();
  can2.setBaudRate(500000);       // 500kbps data rate
  can2.enableFIFO();
  can2.enableFIFOInterrupt();
  can2.onReceive(FIFO, canSniff20);
  can2.mailboxStatus();
  
  can1.begin();
  can1.setBaudRate(1000000);     // 1mbps data rate
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  //can1.onReceive(FIFO, canSniff20);
  can1.mailboxStatus();

  //timer.begin(sendframe, 500000); // Send frame every 500ms 
}

void sendframe(uint32_t id, int* arr)
{
  Serial.println("sendFrame");
  CAN_message_t msg2;
  msg2.id = id;
  
  
  for ( uint8_t i = 0; i < 8; i++ ) {
    int temp = arr[i] >> 1;
    
    msg2.buf[i] = (uint8_t) (temp>255?255:temp);
  }
  
  //msg2.buf[0] = d++;
  msg2.len = 8;
  msg2.seq = 1;
  Serial.println(can1.write(msg2)); // write to can1

  // msg2.id = 0x402;
  // msg2.buf[1] = d++;
  // can1.write(msg2);       // write to can1
  
  // CANFD_message_t msg;
  // msg.len = 64;
  // msg.id = 0x4fd;
  // msg.seq = 1;
  // for ( uint8_t i = 0; i < 64; i++ ) {
  //   msg.buf[i] = i + 1;
  // }
  // msg.buf[0] = d;
  // FD.write( msg);         // write to can3

}


void canSniff(const CANFD_message_t &msg) {
  Serial.print("ISR - MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}

void canSniff20(const CAN_message_t &msg) { // global callback
  Serial.print("T4: ");
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print(" OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print(" BUS "); Serial.print(msg.bus);
  Serial.print(" LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" REMOTE: "); Serial.print(msg.flags.remote);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" IDHIT: "); Serial.print(msg.idhit);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}



void loop() {
  int* minArray = NULL;
  int* avgArray = NULL; 
  CAN_message_t msg;
  if (myImager.isDataReady()) {
    if (myImager.getRangingData(&measurementData)) {
      // Get the arrays from the two functions
      minArray = getMinValuesPerColumn(measurementData, imageWidth);
      avgArray = getAvgValuesPerColumn(measurementData, imageWidth);

      // Print the minimum distances per column
      Serial.print("M,");
      for (int i = 0; i < imageWidth; i++) {
        Serial.print(minArray[i]);
        if (i < imageWidth - 1)
          Serial.print(", ");
      }
      Serial.println("");

      // Print the average distances per column
      Serial.print("A,");
      for (int i = 0; i < imageWidth; i++) {
        Serial.print(avgArray[i]);
        if (i < imageWidth - 1)
          Serial.print(", ");
      }
      Serial.println("");
    }
  }

  // can2.events();
  can1.events();

  if (can1.readMB(msg)) {
      Serial.print("MB: "); Serial.print(msg.mb);
      Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
      Serial.print("  ID: 0x"); Serial.print(msg.id, HEX );
      Serial.print("  EXT: "); Serial.print(msg.flags.extended );
      Serial.print("  LEN: "); Serial.print(msg.len);
      Serial.print(" DATA: ");
      for ( uint8_t i = 0; i <msg.len ; i++ ) {
        Serial.print(msg.buf[i]); Serial.print(" ");
      }
      Serial.print("  TS: "); Serial.println(msg.timestamp);
  }
  uint32_t idMin = 0x15555555;
  if (minArray != NULL) {
    sendframe(idMin, minArray);
  }
  // it wouldnt work without this
  delay(15);

  
  uint32_t idAvg = 0x15555554;
  if (avgArray != NULL){
  sendframe(idAvg, avgArray);
  }
}



