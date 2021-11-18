/*
 * This sketch is for a Lilygo TTGO T-Display which uses an Espressif ESP32.
 * It sends the 5-byte wakupstring to a NCR18650BD Okai ES200G scooter battery pack
 * and interprets the 36-byte data packet returned from the battery pack.
 * TODO: Display battery stats on the display, transmit packet over UDP.
 *
 * Connections: Use a DC to DC converter that has a common negative that converts 42+ volts DC to 5V DC.
 * The negative of the battery pack is the 0V voltage reference for the serial pins.  From the DC-DC
 * converter connect the + and - to the t-display's 5v and GND.
 * On the connector from the battery pack, the blue wire is serial in, so it will connect to the UART out pin of the t-display, pin 25.
 * The green wire is serial out from the battery; it is open-collector so it needs a pull-up to 3.3v.
 * Attach the green wire to the t-display's UART in, pin 26 and also connect a 1kΩ resistor between pin 26 and the 3.3v pin.
 * 
 * To compile for T-Display: set this in boards manager URL in preferences of Arduino IDE:
 * https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
 * Select from boards manager: esp32 > ESP32 Dev Module
 * CPU frequency: 240 MHz, Events run on: Core 1, Flash Frequency: 80 MHz, Flash mode: QIO and DIO both seem to work, Flash size: 4MB
 * Arduino runs on: Core 1, PSRAM: Disabled, Partition Scheme: Huge APP (3 MB No OTA / 1 MB SPIFFS), Upload speed: 921600
 */

#include <WiFi.h>
#include <WiFiUdp.h>

const uint8_t wakeupstring[] = {
  0x3A, 0x13, 0x01, 0x16, 0x79 };

/* // An example:
const uint8_t testcode[] = {
  0x3A, 0x16, 0x20, 0x02, 0x00, 0x34,
  0x64, 0x1B, 0x1B, 0x1B, 0x18, 0x37,
  0x00, 0x00, 0x0F, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x40, 0x83, 0x8F, 0x00,
  0x00, 0xE5, 0xFF, 0xFF, 0xFF, 0x5E,
  0x0E, 0x56, 0x0E, 0x52, 0x2C, 0x13 };
*/

unsigned char crc_bits(unsigned char data)
{   // https://stackoverflow.com/a/36505819/11087027
    unsigned char crc = 0;
    if(data & 1)     crc ^= 0x5e;
    if(data & 2)     crc ^= 0xbc;
    if(data & 4)     crc ^= 0x61;
    if(data & 8)     crc ^= 0xc2;
    if(data & 0x10)  crc ^= 0x9d;
    if(data & 0x20)  crc ^= 0x23;
    if(data & 0x40)  crc ^= 0x46;
    if(data & 0x80)  crc ^= 0x8c;
    return crc;
}

uint8_t dallas_crc8(unsigned char * data, unsigned int size)
{   // https://stackoverflow.com/a/36505819/11087027
    unsigned char crc = 0;
    for ( unsigned int i = 0; i < size; ++i )
    {    crc = crc_bits(data[i] ^ crc); }
    return crc;
}

float voltage(uint8_t * data)
{
  float volts;
  int msb = (int) data[22] * 256;
  int lsb = (int) data[21];
  volts = (msb + lsb) / 1000.0F;
  return volts;
}

float current(uint8_t * data)
{
  float amps;
  short ma = (((short)data[26]) << 8) | data[25];
  // int msb = (signed int) data[26] * 256;
  // int lsb = (int) data[25];
  amps = ma / 1000.0F;
  return amps;
}

float highCellV(uint8_t * data)
{
  float volts;
  int msb = (int) data[30] * 256;
  int lsb = (int) data[29];
  volts = (msb + lsb) / 1000.0F;
  return volts;  
}

float lowCellV(uint8_t * data)
{
  float volts;
  int msb = (int) data[32] * 256;
  int lsb = (int) data[31];
  volts = (msb + lsb) / 1000.0F;
  return volts;
}

uint8_t percentCharged(uint8_t * data)
{
  return data[5];
}

struct stats
{
  // array declared inside structure
  // https://www.journaldev.com/39081/return-array-in-c-plus-plus-function
  uint8_t temperature[4];
};

struct stats batt(uint8_t * data)
{
  struct stats array;
  uint8_t offset = 7; // load t array with elements of data array starting with element 7
  uint8_t items = 4;
  for (uint8_t i = offset; i < (offset + items); i++)
  { array.temperature[i - offset] = data[i]; }
  return array;
}

float power(float voltage, float current)
{
  return voltage * current;
}

float lowToHighCellDifference(float lowCellV, float highCellV)
{
  return highCellV - lowCellV;
}

void clearIncomingBuffer()
{
  while (Serial2.available() > 0)
  {
    int incomingByte = Serial2.read();
    if (incomingByte < 0) { break; }
  }
}

uint8_t size;
long timeout = 1000;
unsigned long desiredCycleTime = 5000;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1,25,26); // RX on pin 25 TX on pin 26
  delay(200);
  Serial.printf("\nStarting.\n");
  Serial2.setTimeout(timeout);
  clearIncomingBuffer();
}

void loop() {
  unsigned long cycleStartTime = millis();
  clearIncomingBuffer();
  size = Serial2.write(wakeupstring, sizeof(wakeupstring) );  // async write begins with this line
  if (size == sizeof(wakeupstring) )
  { Serial.printf("Sent %d byte wakeup string\n", size); }
  else
  { Serial.printf("Error: expected to send %d bytes but Serial2.write reported %d bytes sent.\n", sizeof(wakeupstring), size); }
  uint8_t incomingBuffer [36];
  // Serial.printf("%d\n", sizeof(incomingBuffer));
  size_t bufferSize = Serial2.readBytes(incomingBuffer, sizeof(incomingBuffer));
  clearIncomingBuffer();
  if (bufferSize != 36)
  {
    if (bufferSize > 0)
    {
      Serial.printf("Error: Only received %d out of 36 bytes in %.3f second timeout.\n", bufferSize, (timeout / 1000.0f) );
    }
    else
    {
      Serial.printf("Nothing received in %.3f second timeout.\n", (timeout / 1000.0f) );
    }
  }
  else // the buffer size is correct
  {
    struct stats stat;  
    size = sizeof(incomingBuffer)-1;
    if (incomingBuffer[size] != dallas_crc8(incomingBuffer, size) )
    {
      Serial.printf("**** CRC mismatch. data: ");
      for (uint8_t it=0; it <= size; it++)
      { Serial.printf("%x ", incomingBuffer[it]); }
      Serial.printf("(%x) ****\n", incomingBuffer[size], dallas_crc8(incomingBuffer, size) );
    }
    else
    {
      for (uint8_t it=0; it <= size; it++)
      { Serial.printf("%x ", incomingBuffer[it]); }
      Serial.printf("(%x)\n", incomingBuffer[size], dallas_crc8(incomingBuffer, size) );
      stat = batt(incomingBuffer);
      float volts = voltage(incomingBuffer);
      float amps = current(incomingBuffer);
      Serial.printf("%.3f Volts\n", volts);
      Serial.printf("%.3f Amps\n", amps);
      Serial.printf("%.4f Watts\n", power(volts, amps));
      Serial.printf("Temperatures: ");
      for (uint8_t i=0; i < 4; i++)
      { Serial.printf("%d°C ", stat.temperature[i]); }
      Serial.printf("\n%d%% charged", percentCharged(incomingBuffer) );
      if (amps < 0)
      { Serial.printf(", discharging"); }
      else if (amps > 0)
      { Serial.printf(", charging"); }
      float lowCell = lowCellV(incomingBuffer);
      float highCell = highCellV(incomingBuffer);
      Serial.printf("\nLow cell: %.3fV, High cell: %.3fV, Difference: %.3fV\n", lowCell, highCell, lowToHighCellDifference(lowCell, highCell) );
    }
  }
  
  /*
  // test crc routines
  uint8_t wakeupVariable[sizeof(wakeupstring)];
  memcpy(wakeupVariable, wakeupstring, (sizeof(uint8_t)*(sizeof(wakeupstring)) ) ); // copy wakeupstring const array to wakeupVariable array
  size = sizeof(wakeupVariable)-1;
  // for (uint8_t it=0; it<size; it++)
  // { Serial.printf("%x ", wakeupVariable[it]); }
  Serial.printf("\nWakeup variable - Size: %d, crc expected: %#x calculated: %#x\n", size, wakeupVariable[size], dallas_crc8(wakeupVariable, size));

  uint8_t testVariable[sizeof(testcode)];
  memcpy(testVariable, testcode, (sizeof(uint8_t)*(sizeof(testcode)) ) ); // copy testcode const array to testVariable array
  size = sizeof(testVariable)-1;
  // for (uint8_t it=0; it<size; it++)
  // { Serial.printf("%x ", testVariable[it]); }
  Serial.printf("Test code variable - Size: %d, crc expected: %#x calculated: %#x\n", size, testVariable[size], dallas_crc8(testVariable, size)); */
  
  // now wait as long as needed to restart the cycle almost exactly 5 seconds after the previous one started.
  unsigned long cycleEndTime = millis();
  Serial.printf("Ended at %.3f seconds\n", (cycleEndTime - cycleStartTime) / 1000.0F );
  long desiredEndTime = cycleStartTime + desiredCycleTime; // this will begin to malfunction after being online for 7.1 weeks or half of that?
  long timeToWait = desiredEndTime - cycleEndTime;
  // Serial.printf("cycleStartTime:  %d ms,\n cycleStopTime:  %d ms,\ndesiredEndTime: %d ms,\ntimeToWait: %d ms\n", cycleStartTime, cycleEndTime, desiredEndTime, timeToWait);

  if (timeToWait > 0)
  {
    while (millis() - desiredEndTime > 1)
    { 
      long now = millis();
      if ( now - desiredEndTime > 0) { delay(now - desiredEndTime); }     
    }
  }
  Serial.printf("\nRepeating cycle %.3f seconds after starting it\n", (millis() - cycleStartTime) / 1000.0 );
  
}