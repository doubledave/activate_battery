/*
 * This sketch is for a Lilygo TTGO T-Display which uses an Espressif ESP32.
 * It sends the 5-byte wakupstring to a NCR18650BD Okai ES200G scooter battery pack
 * and interprets the 36-byte data packet returned from the battery pack.
 * TODO: Display battery stats on the display, transmit packet over UDP.
 * https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/examples/WiFiUDPClient/WiFiUDPClient.ino
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
#include "time.h"

const char networkName[] = "ssid";
const char networkPswd[] = "pass";
const char udpAddress[]  = "192.168.1.255";
// const char ntpServer[]= "192.168.1.1";
const char ntpServer[]   = "pool.ntp.org";
const float gmtOffset_hr = -6.0f;
const float daylightOffset_hr = 1.0f;
// const char // I forgot what I was going to put here
const int udpPort = 5606;
//Are we currently connected?
boolean connected = false;
//The udp library class
WiFiUDP udp;
const uint32_t epochShift = 2208988800; // change by number of seconds in 70 years (1900 based timestamp to 1970 based timestamp)  int(365.25 * 70) * 24 * 60 * 60

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

uint32_t swapped(uint32_t num)
{ // https://stackoverflow.com/a/2182184/11087027
  return ((num>>24)&0xff) | // move byte 3 to byte 0
                    ((num<<8)&0xff0000) | // move byte 1 to byte 2
                    ((num>>8)&0xff00) | // move byte 2 to byte 1
                    ((num<<24)&0xff000000); // byte 0 to byte 3
}


struct tm timeinfo;
time_t now;

uint8_t size;
long timeout = 1000;
unsigned long desiredCycleTime = 5000;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1,25,26); // RX on pin 25 TX on pin 26
  delay(200);
  Serial.printf("\nStarting, connecting to WiFi.\n");
  Serial2.setTimeout(timeout);

  //Connect to the WiFi network
  // connectToWiFi(networkName, networkPswd);
  if (sizeof(networkPswd) > 0)
  { WiFi.begin(networkName, networkPswd); }
  else
  { WiFi.begin(networkName); }
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nConnected to wifi. IP: ");
  Serial.println(WiFi.localIP());
  Serial.printf("Setting the time from NTP server\n");
  configTime((gmtOffset_hr * 3600L), (daylightOffset_hr * (int)3600), ntpServer);
  if(!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  else
  {
    Serial.printf("Time set. ");
    char timeStringBuff[50];
    strftime(timeStringBuff, sizeof(timeStringBuff), "%A, %B %d %Y %H:%M:%S", &timeinfo);
    Serial.printf("%s\n", timeStringBuff);
    //uint32_t unixTime = timeinfo.now();
    //Serial.println("\nUnix timestamp: %ld", unixTime);
  }
  Serial.printf("\nOpening reference to UDP port - ");
  udp.begin(udpPort);
  Serial.printf("Done.\n");
  connected = true;
  
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
      if(connected)
      { udp.beginPacket(udpAddress,udpPort);
        uint8_t timestamp [sizeof(now)];
        time(&now);
        now += epochShift;
        now = swapped(now);
        memcpy(&timestamp, &now, sizeof(now));
        udp.write(timestamp, sizeof(timestamp)); 
        udp.write(incomingBuffer, sizeof(incomingBuffer));
        now = swapped(now); now -= epochShift; // change it back
        udp.printf(" %lu", now);
        udp.endPacket();
        // Serial.printf("sent %d bytes UDP\n", sizeof(incomingBuffer));
      }
    }
  }

  char timeStringBuff[50];
  
  time(&now);
  now += epochShift; // change by 70 years (1900 based timestamp to 1970 based timestamp)
  getLocalTime(&timeinfo);
  
  Serial.printf("timestamp: %lu\n", now);
  // Serial.printf("Epochtime: %x\n", getTime());
  strftime(timeStringBuff, sizeof(timeStringBuff) - 1, "%A, %B %d %Y %H:%M:%S", &timeinfo);
  Serial.printf("%s - min sec %d %d\n", timeStringBuff, timeinfo.tm_min, timeinfo.tm_sec);
  
  
  
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

/*
void connectToWifi(const char * ssid, const char * pwd)
{
  Serial.println("Connecting to WiFi network: " + String(ssid));
  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);
  
  //Initiate connection
  if (sizeof(pwd) > 0)
  { WiFi.begin(ssid, pwd); }
  else
  { WiFi.begin(ssid); }
  Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          break;
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
      default: break;
    }
} */
