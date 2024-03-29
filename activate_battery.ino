/*
 * This sketch is for a Lilygo TTGO T-Display which uses an Espressif ESP32.
 * It sends the 5-byte wakupstring to a NCR18650BD Okai ES200G scooter battery pack
 * and interprets the 36-byte data packet returned from the battery pack.
 * TODO: Display battery stats on the display.
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

#include <TFT_eSPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "time.h"

const char networkName[] = "ssid";
const char networkPswd[] = "pass";
const char ntpServer1[]  = "north-america.pool.ntp.org";
// const char ntpServer2[]  = "time.nist.gov"; // backup server
const float gmtOffset_hr = -6.0f;
const float daylightOffset_hr = 1.0f;
const int udpPort = 5606;
char udpAddress[16];        //  This variable gets filled in as the WiFi subnet's broadcast address.


TFT_eSPI tft = TFT_eSPI();

uint8_t mac[6];
//Are we currently connected?
boolean connected = false;
//The udp library class
WiFiUDP udp;
const uint32_t epochShift = 2208988800; // change by number of seconds in 70 years (1900 based timestamp to 1970 based timestamp)
                                        // 60 seconds * 60 minutes * 24 hours * int(365.25 days * 70 years)

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

uint8_t rotation = 3;   // change this value for your desired rotation (0 - 3)
uint8_t fontNum = 4;    // Only font numbers 2,4,6,7 are valid. Font 6 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : . - a p m
uint8_t thisFontHeight; // Font 7 is a 7 segment font and only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : .
uint16_t width;
uint16_t height;

uint16_t rgb(uint8_t r, uint8_t g, uint8_t b)
{ // convert three 8-bit RGB levels to a 16 bit color value
  // choose the 3 rgb values from https://html-color.codes/
  r = r & 0xF8; // rgb565 red uses 5 bits
  g = g & 0xFC; // rgb565 green uses 6 bits
  b = b & 0xF8; // rgb565 blue uses 5 bits
  return (r << 8) | (g << 3) | (b >> 3);  // bit shifting and combing into 16 bits
}

void labelButton1(char* string, uint16_t txtbgcolor)
{ uint16_t txtX;
  uint16_t txtY;
  uint16_t thisTextWidth = tft.textWidth(string, fontNum);
  // Serial.printf("rotation: %d, width: %d, height: %d, bgcolor: %X\n", rotation, width, height, txtbgcolor);
  // Serial.printf("font height: %d, text width: %d, text: \"%s\"\n", thisFontHeight, thisTextWidth, string);
  switch (rotation)
  { 
    case 0: // buttons on bottom, button1 is the left one
    txtX = 2;
    txtY = height - 4 - (thisFontHeight);
    tft.fillRect(txtX - 2, txtY - 2, thisTextWidth + 6, thisFontHeight + 4, txtbgcolor);
    break;
    case 1: // buttons on right, button1 is the bottom one
    txtX = width - 4 - thisTextWidth;
    txtY = height - 4 - thisFontHeight;
    tft.fillRect( txtX - 4, txtY - 2, thisTextWidth + 6, thisFontHeight + 4, txtbgcolor);
    // Serial.printf("txtX: %d, txtY: %d\n", txtX, txtY);
    break;
    case 2: // buttons on top, button1 is the right side one
    txtX = width - 4 - thisTextWidth;
    txtY = 2;
    tft.fillRect( txtX - 4, txtY - 2, thisTextWidth + 6, thisFontHeight + 4, txtbgcolor);
    break;
    case 3: // buttons on left; button1 is the top one
    txtX = 2;
    txtY = 2;
    tft.fillRect( txtX - 2, txtY - 2, thisTextWidth + 6, thisFontHeight + 4, txtbgcolor);
    break;
    default:
    Serial.printf("Invalid rotation number selected.  Needs to be either 0, 1, 2, or 3.");  
  }
  tft.drawString(string, txtX, txtY);
}
void labelButton2(char* string, uint16_t txtbgcolor)
{ uint16_t txtX;
  uint16_t txtY;
  uint16_t thisTextWidth = tft.textWidth(string, fontNum);
  switch (rotation)
  { 
    case 0: // buttons on bottom, button2 is the right side one
    txtX = width - 4 - thisTextWidth;
    txtY = height - 4 - thisFontHeight;
    tft.fillRect(txtX - 4, txtY - 2, thisTextWidth + 6, thisFontHeight + 4, txtbgcolor);
    break;
    case 1: // buttons on right, button2 is the top one
    txtX = width - 4 - thisTextWidth;
    txtY = 2;
    tft.fillRect( txtX - 4, txtY - 2, thisTextWidth + 6, thisFontHeight + 4, txtbgcolor);
    break;
    case 2: // buttons on top, button2 is the left one
    txtX = 2;
    txtY = 2;
    tft.fillRect( txtX - 2, txtY - 2, thisTextWidth + 6, thisFontHeight + 4, txtbgcolor);
    break;
    case 3: // buttons on left; button2 is the bottom one
    txtX = 2;
    txtY = height - 4 - thisFontHeight;
    tft.fillRect( txtX - 2, txtY - 4, thisTextWidth + 6, thisFontHeight + 4, txtbgcolor);
    break;
    default:
    Serial.printf("Invalid rotation number selected.  Needs to be either 0, 1, 2, or 3.");  
  }
  tft.drawString(string, txtX, txtY);
}

uint16_t bgcolor    = rgb(128, 128, 128); //choose your screen background color. See the rgb function above for more info.
uint16_t txtbgcolor = rgb(170, 0,   170); // button indicator background color
uint16_t txtfgcolor = rgb(0,   0,   0  ); // text foreground color

char button1Label[] = "Button1";
char button2Label[] = "Rotate";

void printCentered(char* msg, uint8_t lineNum)
{
  String msgString = msg;
  uint16_t thisTextWidth;
  // Serial.printf("debug: checking msg width. font: %d msg: \"%s\" width: ", fontNum, msg); delay(25);
  thisTextWidth = tft.textWidth(msg, fontNum);
  // Serial.printf("%d pixels\n", thisTextWidth); delay(25);
  while (tft.textWidth(msg, fontNum) > width)
  { Serial.printf("Too wide for screen: \"%s\"\n", msg);
    // Serial.printf("%d characters, dropping character: ", strlen(msg));
    // Serial.printf("%c\n", msg[strlen(msg) - 1]);
    msgString[strlen(msg) - 1] = '\0'; msg = (char* )msgString.c_str(); }
  thisTextWidth = tft.textWidth(msg, fontNum);
  uint16_t txtX = (width / 2) - (thisTextWidth / 2);
  if (rotation > 0 && rotation < 3) { lineNum++; }
  uint16_t txtY = lineNum * thisFontHeight;
  if (txtY + thisFontHeight < height)
  {
    tft.fillRect(0, txtY, width - 1, thisFontHeight, bgcolor);
    tft.drawString(msg, txtX, txtY);
    //labelButton1(button1Label, txtbgcolor); // Don't have a use for button1 yet
    //labelButton2(button2Label, txtbgcolor); // Don't want to label button2 yet; it covers up the battery percentage
  }
  else { Serial.printf("Tried to print beyond bottom of screen: \"%s\"\n", msg); }
  
}

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

struct tm timeinfo;
time_t now;

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
  return ((num>>24)&      0xff) | // move byte 3 to byte 0
         ((num<<8 )&  0xff0000) | // move byte 1 to byte 2
         ((num>>8 )&    0xff00) | // move byte 2 to byte 1
         ((num<<24)&0xff000000);  //      byte 0 to byte 3
}

long smallerof(long a, long b)
{
  if (a > b) { return b; }
  return a;
}

String broadcastAddr(IPAddress localIP, IPAddress subnetMask)
{
  uint32_t ip;
  ip =  localIP[0] * 0x1000000;
  ip += localIP[1] *   0x10000;
  ip += localIP[2] *     0x100;
  ip += localIP[3];
  uint32_t mask;
  mask =  subnetMask[0] * 0x1000000;
  mask += subnetMask[1] *   0x10000;
  mask += subnetMask[2] *     0x100;
  mask += subnetMask[3];
  uint32_t broadcastIP = (ip & mask) + (~mask);
  return String((broadcastIP >> 24) & 0xff) + String(".") +
         String((broadcastIP >> 16) & 0xff) + String(".") +
         String((broadcastIP >> 8 ) & 0xff) + String(".") +
         String((broadcastIP      ) & 0xff);
}

String ip2String(IPAddress ipAddress)
{ // https://stackoverflow.com/a/55805863/11087027
  return String(ipAddress[0]) + String(".") +
         String(ipAddress[1]) + String(".") +
         String(ipAddress[2]) + String(".") +
         String(ipAddress[3]);
}

String mac2String(uint8_t* mac)
{
  char macStr[18]; // https://stackoverflow.com/a/16550666/11087027
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], mac[6]);
  return String(macStr);
}

void updateTimeLCD()
{
  char timeStringBuff[50];
  getLocalTime(&timeinfo);
  strftime(timeStringBuff, sizeof(timeStringBuff) - 1, "%A, %B %d %Y %H:%M:%S", &timeinfo);
  Serial.printf("%s\n", timeStringBuff);
  strftime(timeStringBuff, sizeof(timeStringBuff), "%A,", &timeinfo);
  printCentered(timeStringBuff, 0);
  strftime(timeStringBuff, sizeof(timeStringBuff), "%B", &timeinfo);
  printCentered(timeStringBuff, 1);
  strftime(timeStringBuff, sizeof(timeStringBuff), "%d, %Y", &timeinfo);
  printCentered(timeStringBuff, 2);
  strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M:%S", &timeinfo);
  printCentered(timeStringBuff, 3);
}

uint8_t size;
long timeout = 700;
unsigned long desiredCycleTime = 1000;


void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1,25,26); // RX on pin 25 TX on pin 26
  delay(200);
  Serial2.setTimeout(timeout);
  Serial.printf("\nStarting, initializing TFT display\n");
  tft.init();
  tft.setRotation(rotation);
  width = tft.width();
  height = tft.height();
  tft.fillScreen(bgcolor);
  tft.setTextColor(txtfgcolor);
  tft.setTextFont(fontNum);
  thisFontHeight = tft.fontHeight(fontNum);
  // labelButton1(button1Label, txtbgcolor); // Don't have a use for button1 yet
  labelButton2(button2Label, txtbgcolor);    // redraw button label in case it gets covered up
  
  WiFi.macAddress(mac);
  Serial.printf("Connecting to WiFi %s, MAC: %s\n", networkName, mac2String(mac).c_str());
  printCentered((char *)mac2String(mac).c_str(), 0);
  printCentered("WiFi connecting:",              1);
  printCentered((char *)networkName,             2);
  
  //Connect to the WiFi network
  // connectToWiFi(networkName, networkPswd);
  if (sizeof(networkPswd) > 0)
  { WiFi.begin(networkName, networkPswd); }
  else
  { WiFi.begin(networkName); }
  
  for (int i = 0; ( (i < 60) && (WiFi.status() != WL_CONNECTED) ); i++)
  {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) { connected = true; }
  else { WiFi.disconnect(); }  // stop trying if it timed out.

  if (connected)
  {
    Serial.printf("\nConnected to wifi. IP: ");
    Serial.printf("%s\n", ip2String(WiFi.localIP()).c_str());
    Serial.printf("Setting the time from NTP server\n");
    printCentered((char *)networkName,                         0);
    printCentered((char *)ip2String(WiFi.localIP()).c_str(),   1);
    printCentered((char *)ip2String(WiFi.gatewayIP()).c_str(), 3);
    printCentered((char *)ntpServer1,                          2);
    // printCentered((char *)ntpServer2,                          4);
    Serial.printf("Gateway IP: %s\n", (char *)ip2String(WiFi.gatewayIP()).c_str());
    Serial.printf("mask: %s\n", ip2String(WiFi.subnetMask()).c_str());
    strcpy(udpAddress, (char *)broadcastAddr(WiFi.localIP(), WiFi.subnetMask()).c_str() );
    Serial.printf("Broadcasting to: %s\n", udpAddress);
    configTime((gmtOffset_hr * 3600L), (daylightOffset_hr * (int)3600), ntpServer1, (char *)ip2String(WiFi.gatewayIP()).c_str());
    delay(400);
    if(!getLocalTime(&timeinfo))
    {
      Serial.println("Failed to obtain time");
      printCentered("Time failed", 2);
      printCentered("",            3);
      printCentered("",            4);
      return;
    }
    else
    {
      Serial.printf("Time set. ");
      tft.fillScreen(bgcolor);
      updateTimeLCD();
    }
    printCentered("Opening UDP port", 0);
    Serial.printf("\nOpening reference to UDP port - ");
    udp.begin(udpPort);
    Serial.printf("Done.\n");
    printCentered("", 0);
  }
  else
  {
    Serial.printf("Failed to connect to WiFi.\n");
    printCentered("WiFi failed:", 1); // leaving MAC address & SSID on the screen
  }
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
  clearIncomingBuffer();  // discard anything coming in beyond first 36 bytes. not expecting more
  // before we continue, lets check if WiFi got disconnected
  if (connected)
  { if (WiFi.status() != WL_CONNECTED)
    { connected = false;
      WiFi.disconnect();
      Serial.printf("\n***\n*** WiFi disconnected ***\n***\n\n");
      printCentered("NO WIFI", 4);
    }
  }
  // TODO: work in a way to eventually reconnect to wifi and obtain the time if wifi
  // comes online after it was found to be offline/failed earlier
  if (bufferSize != 36)
  {
    if (bufferSize > 0)
    { Serial.printf("Error: Only received %d out of 36 bytes in %.3f second timeout.\n", bufferSize, (timeout / 1000.0f) ); }
    else
    { Serial.printf("Nothing received in %.3f second timeout.\n", (timeout / 1000.0f) ); }
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
      printCentered((char *)String(String(percentCharged(incomingBuffer)) + "% " + String(amps) + "A" ).c_str(), 4);
      if(connected)
      { udp.beginPacket(udpAddress,udpPort);
        uint8_t timestamp [sizeof(now)];
        time_t now2;
        time(&now);
        now2 = now;
        now += epochShift;
        now = swapped(now);
        memcpy(&timestamp, &now, sizeof(now));
        udp.write(timestamp, sizeof(timestamp)); 
        udp.write(incomingBuffer, sizeof(incomingBuffer));
        // now = swapped(now) - epochShift; // change it back
        udp.printf(" %lu", now2);
        udp.endPacket();
        Serial.printf("sent UDP packet to %s:%d\n", udpAddress, udpPort);
      }
    }
  }

  updateTimeLCD();
  if (connected)
  {
    long rssi = WiFi.RSSI();
    Serial.printf("Strength of WiFi signal from access point: %ld\n", rssi);  
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
  
  // now wait as long as needed to restart the cycle almost exactly 1 seconds after the previous one started.
  unsigned long cycleEndTime = millis();
  Serial.printf("Ended at %.3f seconds, waiting ", (cycleEndTime - cycleStartTime) / 1000.0F );
  long desiredEndTime = cycleStartTime + desiredCycleTime; // this will begin to malfunction after being online for 7.1 weeks or half of that?
  long timeToWait = desiredEndTime - cycleEndTime;
  if (timeToWait > 0)
  {
    while (millis() - desiredEndTime > 1)
    { 
      long now = millis();
      if ( now - desiredEndTime > 0) { delay(now - desiredEndTime); }     
    }
  }
  Serial.printf(" - Full cycle time: %.3f S\n\n", (millis() - cycleStartTime) / 1000.0 );
}
