#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <complex.h> // For cabsf(float)
#include <OneWire.h> // For DS18B20 temperature sensor
#include <DallasTemperature.h> // For DS18B20 temperature sensor
#include <ESP8266httpUpdate.h> // For OTA firmware upgrade
#include <AES.h> // for AES-128-CBC
#include <FastCRC.h> // For CRC-16 Xmodem
#include <secrets.h> // see secrets.example.h
#include <DebugPrint.h> // For serial printing while developing
unsigned long startTime = millis();

#define FW_VERSION "v2"

WiFiClient plainWifi;

#define BUFFER_SIZE 256
byte outputBuffer[BUFFER_SIZE] { 0 };
unsigned char plaintextBuffer[BUFFER_SIZE] { 0 };
byte encryptedBuffer[BUFFER_SIZE] { 0 };
byte aesIV[AES_BLOCK_SIZE] { 0 };
AES aes;

// When debug-printing, use a smaller sleep multiplier so there's no need to wait a
// full minute or more to see next round.
#ifdef DEBUGPRINT
  #define SLEEP_MULTI 1e4
#else
  #define SLEEP_MULTI 1e6
#endif

#define DEVICE_ID 1
#define TEMP_SENSOR_ID 1
// Go into power save mode at POWER_SAVE_MIN volts.
// The battery is connected via a voltage divider to ADC of ESP8266.
#define VOLTAGE_SENSOR_ID 2
#define POWER_SAVE_MIN_V 2.9
// Data (yellow) is connected to GPIO5
#define ONE_WIRE_BUS 5
// Power (red) is connected to GPIO4 which will be pulled up when measuring
#define ONE_WIRE_POWER_PIN 4

// Used to identify rtc data is not corrupted or from another firmware
#define RTC_MAGIC 0x55aaaa58

// RTC memory data struct to pass data between deep sleep boots
typedef struct {
  unsigned long magic;
  float prev_temp;
  unsigned long boot_count;
  unsigned long wifi_fail_count;
  byte sensor_address[8];
  unsigned char use_ap_idx;
  unsigned char udp_fail_count;
} RtcData;

RtcData rtcData;

// Brings stuff such as REASON_DEEP_SLEEP_AWAKE
extern "C" {
  #include <user_interface.h>
}

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS(&oneWire);

// For generating CRC and validating response CRC
FastCRC16 crc16;

// Timer to know when the sensor should be ready to give a reading,
// can do other things while waiting.
unsigned long sensorReadyAt;

// Write RTC memory, close serial, go to deep sleep
void sleep(unsigned long seconds) {
  DEBUG_PRINTF("Sleeping: %lu seconds\n", seconds);
  rtcData.magic = RTC_MAGIC;
  system_rtc_mem_write(66, &rtcData, sizeof(rtcData));
  yield();
  DEBUG_PRINTF("Total time elapsed: %lu millis", millis() - startTime);
  DEBUG_PRINTF("\n\n\n");
  #ifdef DEBUGPRINT
  delay(10000);
  Serial.end();
  #endif
  ESP.deepSleep(seconds * SLEEP_MULTI, WAKE_NO_RFCAL);
}

// The DS18B20 power is wired to ONE_WIRE_POWER_PIN which is brought up
// before measuring, so it doesn't eat up power until needed
void setupTempSensor() {
  pinMode(ONE_WIRE_POWER_PIN, OUTPUT);
  digitalWrite(ONE_WIRE_POWER_PIN, HIGH);
  DS.begin();
  DS.setWaitForConversion(false);
  DEBUG_PRINTLN();

 // If there's no sensor address saved from previous boots, scan for it
  if(rtcData.sensor_address[0] == 0 && rtcData.sensor_address[1] == 0) {
    DEBUG_PRINTLN("Onewire search..");
    oneWire.search(rtcData.sensor_address, true);
    #ifdef DEBUGPRINT
    DEBUG_PRINT("Sensor address:");
    for(unsigned int i=0; i < sizeof(rtcData.sensor_address); i++) {
      DEBUG_PRINTF("%02x", rtcData.sensor_address[i]);
    }
    DEBUG_PRINTLN();
    #endif
  }
  DS.setResolution(rtcData.sensor_address, 10);
  DS.requestTemperaturesByAddress(rtcData.sensor_address);
  // Sensor will have a reading at now+188ms
  sensorReadyAt = millis() + 188;
}

void setupRtcData() {
  rst_info *resetInfo = ESP.getResetInfoPtr();
  if (resetInfo->reason != REASON_DEEP_SLEEP_AWAKE) {
    // regular boot, clear data
    system_rtc_mem_write(66, 0, 4);
    rtcData.boot_count = 0;
    rtcData.wifi_fail_count = 0;
    rtcData.prev_temp = -126.0;
    rtcData.use_ap_idx = 0;
    rtcData.udp_fail_count = 0;
    #ifdef DEBUGPRINT
    DEBUG_PRINTLN("First boot, waiting for terminal");
    delay(5000);
    DEBUG_PRINTLN("Let's go.");
    #endif
  } else {
    // deep sleep wake, read data from rtc memory
    system_rtc_mem_read(66, &rtcData, sizeof(rtcData));
    if(rtcData.magic == RTC_MAGIC) {
      rtcData.boot_count++;
    } else {
      // Whoops, rtc data magic mismatch, some corrupt data, rewrite
      rtcData.boot_count = 0;
      rtcData.wifi_fail_count = 0;
      rtcData.prev_temp = -126.0;
    }
  }
}

// Get random number from ESP hardware
uint8_t getrnd() {
    uint8_t really_random = *(volatile uint8_t *)0x3FF20E44;
    return really_random;
}

// Generate a random IV into a byte array
void randomIV(byte *iv) {
  for (int i = 0 ; i < AES_BLOCK_SIZE ; i++ ) {
    iv[i]= (byte) getrnd();
  }
}

void downcaseHex(char *hex, unsigned int length) {
  for(unsigned int i = 0; i < length; i++) {
    if(hex[i] >= 'A' && hex[i] <= 'Z') {
      hex[i] += 'a' - 'A';
    }
  }
}

unsigned int encrypt(unsigned char *plain_text, byte *iv, uint8_t *key, unsigned char *output) {
  DEBUG_PRINTLN("Generating random IV");
  randomIV(aesIV);
  memcpy(output, aesIV, AES_BLOCK_SIZE);
  DEBUG_PRINTLN("Calculating CRC-16 for payload");
  unsigned int length = strlen((char *)plain_text);
  DEBUG_PRINTF("Payload size: %d bytes:\n", length);
  DEBUG_WRITE(plain_text, length);
  DEBUG_PRINTLN();

  unsigned int crc = crc16.xmodem((uint8_t *)plain_text, length);
  char crc_hex[4]; 
  itoa(crc, crc_hex, 16);
  downcaseHex(crc_hex, 4);
  DEBUG_PRINTF("Payload CRC: %s\n", crc_hex);
  for(int i = 0; i < 4; i++) {
    plain_text[length + i] = crc_hex[i];
  }
  length += 4;
  DEBUG_PRINTF("Encrypting payload of %d bytes\n", length);
  aes.do_aes_encrypt((byte *)plain_text, length, output + 16, key, 128, iv);
  unsigned int encrypted_size = aes.get_size();
  DEBUG_PRINTF("Encrypted size is %d bytes\n", encrypted_size);
  encrypted_size += AES_BLOCK_SIZE;
  DEBUG_PRINTLN("Final out buffer looks like:");
  #ifdef DEBUGPRINT
  int i;
  for (unsigned int i = 0; i < encrypted_size; i++)
  {
      if (i > 0) DEBUG_PRINT(", ");
      DEBUG_PRINTF("0x%02X", output[i]);
  }
  #endif
  printf("\n");
  return encrypted_size;
}


unsigned int decrypt(byte* enciphered, byte *iv, uint8_t *key, byte* output, int length)
{
  DEBUG_PRINTF("Decrypting %d bytes\n", length);
  memset(output, 0, BUFFER_SIZE);
  aes.do_aes_decrypt(enciphered, length, output, key, 128, iv);
  unsigned int result_size = aes.get_size();
  DEBUG_PRINTLN("Buffer looks like:");
  #ifdef DEBUGPRINT
  int i;
  for (unsigned int i = 0; i < result_size; i++)
  {
      if (i > 0) DEBUG_PRINT(", ");
      DEBUG_PRINTF("0x%02X", output[i]);
  }
  #endif
  printf("\n");
  DEBUG_PRINTF("Decrypted size: %d bytes\n", result_size);

  // The string may have padding after the decryption, such as:
  // 0x65 0x45 0x04 0x04 0x04 0x04 where padding == 4
  // 0x65 0x45 0x03 0x03 0x03 0x03 where padding == 4
  unsigned int last_idx = result_size - 1;
  // Take the last byte into temp variable
  unsigned char last_byte = output[last_idx];
  // Only process if it's 1 .. 15 (0 would not be padded, 16 would
  // be a block of nothing but padding, which does not happen)
  if(last_byte >= 1 && last_byte <= 15) {
    bool result = true;

    // Go through the buffer backwards last byte times and if it's all
    // the same value, it's padding. 
    for(unsigned char i = 0; i >= last_byte; i++) {
      if(output[(result_size - 1) - i] != last_byte) {
        result = false;
      }
    }

    // Padding found, eliminate it.
    if(result) {
      result_size -= last_byte;
      output[result_size] = 0;
      DEBUG_PRINTF("Decrypted unpadded size: %d bytes\n", result_size);
    }
  }

  unsigned int without_crc_size = result_size - 4;
  DEBUG_PRINTF("Decrypted unpadded size without CRC: %d bytes\n", without_crc_size);

  unsigned int calculated_crc = crc16.xmodem((uint8_t *)output, without_crc_size);
  char crc_hex[4]; 
  itoa(calculated_crc, crc_hex, 16);
  downcaseHex(crc_hex, 4);
  DEBUG_PRINTF("Calculated CRC: %s\n", crc_hex);
  // I just couldn't get memcpy or any sort of for to work?!
  if(crc_hex[0] == output[without_crc_size] && crc_hex[1] == output[without_crc_size+1] && crc_hex[2] == output[without_crc_size+2] && crc_hex[3] == output[without_crc_size+3]) {
    output[without_crc_size] = 0;
    DEBUG_PRINTF("CRC match, returning %d as final length\n", without_crc_size);
    return without_crc_size;
  } else {
    DEBUG_PRINTLN("CRC mismatch");
    return 0;
  }
}

void wakeUpWifi() {
  WiFi.forceSleepWake();
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  yield();
}

void setupWifi(wifiAp wifi_ap) {
  if(wifi_ap.static_ip) {
    WiFi.config(localIp, gatewayIp, subnetIp, gatewayIp, dnsIp);
  } else {
    WiFi.config(0, 0, 0, 0, 0);
  }
  if(wifi_ap.channel == NULL) {
    WiFi.begin(wifi_ap.ssid, wifi_ap.password);
  } else {
    WiFi.begin(wifi_ap.ssid, wifi_ap.password, wifi_ap.channel, wifi_ap.bssid, true);
  }
  WiFi.setAutoConnect(false);
  WiFi.setAutoReconnect(false);
  yield();
}

bool waitForWifi() {
  DEBUG_PRINT("Waiting for wifi..");
  unsigned long wifiStart = millis();
  while (!WiFi.isConnected()) {
    yield();
    if (millis() - wifiStart > 8000) {
      DEBUG_PRINTLN("failed");
      return false;
    }
    #ifdef DEBUGPRINT
    if((millis() % 1000) == 0) { DEBUG_PRINT("."); }
    #endif
  }

  DEBUG_PRINTLN("SUCCESS");
  DEBUG_PRINTF("Connected to %s\n", WiFi.SSID().c_str());
  return true;
}

float batteryVoltage() {
  pinMode(A0, INPUT);
  return (analogRead(A0)/1023.0) * 4.2;
}

float currentTemp() {
  float tmp;
  DEBUG_PRINT("Waiting for sensor ready.");
  while(millis() < sensorReadyAt) {
    #ifdef DEBUGPRINT
    if((millis() % 50) == 0) {
      DEBUG_PRINT(".");
    }
    #endif
    yield();
  }
  DEBUG_PRINTLN();
  tmp = DS.getTempC(rtcData.sensor_address);
  if(tmp < -126) {
    DEBUG_PRINTLN("No reading, wait a while and retry");
    yield();
    delay(100);
    tmp = DS.getTempC(rtcData.sensor_address);
  }
  if(tmp < -126) {
    DEBUG_PRINTLN("No reading, sleep");
    memset(rtcData.sensor_address, '\0', sizeof(rtcData.sensor_address));
    sleep(60);
  }
  oneWire.depower();
  digitalWrite(ONE_WIRE_POWER_PIN, LOW);
  return tmp;
}

bool firmwareUpgrade(char *url) {
  ESPhttpUpdate.rebootOnUpdate(true);
  yield();
  t_httpUpdate_return ret = ESPhttpUpdate.update(url);
  switch(ret) {
    case HTTP_UPDATE_FAILED:
        DEBUG_PRINTF("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        return false;
        break;

    case HTTP_UPDATE_NO_UPDATES:
        DEBUG_PRINTLN("HTTP_UPDATE_NO_UPDATES");
        return false;
        break;

    case HTTP_UPDATE_OK:
        DEBUG_PRINTLN("HTTP_UPDATE_OK");
        ESP.restart();
        return true;
        break;
  }
  return false;
}

unsigned int udpRequest(char *buffer, char *response_iv, char *response, unsigned int payload_size) {
  WiFiUDP UDP;
  DEBUG_PRINTF("Starting local UDP listener on port %d\n", udpLocalPort);
  UDP.begin(udpLocalPort);
  yield();
  UDP.beginPacket(apiHost, apiUDPPort);
  DEBUG_PRINTLN("Writing packet..");
  unsigned int bytesWritten;
  bytesWritten = UDP.write(buffer, payload_size);
  DEBUG_PRINTF("Bytes written: %d\n", bytesWritten);

  if(UDP.endPacket() < 1) {
    rtcData.udp_fail_count++;
    return 0;
  }

  unsigned long millisNow = millis();
  int cb = 0;

  DEBUG_PRINT("Waiting for UDP response .");
  while(cb < 1) {
    #ifdef DEBUGPRINT
    if(millis() % 100 == 0) {
      DEBUG_PRINT(".");
    }
    #endif
    if((millis() - millisNow) > 3000) {
      DEBUG_PRINTLN("fail\nTimed out waiting for UDP response");
      rtcData.udp_fail_count++;
      return 0;
    }
    cb = UDP.parsePacket();
    if(cb < 1) {
      yield();
    }
  }
  DEBUG_PRINTLN("OK\nUDP data available!");

  unsigned int bytesRead;
  DEBUG_PRINTLN("Reading AES IV");
  bytesRead = UDP.read(response_iv, 16);
  DEBUG_PRINTF("Received %d bytes of UDP API AES IV\n", bytesRead);
  DEBUG_PRINTLN("Reading AES payload");
  bytesRead = UDP.read(response, BUFFER_SIZE);
  DEBUG_PRINTF("Received %d bytes of UDP API payload\n", bytesRead);
  return bytesRead;
}

unsigned int tcpRequest(char *buffer, char *response_iv, char *response, unsigned int payload_size) {
  WiFiClient TCP;

  DEBUG_PRINTLN("Connecting to TCP API");
  TCP.setTimeout(3000);
  if(!TCP.connect(apiHost, apiTCPPort)) {
    DEBUG_PRINTLN("Connection failed");
    return 0;
  }
  yield();

  DEBUG_PRINT("Waiting to send .");
  unsigned long millisNow = millis();
  while(TCP.availableForWrite() < 1) {
    #ifdef DEBUGPRINT
    if(millis() % 100 == 0) {
      DEBUG_PRINT(".");
    }
    #endif
    yield();
    if(millis() - millisNow > 3000) {
      DEBUG_PRINTLN("Server write did not become available, time out.\n");
      return 0;
    }
  }
  DEBUG_PRINTLN("OK\n");

  DEBUG_PRINTF("Sending packet of %d bytes ..\n", payload_size);
  unsigned int bytesWritten = TCP.write(buffer, payload_size);
  if(bytesWritten != payload_size) {
    DEBUG_PRINTLN("Failed");
    return 0;
  }

  DEBUG_PRINT("Waiting for data available .");
  millisNow = millis();
  while(TCP.available() < 1) {
    #ifdef DEBUGPRINT
    if(millis() % 100 == 0) {
      DEBUG_PRINT(".");
    }
    #endif
    yield();
    if(millis() - millisNow > 3000) {
      DEBUG_PRINTLN("Server read did not become available, time out.\n");
      return 0;
    }
  }
  DEBUG_PRINTLN("OK\n");

  unsigned int bytesRead;
  bytesRead = TCP.readBytes(response_iv, 16);
  DEBUG_PRINTF("Received %d bytes TCP API IV\n", bytesRead);
  bytesRead = TCP.readBytes(response, BUFFER_SIZE);
  DEBUG_PRINTF("AES payload bytes read: %d bytes\n", bytesRead);

  DEBUG_PRINTLN("Disconnecting");
  TCP.stop();
  return bytesRead;
}

void processResponse(char *buffer, unsigned int length) {
  int sleep_time = 0;

  DEBUG_PRINTLN("Server response:");
  DEBUG_WRITE(buffer, length);
  DEBUG_PRINTLN();
  sscanf(buffer, "SLEEP %d", &sleep_time);
  if(sleep_time > 0) {
    DEBUG_PRINTF("Requested sleep: %d seconds. See you!\n", sleep_time);
    sleep(sleep_time);
  }
  char fw_url[64]; 
  sscanf(buffer, "FW %s", fw_url);
  if(strlen(fw_url) > 0) {
    DEBUG_PRINTF("Firmware upgrade requested from %s\nUpgrading.\n", fw_url);
    if(!firmwareUpgrade(fw_url)) {
      DEBUG_PRINTLN("Failed for some reason.");
    }
  }
}

void setup() {
  float currentVoltage;

  #ifdef DEBUGPRINT
  Serial.begin(115200);
  DEBUG_PRINTF("\n\n\n");
  #endif

  setupRtcData();

  currentVoltage = batteryVoltage();
  DEBUG_PRINTF("Current voltage: %fv\n", currentVoltage);

  float minDelta;

  // When in power save, report only 5C steps
  if(currentVoltage < POWER_SAVE_MIN_V) {
    DEBUG_PRINTLN("In power save mode");
    minDelta = 5;
  } else {
    minDelta = 0.5;
  }

  DEBUG_PRINTF("Boot count: %lu\n", rtcData.boot_count);

  setupTempSensor();

  aes.set_key(aesKey, AES_BLOCK_SIZE);

  float currTemp = currentTemp();
  float tempDelta = currTemp - rtcData.prev_temp;
  
  sprintf((char *)plaintextBuffer, "D:%d|b:%.3f|B:%lu|W:%d|t:%.2f|V:%s|R:%d", DEVICE_ID, currentVoltage, rtcData.boot_count, rtcData.use_ap_idx, currTemp, FW_VERSION, WiFi.RSSI());

  if(rtcData.prev_temp < -120) { tempDelta = minDelta + 1.0; }

  DEBUG_PRINTF("Current temp: %fC - delta: %fC\n", currTemp, tempDelta);

  if(cabsf(tempDelta) < minDelta) {
    DEBUG_PRINTLN("Temp delta too small, dont bother connecting WIFI");
    sleep(60);
  }

  wakeUpWifi();

  DEBUG_PRINTF("Configuring wifi #%d\n", rtcData.use_ap_idx);
  setupWifi(wifiApList[rtcData.use_ap_idx]);

  unsigned int payload_size = encrypt((unsigned char *)plaintextBuffer, aesIV, aesKey, outputBuffer);

  if(waitForWifi()) {
    rtcData.wifi_fail_count = 0;
  } else {
    rtcData.wifi_fail_count++;
    if(rtcData.wifi_fail_count > 2) {
      rtcData.use_ap_idx++;
      rtcData.udp_fail_count = 0;
      if(rtcData.use_ap_idx >= sizeof(wifiApList)) {
        rtcData.use_ap_idx = 0;
      }
    }
    sleep(60);
  }

  unsigned int bytesRead = 0;

  if(rtcData.udp_fail_count < 2) {
    bytesRead = udpRequest((char *)outputBuffer, (char *)aesIV, (char *)encryptedBuffer, payload_size); 
    if(bytesRead > 0) { rtcData.udp_fail_count = 0; }
  }

  if(bytesRead == 0) {
    bytesRead = tcpRequest((char *)outputBuffer, (char *)aesIV, (char *)encryptedBuffer, payload_size); 
  }

  if(bytesRead == 0) {
    DEBUG_PRINTLN("Got nothing :(");
    sleep(60);
  }

  unsigned int plaintext_size = decrypt(encryptedBuffer, aesIV, aesKey, plaintextBuffer, bytesRead);
  if(plaintext_size == 0) {
    DEBUG_PRINTLN("Decrypt returned nothing :(");
    sleep(60);
  }

  processResponse((char *)plaintextBuffer, plaintext_size); // should put device to sleep or upgrade + reset
  sleep(60); // make sure
}

// No need to loop, everything is "wake up -> do work -> go to sleep". Like life, except there's sauna in the middle.
void loop() {};
