#include <Arduino.h>
#include <complex.h> // for cabsf(float)
#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFiUdp.h>
#include "SompaData.h"
#include <ESP8266httpUpdate.h>
#include <secrets.h>
#include <DebugPrint.h>

// When debug-printing, use a smaller sleep multiplier so there's no need to wait a
// full minute or more to see next round.
#ifdef DEBUGPRINT
  #define SLEEP_MULTI 100
#else
  #define SLEEP_MULTI 1e6
#endif

#define DEVICE_ID 1
#define TEMP_SENSOR_ID 1
#define VOLTAGE_SENSOR_ID 2

// Data (yellow) is connected to GPIO5
#define ONE_WIRE_BUS 5
// Power (red) is connected to GPIO4 which will be pulled up when measuring
#define ONE_WIRE_POWER_PIN 4

// Used to identify rtc data is not corrupted or from another firmware
#define RTC_MAGIC 0x55aaaa56

// RTC memory data struct to pass data between deep sleep boots
typedef struct {
  unsigned long magic;
  float prev_temp;
  unsigned long boot_count;
  unsigned long wifi_fail_count;
  byte sensor_address[8];
  unsigned int last_channel;
  byte last_bssid[6];
} RtcData;

RtcData rtcData;

// Brings stuff such as REASON_DEEP_SLEEP_AWAKE
extern "C" {
  #include <user_interface.h>
}

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS(&oneWire);
WiFiUDP UDP;

// Timer to know when the sensor should be ready to give a reading,
// can do other things while waiting.
unsigned long sensorReadyAt;

// The sleep_seconds global is used to set sleep duration if the end of
// the loop is reached
unsigned long sleep_seconds;

// Could be used to select alt wifi after sleep TODO
unsigned long wifiId;

// Initialize prevTemp with -127
float prevTemp = -127.0;

unsigned long startTime;

// Write RTC memory, close serial, go to deep sleep
void sleep(unsigned long seconds) {
  DEBUG_PRINTF("Sleeping: %d seconds\n", seconds);
  rtcData.magic = RTC_MAGIC;
  system_rtc_mem_write(66, &rtcData, sizeof(rtcData));
  yield();
  DEBUG_PRINTF("Total time elapsed: %lu millis", millis() - startTime);
  #ifdef DEBUGPRINT
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
    for(int i=0; i < sizeof(rtcData.sensor_address); i++) {
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

// Returns true if a wifi ap with the given ssid is found listening
bool wifiListening(const char * ssid) {
  int n = WiFi.scanNetworks();
  if(n > 0) {
    for (int i = 0; i < n; ++i) {
      if(WiFi.SSID(i) == ssid) {
        DEBUG_PRINTLN("Wifi found in network scan");
        return true;
      }
    }
  }
  return false;
}

void wakeUpWifi() {
  WiFi.forceSleepWake();
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  yield();
}

void setupWifi() {
  WiFi.config(localIp, gatewayIp, subnetIp, gatewayIp, dnsIp);
  if(rtcData.boot_count == 0 || rtcData.wifi_fail_count > 3) {
    WiFi.begin(wifi_SSID, wifi_PWD);
  } else {
    // Use saved wifi channel & bssid to connect faster
    WiFi.begin(wifi_SSID, wifi_PWD, rtcData.last_channel, rtcData.last_bssid, true);
  }
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  yield();
  wifiId = 1;
}

bool setupWifiAlt() {
  if(wifiListening(wifi_SSID_ALT)) {
    WiFi.begin(wifi_SSID_ALT, wifi_PWD_ALT);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
    yield();
    wifiId = 2;
    return true;
  }

  if(wifiListening(wifi_SSID_ALT_2)) {
    WiFi.begin(wifi_SSID_ALT_2);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
    wifiId = 3;
    return true;
  }

  return false;
}

void sendUDP(unsigned char * buffer, size_t bytes) {
  DEBUG_PRINT("Sending UDP packet..");
  UDP.beginPacket(udpHost, udpPort);
  DEBUG_PRINT('.');
  UDP.write(buffer, bytes);
  DEBUG_PRINT('.');
  UDP.endPacket();
  DEBUG_PRINTLN("sent");
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

bool firmwareUrl_callback(pb_istream_t *stream, const pb_field_t *field, void **arg) {
    unsigned char buf[64] = {0};
    const char* ptr_const = (const char*) buf;

    size_t len = stream->bytes_left;

    if (len > sizeof(buf) - 1 || !pb_read(stream, buf, len))
        return false;

  DEBUG_PRINT("Server wants us to upgrade: '");
  DEBUG_WRITE(buf, len);
  DEBUG_PRINTLN("' firmware");
  ESPhttpUpdate.rebootOnUpdate(true);
  t_httpUpdate_return ret = ESPhttpUpdate.update(ptr_const);
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

void recvPacket(bool goto_sleep) {
  int cb = 0;
  unsigned long millisNow = millis();
  byte packetBuffer[128];

  while(cb < 1) {
    if((millis() - millisNow) > 3000) {
      DEBUG_PRINTLN("Timed out waiting for UDP response");
      return;
    }
    cb = UDP.parsePacket();
    if(cb < 1) {
      yield();
    }
  }
  UDP.read(packetBuffer, 128);
  DEBUG_PRINTF("Received: %d bytes\n", sizeof(packetBuffer));
  ServerResponse response = ServerResponse_init_zero;
  response.firmware_url.funcs.decode = &firmwareUrl_callback;
  pb_istream_t stream = pb_istream_from_buffer(packetBuffer, 128);
  if(pb_decode(&stream, ServerResponse_fields, &response)) {
      if(response.goto_sleep_seconds > 0) {
        DEBUG_PRINTF("Server asks to sleep for %d seconds\n", response.goto_sleep_seconds);
        if(goto_sleep) {
          sleep(response.goto_sleep_seconds);
        } else {
          DEBUG_PRINTLN("Not going to do that.");
        }
      } else {
        DEBUG_PRINTLN("Server didn't specify sleep seconds?! Going to sleep 2 minutes.");
        sleep(120);
      }
  } else {
    DEBUG_PRINTF("Decoding failed: %s\n", PB_GET_ERROR(&stream));
  }
}

void sendBatteryReport(float volt) {
  unsigned char buffer[SensorReport_size];
  size_t message_length;
  bool status;

  SensorReport message = SensorReport_init_zero;
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  message.device_id = DEVICE_ID;
  message.sensor_id = VOLTAGE_SENSOR_ID;
  message.which_value = SensorReport_f_tag;
  message.value.f = volt;
  message.wifi_failcount = rtcData.wifi_fail_count;
  message.boot_count = rtcData.boot_count;
  message.wifi_id = wifiId;
  message.millis = millis();

  status = pb_encode(&stream, SensorReport_fields, &message);
  message_length = stream.bytes_written;

  if (!status) {
    DEBUG_PRINTF("Encoding failed: %s\n", PB_GET_ERROR(&stream));
    return;
  } else {
    DEBUG_PRINTF("Message encoded, length: %d bytes", message_length);
  }

  DEBUG_PRINTLN("Sending packet");
  UDP.beginPacket(udpHost, udpPort);
  UDP.write(buffer, message_length);
  UDP.endPacket();
  yield();
  recvPacket(false); // should put device to sleep
}

void sendTempReport(float temp) {
  unsigned char buffer[SensorReport_size];
  size_t message_length;
  bool status;

  SensorReport message = SensorReport_init_zero;
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  message.device_id = DEVICE_ID;
  message.sensor_id = TEMP_SENSOR_ID;
  message.which_value = SensorReport_f_tag;
  message.value.f = temp;
  message.wifi_failcount = rtcData.wifi_fail_count;
  message.boot_count = rtcData.boot_count;
  message.wifi_id = wifiId;
  message.millis = millis();

  status = pb_encode(&stream, SensorReport_fields, &message);
  message_length = stream.bytes_written;

  if (!status) {
    DEBUG_PRINTF("Encoding failed: %s\n", PB_GET_ERROR(&stream));
    sleep(60);
  } else {
    DEBUG_PRINTF("Message encoded, length: %d bytes", message_length);
    rtcData.prev_temp = temp;
  }

  DEBUG_PRINTLN("Sending packet");
  UDP.beginPacket(udpHost, udpPort);
  UDP.write(buffer, message_length);
  UDP.endPacket();
  yield();
  recvPacket(true);
}

void setup() {
  startTime = millis();
  float currentVoltage;

  #ifdef DEBUGPRINT
  Serial.begin(115200);
  #endif

  setupRtcData();


  DEBUG_PRINTF("Boot count: %lu\n", rtcData.boot_count);
  DEBUG_PRINTF("Current voltage: %fv\n", currentVoltage);

  setupTempSensor();
  wakeUpWifi(); // the sensor will do sensor stuff in the background
  
  currentVoltage = batteryVoltage();
  DEBUG_PRINTF("Current voltage: %fv\n", currentVoltage);

  float currTemp = currentTemp();
  float tempDelta = currTemp - rtcData.prev_temp;

  if(rtcData.prev_temp < -120) { tempDelta = 1.0; }

  DEBUG_PRINTF("Current temp: %fC\n - delta: %fC\n", currTemp, tempDelta);

  if(cabsf(tempDelta) < 0.5) {
    DEBUG_PRINTLN("Temp delta too small, dont bother connecting WIFI");
    sleep(60);
  }

  // Try main wifi until it fails 10 times or if the other wifis have been
  // used (or failed) for over 10 times already, maybe the main wifi is back up.
  if(rtcData.wifi_fail_count < 10 || rtcData.wifi_fail_count > 30) {
    setupWifi();
  } else if(!setupWifiAlt()) {
    rtcData.wifi_fail_count++;
    sleep(3*60);
  }

  if(waitForWifi()) {
    if(WiFi.SSID() == wifi_SSID) {
      DEBUG_PRINTLN("Reset WIFI fail count, connected to primary Wifi");
      rtcData.wifi_fail_count = 0;

      // Store the wifi channel and BSSID to rtcData for faster connect
      rtcData.last_channel = WiFi.channel();
      memcpy(rtcData.last_bssid, WiFi.BSSID(), 6 );
    } else {
      // Connected to alt wifi, but count it as "failure" anyway:
      rtcData.wifi_fail_count++;
    }
  } else {
    rtcData.wifi_fail_count++;
    DEBUG_PRINTF("WIFI fail count: %lu\n", rtcData.wifi_fail_count);
    sleep(3*60);
  }

  // Start UDP response socket
  UDP.begin(udpLocalPort);

  sendBatteryReport(currentVoltage);
  sendTempReport(currTemp); // should put the device to sleep
  sleep(60); // but make sure
}

// No need to loop, everything is "wake up -> do work -> go to sleep". Like life, except there's sauna in the middle.
void loop() {};
