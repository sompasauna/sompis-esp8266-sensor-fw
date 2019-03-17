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
  #define SLEEP_MULTI 1e4
#else
  #define SLEEP_MULTI 1e6
#endif

#define DEVICE_ID 1
#define TEMP_SENSOR_ID 1
#define VOLTAGE_SENSOR_ID 2
// Go into power save mode at POWER_SAVE_MIN volts.
#define POWER_SAVE_MIN_V 2.85
// Data (yellow) is connected to GPIO5
#define ONE_WIRE_BUS 5
// Power (red) is connected to GPIO4 which will be pulled up when measuring
#define ONE_WIRE_POWER_PIN 4

// Used to identify rtc data is not corrupted or from another firmware
#define RTC_MAGIC 0x55aaaa57

// RTC memory data struct to pass data between deep sleep boots
typedef struct {
  unsigned long magic;
  float prev_temp;
  unsigned long boot_count;
  unsigned long wifi_fail_count;
  byte sensor_address[8];
  unsigned int use_ap_idx;
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
  DEBUG_PRINTF("\n\n\n");
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
  WiFi.begin(wifi_ap.ssid, wifi_ap.password, wifi_ap.channel, wifi_ap.bssid, true);
  WiFi.setAutoConnect(false);
  WiFi.setAutoReconnect(false);
  yield();
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

unsigned char outBuffer[(SensorReport_size + 1) * 8];
unsigned int outBufferSize;

void appendSensorReport(SensorReport sensor_report) {
  bool status;
  
  if(outBufferSize >= sizeof(outBuffer)) {
    DEBUG_PRINTLN("outBuffer full, can't append.");
    return;
  }

  pb_ostream_t stream = pb_ostream_from_buffer(&outBuffer[outBufferSize + 1], SensorReport_size);

  status = pb_encode(&stream, SensorReport_fields, &sensor_report);

  if (!status) {
    DEBUG_PRINTF("Encoding failed: %s\n", PB_GET_ERROR(&stream));
  } else {
    DEBUG_PRINTF("Message encoded, length: %d bytes\n", stream.bytes_written);
    unsigned char message_size = (unsigned char) stream.bytes_written; 
    outBuffer[outBufferSize] = message_size;
    outBufferSize += message_size + 1;
    DEBUG_PRINTF("Outbuffer size now %d bytes\n", outBufferSize);
  }
  return;
}

SensorReport generateBatteryReport(float volt) {
  SensorReport message = SensorReport_init_zero;

  message.device_id = DEVICE_ID;
  message.sensor_id = VOLTAGE_SENSOR_ID;
  message.which_value = SensorReport_f_tag;
  message.value.f = volt;

  return message;
}

SensorReport generateTempReport(float temp) {
  SensorReport message = SensorReport_init_zero;

  message.device_id = DEVICE_ID;
  message.sensor_id = TEMP_SENSOR_ID;
  message.which_value = SensorReport_f_tag;
  message.value.f = temp;
  message.boot_count = rtcData.boot_count;
  message.wifi_id = wifiId;
  message.millis = millis();

  return message;
}

void sendReport() {
  // Start UDP response socket
  UDP.begin(udpLocalPort);

  DEBUG_PRINTLN("Sending packet");
  UDP.beginPacket(udpHost, udpPort);
  UDP.write(outBuffer, outBufferSize);
  UDP.endPacket();
  yield();
  recvPacket(true);
}

void setup() {
  startTime = millis();
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

  appendSensorReport(generateBatteryReport(currentVoltage));

  float currTemp = currentTemp();
  float tempDelta = currTemp - rtcData.prev_temp;

  if(rtcData.prev_temp < -120) { tempDelta = 1.0; }

  DEBUG_PRINTF("Current temp: %fC - delta: %fC\n", currTemp, tempDelta);

  if(cabsf(tempDelta) < minDelta) {
    DEBUG_PRINTLN("Temp delta too small, dont bother connecting WIFI");
    sleep(60);
  }

  wakeUpWifi();

  DEBUG_PRINTF("Configuring wifi #%d\n", rtcData.use_ap_idx);
  setupWifi(wifiApList[rtcData.use_ap_idx]);
  if(waitForWifi()) {
    rtcData.wifi_fail_count = 0;
    wifiId = wifiApList[0].id;
  } else {
    rtcData.wifi_fail_count++;
    if(rtcData.wifi_fail_count > 2) {
      rtcData.use_ap_idx++;
      if(rtcData.use_ap_idx >= sizeof(wifiApList)) {
        rtcData.use_ap_idx = 0;
      }
    }
    sleep(60);
  }

  appendSensorReport(generateTempReport(currTemp));
  rtcData.prev_temp = currTemp;
  sendReport(); // should put the device to sleep
  sleep(60); // but make sure
}

// No need to loop, everything is "wake up -> do work -> go to sleep". Like life, except there's sauna in the middle.
void loop() {};
