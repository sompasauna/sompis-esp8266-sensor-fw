# sompis-esp8266-sensor-fw

Firmware for the ESP8266 based temperature sensors for [Sompasauna](https://sompasauna.fi/).

## TLDR

- ESP8266
- DS18B20
- AES-128-CBC + CRC16 plain text ascii protocol over UDP or TCP.
- No MQTT

## Usage

It's a platformio project, you can probably load it into it.

### The API

The server side Ruby UDP/TCP API is at [somppasauna/sompis-sensor-api](https://github.com/sompasauna/sompis-sensor-api).

## Device

- Esp8266-12E/F/.. chip
- Possibly a breakout board
- A bunch of resistors
- A capacitor
- Maybe a regulator?
- Some way to feed it power!
- A DS18B20 temperature sensor
- A jump cable from GPIO16 to RST so it can wake up from deep sleep using a timer
- A voltage divider between battery, ESP8266 ADC analog input and GND to measure battery charge level

## Logic

- Wake up with the wifi disabled
- Read some bytes from the tiny real-time clock memory that survives deep sleeps so some values can be persisted between boots
- Tell the sensor to start the temperature measuring, according to documentation, this takes 188ms. Some other stuff can be done while it's measuring in the background.
- While it's working, do some preparations for connecting to the Wifi
- Once the measurement is there, compare it to the previously reported value. If the temperature delta is less than 0.5c, don't bother reporting at all and just go to sleep.
- If the temperature delta is over 0.5c, see if the device can connect to the primary wifi.
- If the connection is succesfull, store the BSSID and wifi channel to use on the subsequent boots, it's a whole lot faster to connect when you set the channel and BSSID. A static IP is also used on the primary wifi.
- If primary wifi won't connect, go to sleep. Unless it has failed 3 times already, then it's time to try the secondary wifi ap. And it that doesn't work, the tertiary. And so on.
- At this point, some wifi should be connected or the device is sleeping. Let's go on.
- Construct and encode a message using the ASCII line protocol
- AES-128-CBC encrypt the message (it's like 15 bytes, shouldn't take a lot of cpu time)
- Send it over UDP to a server listening on a fixed ip address (dns resolution takes time)
- The server sends a response
- AES-128 decrypt the response
- The response usually just contains `SLEEP xx`, which orders the device to go to sleep for X seconds. This can be useful when there are multiple devices, they can be synced to report at the same time, so the mobile hotspot doesn't have to communicate all the time. This should save some battery. This can also be used to adjust the reporting rate in case it seems to drain too much battery or is happening too frequently / infrequently.
- The response can also include an URL to a firmware upgrade in the form of `FW http://firmware.url.example.com/fwxyz.bin`, and the device will then update itself over HTTP.
- The response for other type of devices could also include something like `TOGGLE X` where `X` could for example be a led / indicator / something. Or `DISPLAY message` if there was a message display somewhere.
- If the device detects that the battery charge is getting low, it will go into power save mode where measurements are only reported if temperature delta is over 5 celsius.

MQTT, HTTP or other "popular" ways to publish / receive IoT data has not been used. The early versions used a Protocol Buffers based serialization, but with the encryption having 16 byte block size, it didn't make any traffic savings and was complicated to use.

MQTT has some drawbacks, such as generating about 6 kilobytes of traffic just to post one value. If any sort of response is expected, the device would have to subscribe to a message queue and wait a few seconds for a response. For HTTP, the headers alone are quite lenghty. Of course HTTPS and MQTTS should be used which adds even more. The ESP8266 WiFi and mobile data are both a bit slow and also the mobile hotspot is using a prepaid plan which has billing by the megabyte.

## The protocol

- The message is built as plain text: `field_name:value|field_name2:value2`. The "required" field name is `D`, which is the device id. 
- A CRC16 (xmodem) checksum is added to end of the message as 4 hex characters: `D:1|t:23.25|b:3.823cf7g`
- A random initialization vector of 16 bytes is generated and used to AES-128-CBC encrypt the message.
- The IV and the encrypted payload are concatenated and sent over UDP (or TCP if UDP fails on the WiFi currently in use) to the server
- The server response has the same encryption scheme, but the syntax is something like `ACTION value`.

# Contributing

Issues and PR's welcome. You're also welcome to [Sompasauna](https://sompasauna.fi/), you can see the device in action while enjoying the sauna. For free. Around the clock. Every day.

# License

 [![License: CC BY-SA 4.0](https://licensebuttons.net/l/by-sa/4.0/80x15.png)](https://creativecommons.org/licenses/by-sa/4.0/) Creative Commons Attribution-ShareAlike 4.0 International
