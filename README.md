# sompis-esp8266-sensor-fw

Firmware for the ESP8266 based temperature sensors.

WIP. `main.cpp` may not even compile currently because it was cleaned up massively to open source the code.

## Usage

It's a platformio project, you can probably load it into it.

## Device

- Esp8266-12E/F/.. chip
- Possibly a breakout board
- A bunch of resistors
- A capacitor
- Maybe a regulator
- Some way to feed it power
- A DS18B20 temperature sensor
- A jump cable from GPIO16 to RST so it can wake up from deep sleep using a timer

## Logic

- Wake up with the wifi disabled
- Read some bytes from the tiny real-time clock memory that survives deep sleeps so some values can be persisted between boots
- Tell the sensor to start the temperature measuring, according to documentation, this takes 188ms. Some other stuff can be done while it's measuring in the background.
- While it's working, do some preparations for connecting to the Wifi
- Once the measurement is there, compare it to the previously reported value. If the temperature delta is less than 0.5c, don't bother reporting at all and just go to sleep.
- If the temperature delta is over 0.5c, see if the device can connect to the primary wifi.
- If the connection is succesfull, store the BSSID and wifi channel to use on the subsequent boots, it's a whole lot faster to connect when you set the channel and BSSID. A static IP is also used on the primary wifi.
- If primary wifi won't connect, go to sleep. Unless it has failed 3 times already, then it's time to try the secondary wifi ap. And it that doesn't work, the tertiary.
- At this point, some wifi should be connected or the device is sleeping. Let's go on.
- Construct and encode a message using the protocol buffers definition found in (lib/SompaProto)[lib/SompaProto] and tell it the sensor id, device id, temperature, and anything else that needs to be reported.
- AES-128 encrypt the message (it's like 7 bytes, shouldn't take a lot of cpu time)
- Send it over UDP to a server listening on a fixed ip address (dns resolution takes time)
- The server sends a response (in protobuf again)
- AES-128 decrypt the response
- The response usually contains `goto_sleep_seconds` value, which orders the device to go to sleep for X seconds. This can be useful when there are multiple devices, they can be synced to report at the same time, so the mobile hotspot doesn't have to communicate all the time. This should save some battery. This can also be used to adjust the reporting rate in case it seems to drain too much battery or is happening too frequently / infrequently.
- The response can also include an URL to a firmware upgrade, and the device can update itself over HTTP. This haven't been tested much and there are some risks involved.
- The response could also include something like `toggle = X` where `X` could for example be a led / indicator / something. Or `display = "message"` if there was a message display somewhere.

MQTT, HTTP or other "popular" ways to publish / receive IoT data has not been used, because they're surprisingly slow and the messages are lengthy. Also waiting for a response is kind of difficult using MQTT. With the binary UDP protocol buffers thing, the communication is fast and minimal, much unlike a good sauna conversation.

As the device sources are now public, the AES-128 encryption was added because "security over obscurity" is no longer valid and it would be too easy to troll the server or device. Especially if an unencrypted wifi (like the city wlan) is used.

