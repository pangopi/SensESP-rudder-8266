# SensESP-rudder-8266
SignalK rudder position sensor using SensESP running on an ESP8266

> I had to use an ESP8266 because I was out of ESP32's. But it works well so I've kept it running.

This device reports the rudder angle to SignalK by using a potentiometer connected to the rudder post with a belt. It uses [SensESP framework](https://github.com/SignalK/SensESP) version 1.07 (for ESP8266 compatability) to use sensors and report to SignalK.
