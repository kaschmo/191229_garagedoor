# 191229_garagedoor
ESP8266 based garagedoor opener relay with temp sensor and relay input.

Garage Door Controller
- Sends and receives values via MQTT (topic: cmnd or stat/garage_relay/***). MQTT on port 1883
- BME280 sensor for P,H,T Measurements via I2C
- Relay for switching Garage Door. via 5V VCC and grounding pin
- GPIO In for status relay (open/close) on garage door. Via Optocoupler since 24V relay.