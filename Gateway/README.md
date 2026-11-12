# Gateway

- `gateway/gateway.ino`: firmware for the Arduino Uno with an RFM95W (LoRa) and a SIM800L (GPRS)

Sends ON at sunset and OFF at sunrise to all lamps, collects their reports, and sends them to the server over GPRS every 5 minutes. See the [main README](../README.md).
