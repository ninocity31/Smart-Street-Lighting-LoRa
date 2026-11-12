# Node — lamp post (master or slave)

- `node/node.ino`: firmware for the TTGO LoRa32 V2.1 (ESP32 + SX1276)

Set `NODE_ROLE` to `'M'` (master, with the HB-100 radar) or `'E'` (slave), and give each board its own `NODE_ID`.
The node dims its lamp, checks it with the LDR, and reports to the gateway every 5 minutes. See the [main README](../README.md).
