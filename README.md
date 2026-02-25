# LoRaWAN SHTC3 Sensor (ESP32-PICO-MINI-02)

ESP-IDF project for an ESP32-based ItsyBitsy board with:
- Adafruit RFM95W (SX1276)
- SHTC3 temperature/humidity sensor
- LoRaWAN uplinks to ChirpStack

Behavior:
- Reads temperature + humidity from SHTC3.
- Sends one uplink every 30 seconds.
- Deep-sleeps between sends to reduce battery use.

## Join mode

Current default is **OTAA**.
- Set `LORA_USE_OTAA 1` in `main/lora_config.h` to use OTAA.
- Set `LORA_USE_OTAA 0` to use ABP fallback.

## Network profile used here

- Region: `US915`
- Uplink channels: `0-7` (sub-band 1)
- Radio: `SX1276` (RFM95W)

## Wiring (ItsyBitsy ESP32 profile)

### ItsyBitsy ESP32 <-> RFM95W (SPI)
- `GPIO19` -> `RFM95W SCK`
- `GPIO22` -> `RFM95W MISO`
- `GPIO21` -> `RFM95W MOSI`
- `GPIO26` -> `RFM95W CS` (NSS)
- `GPIO25` -> `RFM95W RST`
- `GPIO4` -> `RFM95W DIO0`
- `GPIO38` -> `RFM95W DIO1`
- `DIO2` unused (`-1`) in current config
- `3V3` -> `RFM95W VIN`
- `GND` -> `RFM95W GND`

### Radio power BJTs
- `GPIO33` -> BJT control A
- `GPIO32` -> BJT control B
- On wake/active: `GPIO33=LOW`, `GPIO32=HIGH`
- Before deep sleep: `GPIO33=HIGH`, `GPIO32=LOW`
- Firmware enables GPIO hold so these levels are retained through deep sleep.

### ItsyBitsy ESP32 <-> SHTC3 (I2C)
- `GPIO15` -> `SDA`
- `GPIO27` -> `SCL`
- `3V3` -> `VCC`
- `GND` -> `GND`

### Battery voltage divider -> ADC
- `GPIO37` -> divider midpoint
- `GPIO14` -> divider power enable (HIGH=on, LOW=off)
- `R_top = 4650 ohm` from battery+ to midpoint
- `R_bottom = 1000 ohm` from midpoint to GND
- Conversion used in firmware: `Vbat = Vadc * (5650 / 1000)`
- Firmware drives `GPIO14` HIGH while awake and LOW before deep sleep.

## ChirpStack setup

### OTAA (default)
Create an OTAA device and copy these values into `main/lora_config.h`:
- `LORA_APPEUI` (little-endian byte order for LMIC)
- `LORA_DEVEUI` (little-endian byte order for LMIC)
- `LORA_APPKEY` (MSB byte order)

### ABP (fallback)
If `LORA_USE_OTAA` is `0`, set:
- `LORA_DEVADDR`
- `LORA_NWKSKEY`
- `LORA_APPSKEY`

Set channel plan in ChirpStack/gateway to match:
- US915 sub-band 1 (channels 0-7 + channel 64)

## Payload format (port 2)

6 bytes total:
- Bytes 0-1: temperature in centi-degrees C, signed int16 big-endian.
- Bytes 2-3: humidity in centi-%RH, unsigned int16 big-endian.
- Bytes 4-5: battery in millivolts, unsigned uint16 big-endian.

Example decoder (JavaScript):

```js
function decodeUplink(input) {
  const b = input.bytes;
  if (b.length !== 6) return { errors: ["invalid length"] };

  let t = (b[0] << 8) | b[1];
  if (t & 0x8000) t -= 0x10000;
  const h = (b[2] << 8) | b[3];
  const v = (b[4] << 8) | b[5];

  return {
    data: {
      temperature_c: t / 100,
      humidity_rh: h / 100,
      battery_v: v / 1000
    }
  };
}
```

## Build

1. Install ESP-IDF.
2. Ensure LMIC region is US915 in your build config (`menuconfig` if needed).
3. In project root:

```bash
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Notes

- OTAA session and frame counters are now persisted in RTC memory and restored after deep sleep, so it does not rejoin every 30-second cycle.
- If RTC memory is lost (full power loss/reset), device will OTAA-join again automatically.
- 30-second uplinks may violate LoRaWAN fair-use expectations depending on airtime (spreading factor, payload, gateway policy).
