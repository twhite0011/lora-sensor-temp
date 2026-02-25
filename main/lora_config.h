#pragma once

#include <stdint.h>

// Target radio/module profile:
// - Adafruit RFM95W (Semtech SX1276)
// - US915 plan

// Wake/send period.
#define REPORT_INTERVAL_SECONDS 30

// Join timeout (OTAA).
#define LORA_JOIN_TIMEOUT_SECONDS 120

// RX window widening via LMIC clock-error compensation (% of MAX_CLOCK_ERROR).
// Larger value keeps RX windows open longer (higher join robustness, more RX power).
#define LORA_CLOCK_ERROR_PERCENT 5

// I2C pins for SHTC3 (board-default STEMMA pins on ItsyBitsy ESP32).
#define I2C_PORT_NUM 0
#define I2C_SDA_GPIO 15
#define I2C_SCL_GPIO 27
#define I2C_FREQ_HZ 100000

// Battery ADC (via resistor divider to GPIO37).
#define BATTERY_ADC_GPIO 37
#define BATTERY_DIV_EN_GPIO 14
#define BATTERY_ADC_SAMPLES 16
#define BATTERY_ADC_VREF_MV 1100
#define BATTERY_ADC_MAX_COUNTS 4096
#define BATTERY_DIVIDER_NUM 5650
#define BATTERY_DIVIDER_DEN 1000

// LoRa SPI pins (must match LMIC HAL / menuconfig SPI config).
#define LORA_PIN_SCK 19
#define LORA_PIN_MISO 22
#define LORA_PIN_MOSI 21

// LoRa radio control pins.
#define LORA_PIN_NSS 26
#define LORA_PIN_RST 25
#define LORA_PIN_DIO0 4
#define LORA_PIN_DIO1 38
#define LORA_PIN_DIO2 -1

// External BJT radio power switch control:
// Wake state: PWR_A low, PWR_B high
// Sleep state: PWR_A high, PWR_B low
#define LORA_PWR_PIN_A 33
#define LORA_PWR_PIN_B 32

// OTAA keys:
// - APPEUI and DEVEUI must be little-endian byte order for LMIC.
// - APPKEY is MSB byte order.
static const uint8_t LORA_APPEUI[8] = { 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13 };

static const uint8_t LORA_DEVEUI[8] = { 0x10, 0x49, 0xce, 0xcd, 0x32, 0xa7, 0x87, 0xab };

static const uint8_t LORA_APPKEY[16] = { 0xa9, 0x57, 0x4e, 0xed, 0xf7, 0x74, 0x26, 0x1e, 0x13, 0xd2, 0xc5, 0x2d, 0xad, 0xb5, 0x7e, 0x2a };

// Radio profile defaults.
#define LORA_TX_DATARATE DR_SF7
#define LORA_TX_POWER_DBM 14

// US915 sub-band (1..8). Sub-band 1 = channels 0-7 (+500k channel 64).
#define LORA_SUBBAND 1
