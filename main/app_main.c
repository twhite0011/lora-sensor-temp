#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lmic.h"
#include "hal/hal.h"

#include "lora_config.h"
#include "shtc3.h"

static const char *TAG = "app";
static volatile bool s_tx_done = false;
static volatile bool s_join_done = false;
static volatile bool s_join_failed = false;
#define LORA_RADIO_SETTLE_DELAY_MS 250

#define SESSION_MAGIC 0x4C4D4943u
#define SESSION_VERSION 1u

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t netid;
    uint32_t devaddr;
    uint8_t nwk_key[16];
    uint8_t art_key[16];
    uint32_t seqno_up;
    uint32_t seqno_dn;
    uint8_t valid;
} lora_session_rtc_t;

RTC_DATA_ATTR static lora_session_rtc_t s_rtc_session;

const lmic_pinmap lmic_pins = {
    .nss = LORA_PIN_NSS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LORA_PIN_RST,
    .dio = {LORA_PIN_DIO0, LORA_PIN_DIO1, LORA_PIN_DIO2},
    .spi = {LORA_PIN_MISO, LORA_PIN_MOSI, LORA_PIN_SCK},
};

void os_getArtEui(u1_t *buf)
{
    memcpy(buf, LORA_APPEUI, 8);
}

void os_getDevEui(u1_t *buf)
{
    memcpy(buf, LORA_DEVEUI, 8);
}

void os_getDevKey(u1_t *buf)
{
    memcpy(buf, LORA_APPKEY, 16);
}

void onEvent(ev_t ev)
{
    switch (ev) {
        case EV_JOINED:
            ESP_LOGI(TAG, "LoRa OTAA joined");
            s_join_done = true;
            s_rtc_session.magic = SESSION_MAGIC;
            s_rtc_session.version = SESSION_VERSION;
            s_rtc_session.netid = LMIC.netid;
            s_rtc_session.devaddr = LMIC.devaddr;
            memcpy(s_rtc_session.nwk_key, LMIC.nwkKey, sizeof(s_rtc_session.nwk_key));
            memcpy(s_rtc_session.art_key, LMIC.artKey, sizeof(s_rtc_session.art_key));
            s_rtc_session.seqno_up = LMIC.seqnoUp;
            s_rtc_session.seqno_dn = LMIC.seqnoDn;
            s_rtc_session.valid = 1;
            break;
        case EV_JOIN_FAILED:
        case EV_REJOIN_FAILED:
            ESP_LOGW(TAG, "LoRa OTAA join failed event: %d", ev);
            s_join_failed = true;
            break;
        case EV_TXCOMPLETE:
            ESP_LOGI(TAG, "LoRa TX complete");
            s_tx_done = true;
            if (s_rtc_session.valid) {
                s_rtc_session.seqno_up = LMIC.seqnoUp;
                s_rtc_session.seqno_dn = LMIC.seqnoDn;
            }
            break;
        default:
            break;
    }
}

static void log_wiring_profile(void)
{
    ESP_LOGI(TAG, "RFM95W wiring: SCK=%d MISO=%d MOSI=%d NSS=%d RST=%d DIO0=%d DIO1=%d DIO2=%d",
             LORA_PIN_SCK,
             LORA_PIN_MISO,
             LORA_PIN_MOSI,
             LORA_PIN_NSS,
             LORA_PIN_RST,
             LORA_PIN_DIO0,
             LORA_PIN_DIO1,
             LORA_PIN_DIO2);
    ESP_LOGI(TAG, "SHTC3 wiring: SDA=%d SCL=%d", I2C_SDA_GPIO, I2C_SCL_GPIO);
    ESP_LOGI(TAG, "LoRa power ctrl: A=%d B=%d", LORA_PWR_PIN_A, LORA_PWR_PIN_B);
    ESP_LOGI(TAG, "Battery ADC: GPIO=%d EN=%d samples=%d", BATTERY_ADC_GPIO, BATTERY_DIV_EN_GPIO, BATTERY_ADC_SAMPLES);
}

static esp_err_t read_battery_mv(uint16_t *battery_mv)
{
    if (battery_mv == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    adc_unit_t unit = ADC_UNIT_1;
    adc_channel_t channel = ADC_CHANNEL_0;
    ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(BATTERY_ADC_GPIO, &unit, &channel), TAG, "adc io->channel failed");
    if (unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "Battery ADC pin must map to ADC1");
        return ESP_ERR_INVALID_ARG;
    }

    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = unit,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_unit_handle_t adc_handle = NULL;
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_cfg, &adc_handle), TAG, "adc init failed");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_0,
        .bitwidth = ADC_BITWIDTH_12,
    };

    esp_err_t ret = adc_oneshot_config_channel(adc_handle, channel, &chan_cfg);
    if (ret != ESP_OK) {
        (void)adc_oneshot_del_unit(adc_handle);
        return ret;
    }

    int64_t sum_raw = 0;
    for (int i = 0; i < BATTERY_ADC_SAMPLES; i++) {
        int raw = 0;
        ret = adc_oneshot_read(adc_handle, channel, &raw);
        if (ret != ESP_OK) {
            (void)adc_oneshot_del_unit(adc_handle);
            return ret;
        }
        sum_raw += raw;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    (void)adc_oneshot_del_unit(adc_handle);

    float raw_avg = (float)sum_raw / (float)BATTERY_ADC_SAMPLES;
    float adc_mv = (raw_avg * (float)BATTERY_ADC_VREF_MV) / (float)BATTERY_ADC_MAX_COUNTS;
    float batt_mv = adc_mv * ((float)BATTERY_DIVIDER_NUM / (float)BATTERY_DIVIDER_DEN);

    if (batt_mv < 0.0f) {
        batt_mv = 0.0f;
    } else if (batt_mv > 65535.0f) {
        batt_mv = 65535.0f;
    }

    *battery_mv = (uint16_t)lroundf(batt_mv);
    return ESP_OK;
}

static void radio_power_pins_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LORA_PWR_PIN_A) | (1ULL << LORA_PWR_PIN_B),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&io_conf);

    gpio_config_t batt_en_conf = {
        .pin_bit_mask = (1ULL << BATTERY_DIV_EN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&batt_en_conf);
}

static void radio_power_set_awake(void)
{
    (void)gpio_deep_sleep_hold_dis();
    (void)gpio_hold_dis(LORA_PWR_PIN_A);
    (void)gpio_hold_dis(LORA_PWR_PIN_B);
    (void)gpio_set_level(LORA_PWR_PIN_A, 0);
    (void)gpio_set_level(LORA_PWR_PIN_B, 1);

    (void)gpio_hold_dis(BATTERY_DIV_EN_GPIO);
    (void)gpio_set_level(BATTERY_DIV_EN_GPIO, 1);
}

static void radio_power_set_sleep(void)
{
    (void)gpio_set_level(LORA_PWR_PIN_A, 1);
    (void)gpio_set_level(LORA_PWR_PIN_B, 0);
    (void)gpio_hold_en(LORA_PWR_PIN_A);
    (void)gpio_hold_en(LORA_PWR_PIN_B);

    (void)gpio_set_level(BATTERY_DIV_EN_GPIO, 0);
    (void)gpio_hold_en(BATTERY_DIV_EN_GPIO);

    (void)gpio_deep_sleep_hold_en();
}

static void enter_deep_sleep(void)
{
    uint64_t sleep_us = (uint64_t)REPORT_INTERVAL_SECONDS * 1000000ULL;
    radio_power_set_sleep();
    ESP_LOGI(TAG, "Deep sleep for %u seconds", REPORT_INTERVAL_SECONDS);
    esp_sleep_enable_timer_wakeup(sleep_us);
    esp_deep_sleep_start();
}

static bool wait_for_join(uint32_t timeout_s)
{
    int64_t start_us = esp_timer_get_time();
    int64_t timeout_us = (int64_t)timeout_s * 1000000LL;

    while (!s_join_done && !s_join_failed && (esp_timer_get_time() - start_us < timeout_us)) {
        os_runloop_once();
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    return s_join_done && !s_join_failed;
}

static bool wait_for_tx_complete(uint32_t timeout_s)
{
    int64_t start_us = esp_timer_get_time();
    int64_t timeout_us = (int64_t)timeout_s * 1000000LL;

    while (!s_tx_done && (esp_timer_get_time() - start_us < timeout_us)) {
        os_runloop_once();
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    return s_tx_done;
}

static void configure_region_channels(void)
{
#if defined(CFG_us915) || defined(CFG_au915)
    uint8_t sb = (LORA_SUBBAND >= 1 && LORA_SUBBAND <= 8) ? LORA_SUBBAND : 1;
    uint8_t subband_idx = (uint8_t)(sb - 1);

    // Apply 125kHz sub-band and force matching 500kHz channel.
    LMIC_selectSubBand(subband_idx);
    for (int ch = 64; ch < 72; ch++) {
        LMIC_disableChannel((u1_t)ch);
    }
    LMIC_enableChannel((u1_t)(64 + subband_idx));
    ESP_LOGI(TAG, "Configured US915 sub-band %u", (unsigned)sb);
#endif
}

static bool lora_init_session(void)
{
    os_init();
    LMIC_reset();
    LMIC_setClockError((MAX_CLOCK_ERROR * LORA_CLOCK_ERROR_PERCENT) / 100);
    configure_region_channels();

    LMIC_setLinkCheckMode(0);
    LMIC_setAdrMode(0);
    LMIC_setDrTxpow(LORA_TX_DATARATE, LORA_TX_POWER_DBM);

    if (s_rtc_session.magic == SESSION_MAGIC &&
        s_rtc_session.version == SESSION_VERSION &&
        s_rtc_session.valid == 1) {
        LMIC_setSession(s_rtc_session.netid,
                        s_rtc_session.devaddr,
                        (u1_t *)s_rtc_session.nwk_key,
                        (u1_t *)s_rtc_session.art_key);
        LMIC.seqnoUp = s_rtc_session.seqno_up;
        LMIC.seqnoDn = s_rtc_session.seqno_dn;
        ESP_LOGI(TAG, "Restored OTAA session from RTC (devaddr=0x%08" PRIx32 ", up=%" PRIu32 ")",
                 s_rtc_session.devaddr,
                 s_rtc_session.seqno_up);
        return true;
    }

    ESP_LOGI(TAG, "Starting OTAA join");
    s_join_done = false;
    s_join_failed = false;
    LMIC_startJoining();

    if (!wait_for_join(LORA_JOIN_TIMEOUT_SECONDS)) {
        ESP_LOGW(TAG, "OTAA join timeout/failure");
        return false;
    }

    return true;
}

static void build_payload(uint8_t out[6], float temperature_c, float humidity_rh, uint16_t battery_mv)
{
    int16_t t_centi = (int16_t)lroundf(temperature_c * 100.0f);
    uint16_t h_centi = (uint16_t)lroundf(humidity_rh * 100.0f);

    out[0] = (uint8_t)((t_centi >> 8) & 0xFF);
    out[1] = (uint8_t)(t_centi & 0xFF);
    out[2] = (uint8_t)((h_centi >> 8) & 0xFF);
    out[3] = (uint8_t)(h_centi & 0xFF);
    out[4] = (uint8_t)((battery_mv >> 8) & 0xFF);
    out[5] = (uint8_t)(battery_mv & 0xFF);
}

void app_main(void)
{
    radio_power_pins_init();
    radio_power_set_awake();
    vTaskDelay(pdMS_TO_TICKS(LORA_RADIO_SETTLE_DELAY_MS));

    log_wiring_profile();

    float temperature_c = 0.0f;
    float humidity_rh = 0.0f;
    uint16_t battery_mv = 0;

    esp_err_t ret = shtc3_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHTC3 init failed: %s", esp_err_to_name(ret));
        enter_deep_sleep();
        return;
    }

    ret = shtc3_read(&temperature_c, &humidity_rh);
    shtc3_deinit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHTC3 read failed: %s", esp_err_to_name(ret));
        enter_deep_sleep();
        return;
    }

    ret = read_battery_mv(&battery_mv);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Battery read failed: %s", esp_err_to_name(ret));
        enter_deep_sleep();
        return;
    }

    ESP_LOGI(TAG, "Sensor temp=%.2f C humidity=%.2f %%RH", temperature_c, humidity_rh);
    ESP_LOGI(TAG, "Battery=%u mV", (unsigned)battery_mv);

    uint8_t payload[6] = {0};
    build_payload(payload, temperature_c, humidity_rh, battery_mv);

    if (!lora_init_session()) {
        enter_deep_sleep();
        return;
    }

    if (LMIC.opmode & OP_TXRXPEND) {
        ESP_LOGW(TAG, "LMIC busy, skipping TX");
        enter_deep_sleep();
        return;
    }

    s_tx_done = false;
    LMIC_setTxData2(2, payload, sizeof(payload), 0);
    ESP_LOGI(TAG, "LoRa TX queued: %d bytes", (int)sizeof(payload));

    if (!wait_for_tx_complete(20)) {
        ESP_LOGW(TAG, "TX wait timeout, sleeping anyway");
    }

    enter_deep_sleep();
}
