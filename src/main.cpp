#include "hal/rtc_hal.h"
#include "rom/rtc.h"
//#include "esp_rom_uart.h"
#include <Arduino.h>

constexpr uint8_t LED_PIN = 8;
constexpr bool LED_LEVEL = LOW;

static const uint8_t RTC_RODATA_ATTR CNT_PINS[] = { 1, 2, 3, 4, 5 };
constexpr bool CNT_LEVEL = HIGH;

constexpr uint64_t DEBOUNCE_TIME = 150000 * 50 / 1000; // 50 ms. (RTC internal quartz 150 kHz)

volatile uint32_t RTC_DATA_ATTR cnts[sizeof(CNT_PINS) / sizeof(CNT_PINS[0])] = { 0 };
volatile uint64_t RTC_DATA_ATTR cnt_times[sizeof(CNT_PINS) / sizeof(CNT_PINS[0])] = { 0 };

static inline uint64_t rtc_raw_time() {
  SET_PERI_REG_MASK(RTC_CNTL_TIME_UPDATE_REG, RTC_CNTL_TIME_UPDATE);
  return (uint64_t)READ_PERI_REG(RTC_CNTL_TIME_LOW0_REG) | ((uint64_t)READ_PERI_REG(RTC_CNTL_TIME_HIGH0_REG) << 32);
}

static void IRAM_ATTR gpio_handler(void *arg) {
  uint64_t now;

  now = rtc_raw_time();
  if (gpio_get_level((gpio_num_t)CNT_PINS[(uint32_t)arg]) == CNT_LEVEL) {
    cnt_times[(uint32_t)arg] = now;
  } else {
    if (cnt_times[(uint32_t)arg]) {
      if (now - cnt_times[(uint32_t)arg] >= DEBOUNCE_TIME)
        ++cnts[(uint32_t)arg];
      cnt_times[(uint32_t)arg] = 0;
    }
  }
}

static void RTC_IRAM_ATTR gpio_set_pullup(uint8_t pin) {
//  REG_CLR_BIT(RTC_CNTL_PAD_HOLD_REG, BIT(pin));
  REG_CLR_BIT(GPIO_PIN_MUX_REG[pin], FUN_PD);
  REG_SET_BIT(GPIO_PIN_MUX_REG[pin], FUN_PU);
//  REG_SET_BIT(RTC_CNTL_PAD_HOLD_REG, BIT(pin));
}

static void RTC_IRAM_ATTR gpio_set_pulldown(uint8_t pin) {
//  REG_CLR_BIT(RTC_CNTL_PAD_HOLD_REG, BIT(pin));
  REG_CLR_BIT(GPIO_PIN_MUX_REG[pin], FUN_PU);
  REG_SET_BIT(GPIO_PIN_MUX_REG[pin], FUN_PD);
//  REG_SET_BIT(RTC_CNTL_PAD_HOLD_REG, BIT(pin));
}

void RTC_IRAM_ATTR wake_stub() {
  esp_default_wake_deep_sleep();

  if ((READ_PERI_REG(RTC_CNTL_SLP_WAKEUP_CAUSE_REG) & RTC_CNTL_WAKEUP_CAUSE) & RTC_GPIO_TRIG_EN) {
    uint32_t reg = READ_PERI_REG(RTC_CNTL_GPIO_WAKEUP_REG);
    uint64_t now = rtc_raw_time();

    for (uint8_t i = 0; i < sizeof(CNT_PINS) / sizeof(CNT_PINS[0]); ++i) {
      if ((reg & RTC_CNTL_GPIO_WAKEUP_STATUS) & ((uint32_t)1 << CNT_PINS[i])) {
        if (((reg >> (RTC_CNTL_GPIO_PIN0_INT_TYPE_S - CNT_PINS[i] * 3)) & RTC_CNTL_GPIO_PIN0_INT_TYPE_V) == GPIO_INTR_HIGH_LEVEL) {
          reg &= ~(RTC_CNTL_GPIO_PIN0_INT_TYPE_V << (RTC_CNTL_GPIO_PIN0_INT_TYPE_S - CNT_PINS[i] * 3));
          reg |= GPIO_INTR_LOW_LEVEL << (RTC_CNTL_GPIO_PIN0_INT_TYPE_S - CNT_PINS[i] * 3);
          gpio_set_pullup(CNT_PINS[i]);
          if (CNT_LEVEL) {
            cnt_times[i] = now;
          } else {
            if (cnt_times[i]) {
              if (now - cnt_times[i] >= DEBOUNCE_TIME)
                ++cnts[i];
              cnt_times[i] = 0;
            }
          }
        } else {
          reg &= ~(RTC_CNTL_GPIO_PIN0_INT_TYPE_V << (RTC_CNTL_GPIO_PIN0_INT_TYPE_S - CNT_PINS[i] * 3));
          reg |= GPIO_INTR_HIGH_LEVEL << (RTC_CNTL_GPIO_PIN0_INT_TYPE_S - CNT_PINS[i] * 3);
          gpio_set_pulldown(CNT_PINS[i]);
          if (CNT_LEVEL) {
            if (cnt_times[i]) {
              if (now - cnt_times[i] >= DEBOUNCE_TIME)
                ++cnts[i];
              cnt_times[i] = 0;
            }
          } else {
            cnt_times[i] = now;
          }
        }
      }
    }
    rtc_hal_gpio_clear_wakeup_pins();
//    rtc_hal_gpio_set_wakeup_pins();
    reg &= ~(RTC_CNTL_GPIO_WAKEUP_STATUS | RTC_CNTL_GPIO_WAKEUP_STATUS_CLR);
    WRITE_PERI_REG(RTC_CNTL_GPIO_WAKEUP_REG, reg);
  }

  if (! ((READ_PERI_REG(RTC_CNTL_SLP_WAKEUP_CAUSE_REG) & RTC_CNTL_WAKEUP_CAUSE) & RTC_TIMER_TRIG_EN)) {
    WRITE_PERI_REG(RTC_ENTRY_ADDR_REG, (uint32_t)&wake_stub);
    set_rtc_memory_crc();

    CLEAR_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_SLEEP_EN);
    SET_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_SLEEP_EN);
    while (true) {}
  }
}

void setup() {
  ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
  {
    gpio_config_t gpio_cfg;

    gpio_cfg.pin_bit_mask = 0;
    gpio_cfg.mode = GPIO_MODE_INPUT;
    gpio_cfg.pull_up_en = CNT_LEVEL ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE;
    gpio_cfg.pull_down_en = CNT_LEVEL ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    gpio_cfg.intr_type = GPIO_INTR_ANYEDGE;
    for (uint8_t i = 0; i < sizeof(CNT_PINS) / sizeof(CNT_PINS[0]); ++i) {
      gpio_cfg.pin_bit_mask |= (uint32_t)1 << CNT_PINS[i];
    }
    ESP_ERROR_CHECK(gpio_config(&gpio_cfg));
    for (uint8_t i = 0; i < sizeof(CNT_PINS) / sizeof(CNT_PINS[0]); ++i) {
      ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)CNT_PINS[i], gpio_handler, (void*)(uint32_t)i));
    }

    gpio_cfg.pin_bit_mask = (uint32_t)1 << LED_PIN;
    gpio_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_cfg.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&gpio_cfg));
    gpio_set_level((gpio_num_t)LED_PIN, LED_LEVEL);
  }

  Serial.begin(115200);
  Serial.print("Counters");
  for (uint8_t i = 0; i < sizeof(CNT_PINS) / sizeof(CNT_PINS[0]); ++i) {
    if (i)
      Serial.print(',');
    Serial.printf(" #%i: %u", i + 1, cnts[i]);
  }
  Serial.println();
  delay(5000);

  esp_sleep_enable_timer_wakeup(15000000); // 15 sec.
  esp_set_deep_sleep_wake_stub(wake_stub);
  esp_deep_sleep_disable_rom_logging();
//  Serial.flush();
  for (uint8_t i = 0; i < sizeof(CNT_PINS) / sizeof(CNT_PINS[0]); ++i) {
//    gpio_intr_disable((gpio_num_t)CNT_PINS[i]);
    esp_deep_sleep_enable_gpio_wakeup((uint32_t)1 << CNT_PINS[i], gpio_get_level((gpio_num_t)CNT_PINS[i]) ? ESP_GPIO_WAKEUP_GPIO_LOW : ESP_GPIO_WAKEUP_GPIO_HIGH);
  }
  esp_deep_sleep_start();
}

void loop() {}
