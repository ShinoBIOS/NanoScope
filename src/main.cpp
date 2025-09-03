#include <Arduino.h>

#include "hardware/vreg.h"
#include "hardware/xosc.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/structs/pwm.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#include "pico/util/queue.h"
#include "pico/multicore.h"

#include "digital_write.h"
#include "74HC4053D.h"
#include "74HC4051D.h"
#include "pwm.pio.h"
#include "HC165.h"
#include "fps.h"

#include <TFT_eSPI.h>
#include <EEPROM.h>
#include "hardware/gpio.h"

#define SELECTED_COLOR RGB565(61, 184, 80)
#define BACK_COLOR RGB565(40, 40, 40)

#define RGB565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

#define AD9288_S2 44
#define AD9288_A_CLK 28
#define AD9288_B_CLK 27

#define TRIG_PWM_A 26
#define TRIG_PWM_B 29
#define TFT_WIDTH_WR TFT_HEIGHT
#define TFT_HEIGHT_WR TFT_WIDTH

#define OSIL_ZONE_X 0
#define OSIL_ZONE_Y 30
#define OSIL_ZONE_W TFT_WIDTH_WR
#define OSIL_ZONE_H TFT_HEIGHT_WR - (OSIL_ZONE_Y * 2)

#define MENU_ITEM_w 40
#define MENU_ITEM_P 5
#define MENU_ITEM_H OSIL_ZONE_Y - (MENU_ITEM_P * 2)
#define MENU_ITEMS_ZONE_H OSIL_ZONE_Y
#define MENU_ITEMS_ZONE_w TFT_WIDTH_WR

#define GRID_SIZE (OSIL_ZONE_H) / 10
#define GRID_STEP (GRID_SIZE) / 6

#define AD9288_LVLS_PER_GRID (255.f) / 10.f
#define AD9288_MV_PER_LV 4.015

#define CH_A_COLOR RGB565(238, 225, 72)
#define CH_B_COLOR RGB565(255, 40, 40)

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)       \
  ((byte) & 0x80 ? '1' : '0'),     \
      ((byte) & 0x40 ? '1' : '0'), \
      ((byte) & 0x20 ? '1' : '0'), \
      ((byte) & 0x10 ? '1' : '0'), \
      ((byte) & 0x08 ? '1' : '0'), \
      ((byte) & 0x04 ? '1' : '0'), \
      ((byte) & 0x02 ? '1' : '0'), \
      ((byte) & 0x01 ? '1' : '0')

#define SIZE(x) (sizeof(x) / sizeof(x[0]))
#define CURRENT_CAHNNEL (channels[selected_cahnnel])

const uint CAPTURE_PIN_BASE = 0;
const uint CAPTURE_PIN_COUNT = 18;
const uint CAPTURE_N_SAMPLES = 8192 / 2;

static constexpr inline uint bits_packed_per_word(uint pin_count)
{
  // If the number of pins to be sampled divides the shift register size, we
  // can use the full SR and FIFO width, and push when the input shift count
  // exactly reaches 32. If not, we have to push earlier, so we use the FIFO
  // a little less efficiently.
  const uint SHIFT_REG_WIDTH = 32;
  return SHIFT_REG_WIDTH - (SHIFT_REG_WIDTH % pin_count);
}

enum
{
  DIV_1X = 4,
  DIV_2X = 6,
  DIV_5X = 0,
  DIV_10X = 3,
};

#define DIV_1X_REAL_K 1.1
#define DIV_10X_REAL_K 11.2

const struct
{
  const char *text;
  float div;
  float k;
  uint8_t sdiv;
  uint8_t _10x;
} sdiv_table[] = {{"5mV", 0.5, 10, DIV_1X, 0}, {"10mV", 0.5 * DIV_1X_REAL_K, 5, DIV_1X, 0}, {"25mV", 0.5 * DIV_1X_REAL_K, 2, DIV_1X, 0}, {"50mV", 0.5 * DIV_1X_REAL_K, 1, DIV_1X, 0}, {"100mV", 1.98 * 0.5 * DIV_1X_REAL_K, 1, DIV_2X, 0}, {"256mV", 5 * 0.5 * DIV_1X_REAL_K, 1, DIV_5X, 0}, {"1V", DIV_10X_REAL_K, 1.15, DIV_1X, 1}, {"2V", 1.98 * DIV_10X_REAL_K, 1.075, DIV_2X, 1}, {"6V", 5 * DIV_10X_REAL_K, 0.96, DIV_5X, 1}, {"12V", 10 * DIV_10X_REAL_K, 1.04, DIV_10X, 1}};

const char *trig_source_table[] = {"CHA", "CHB"};
const char *sync_table[] = {"AUTO", "NORM", "TRIG"};
const char *ch_table[] = {"CHA", "CHB"};
// const char *fdiv_table[] = {"1X", "10X"};
const char *ac_dc_table[] = {"DC", "AC"};

const struct
{
  const char *text;
  double div;
  uint8_t step = 1;
} time_div_table[] = {{"10ms", 0.01}, {"1ms", 0.001}, {"500us", 0.0005}, {"100us", 0.0001}, {"50us", 0.00005}, {"25us", 0.000025}, {"10us", 0.00001}, {"1us", 0.000001}, {"500ns", 5.e-7}, {"200ns", 2.e-7}, {"180ns", 1.8e-7}, {"90ns", 1.8e-7, 2}, {"45ns", 1.8e-7, 4}};

const uint TOTAL_SAMPLES_BITS = (CAPTURE_N_SAMPLES * CAPTURE_PIN_COUNT) + (bits_packed_per_word(CAPTURE_PIN_COUNT) - 1);
const uint BUF_SIZE_WORDS = TOTAL_SAMPLES_BITS / bits_packed_per_word(CAPTURE_PIN_COUNT);
const uint cpu_freq_khz = 200 * 1000;

auto_init_mutex(_mutex);
void __not_in_flash_func(trig_intr_cb)(uint p, uint32_t m);

struct message
{
  uint8_t ready = 0;
  char content[64];
  uint16_t time_to_print = 2000;
  uint32_t start_print_time = 0;
};

#define MAX_MESSAGES 4

message messages[MAX_MESSAGES];

enum
{
  CH_A,
  CH_B,
  CH_COUNT
};

enum
{
  AUTO_SYNC,
  NORM_SYNC,
  TRIG_SYNC
};

enum
{
  TDIV_10ms,
  TDIV_1ms,
  TDIV_500us,
  TDIV_100us,
  TDIV_50us,
  TDIV_25us,
  TDIV_1us,
  TDIV_500ns,
  TDIV_200ns,
};

struct channel_info
{
  uint8_t ch;
  uint16_t color;
  uint8_t _sync_mode = AUTO_SYNC;
  uint8_t is_triggered = 0;
  int16_t offset_x = 0;
  uint8_t visible = 1;
  uint8_t is_ac = 0;
  uint8_t is_10x = 0;
  uint8_t sdiv = 6;
  uint8_t is_trig_fall = 0;
  uint8_t measurement_visible = 1;
  uint8_t trig_source;
};

typedef struct
{
  uint32_t buffer[BUF_SIZE_WORDS];
  uint8_t is_triggered[CH_COUNT];
  // uint32_t freq[CH_COUNT];

} queue_entry_t;

channel_info channels[] = {
    channel_info{CH_A, CH_A_COLOR, .trig_source = CH_A},
    channel_info{CH_B, CH_B_COLOR, .trig_source = CH_B},
};

enum
{
  UP_ACTION,
  DOWN_ACTION,
  LEFT_ACTION,
  RIGHT_ACTION,
};

struct menu_item
{
  char text[32];
  char info[32];

  void (*function)(menu_item *, uint8_t);
  void (*render)(menu_item *, uint16_t, uint16_t, uint16_t, uint16_t);
  uint32_t data;
  uint8_t is_top;
  uint8_t hold = 0;
  uint8_t selected = 0;
};

// volatile uint32_t ch_freq[CH_COUNT];
volatile uint32_t trigged_count[CH_COUNT];
volatile uint8_t is_triggered[CH_COUNT];
volatile uint8_t capture = 0;

HC4053D hc4053_f_div{30, 31, NULL};
HC4053D hc4053_ac_dc{39, 38, NULL};

HC4051D hc4051_s_div_ina{32, 33, 34};
HC4051D hc4051_s_div_inb{35, 36, 37};

PioPwm pwm_trig_a{pio0, 29};
PioPwm pwm_trig_b{pio0, 26};

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library
TFT_eSprite spr = TFT_eSprite(&tft);
uint16_t *spr_buffer;

HC165 hc165{20, 42, 41};

uint8_t overlay_oscil_max_count = 10;
uint8_t selected_cahnnel = 0;
dma_channel_hw_t *mdma_hw;
queue_t _queue;
uint8_t hold = 0;
PIO ad9288_pio = pio0;
uint ad9288_sm;
uint ad9288_dma_chan;
uint ad9288_offset;
uint8_t tdiv = TDIV_1us;
uint8_t require_update_trig_front = 0;
// PIO trig_pio = pio0;
// uint trig_sm;

struct
{
  uint8_t min_ad9288_v = 0;
  uint8_t max_ad9288_v = 0;
  uint32_t freq = 0;
  uint32_t freq_tmp = 0;
  uint8_t freq_count = 0;
  uint16_t mean = 0;
} signal_metric[CH_COUNT];

int16_t ad9288_to_voltage(uint8_t ch, uint8_t x);
void set_time_for_grid(uint8_t);
void ch_set_fdiv(uint8_t ch, uint8_t _10x);

void ch_set_sdiv(uint8_t ch, uint8_t div)
{
  ch_set_fdiv(ch, sdiv_table[div]._10x);

  if (ch == CH_A)
  {
    hc4051_s_div_ina.set_channel(sdiv_table[div].sdiv);
  }
  else
  {
    hc4051_s_div_inb.set_channel(sdiv_table[div].sdiv);
  }

  channels[ch].sdiv = div;
}

void ch_set_fdiv(uint8_t ch, uint8_t _10x)
{
  hc4053_f_div.set_channel_level(ch, !_10x);
  channels[ch].is_10x = _10x;
}

void ch_set_ac_dc(uint8_t ch, uint8_t ac)
{
  hc4053_ac_dc.set_channel_level(ch, !ac);
  channels[ch].is_ac = ac;
}

void ch_set_trig_front(uint8_t ch, uint8_t falling)
{
  // CoreMutex m(&_mutex);

  if (ch != CH_A)
    return;

  uint pin = ch == CH_A ? 16 : 17;

  uint32_t mask = falling ? GPIO_IRQ_EDGE_FALL : GPIO_IRQ_EDGE_RISE;

  gpio_set_irq_enabled_with_callback(pin, mask, true, trig_intr_cb);
}

void draw_menu_item_default(menu_item *item, uint16_t x, uint16_t y)
{
  spr.setTextColor(TFT_BLACK);
  uint16_t color = RGB565(238, 225, 72);

  if (item->hold)
  {
    if (item->selected)
    {
      color = SELECTED_COLOR;
    }
    else
    {
      spr.fillRoundRect(x - 2, y + MENU_ITEM_P - 2, MENU_ITEM_w + 4, MENU_ITEM_H + 4, 5, SELECTED_COLOR);
    }
  }

  spr.fillRoundRect(x, y + MENU_ITEM_P, MENU_ITEM_w, MENU_ITEM_H, 5, color);
  spr.setCursor(x + 5, y + MENU_ITEM_P + 2);
  spr.print(item->text);
}

void item_action_next(menu_item *item, uint8_t action, uint16_t add = 1)
{
  if (LEFT_ACTION == action)
  {
    item->data = item->data - add;
  }
  else if (RIGHT_ACTION == action)
  {
    item->data = item->data + add;
  }
}

void ch_ac_dc_item_action(menu_item *item, uint8_t action)
{
  item_action_next(item, action);
  ch_set_ac_dc(selected_cahnnel, item->data % 2 == 0);
}

// void ch_fdiv_x_item_action(menu_item *item, uint8_t action)
// {
//   item_action_next(item, action);
//   ch_set_fdiv(selected_cahnnel, item->data % 2 == 0);
// }

void ch_sdiv_x_item_action(menu_item *item, uint8_t action)
{
  item_action_next(item, action);
  ch_set_sdiv(selected_cahnnel, item->data % SIZE(sdiv_table));
}

void change_ch_action(menu_item *item, uint8_t action)
{
  item_action_next(item, action);
  item->text[0] = {0};
  selected_cahnnel = item->data % 2 == 0;
  strcpy(item->text, ch_table[selected_cahnnel]);
}

void change_sync_mode_action(menu_item *item, uint8_t action)
{
  item_action_next(item, action);
  CURRENT_CAHNNEL._sync_mode = item->data % SIZE(sync_table);
}

void change_sync_source_action(menu_item *item, uint8_t action)
{
  item_action_next(item, action);
  CURRENT_CAHNNEL.trig_source = item->data % SIZE(trig_source_table);
}

void change_trig_front_action(menu_item *item, uint8_t action)
{
  item_action_next(item, action);
  channels[selected_cahnnel].is_trig_fall = item->data % 2 == 0;

  CoreMutex m(&_mutex);
  require_update_trig_front = 1;
}

void hold_action(menu_item *item, uint8_t action)
{
  item_action_next(item, action);

  hold = item->data % 2 == 0;
}

void hold_render(menu_item *item, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  uint16_t color = RGB565(238, 225, 72);

  item->text[0] = {0};

  if (hold)
  {
    strcpy(item->text, "HOLD");
  }
  else
  {
    strcpy(item->text, "RUN");
  }

  if (item->hold)
  {
    if (item->selected)
    {
      color = SELECTED_COLOR;
    }
    else
    {
      spr.fillRoundRect(x - 2, y + MENU_ITEM_P - 2, MENU_ITEM_w + 4, MENU_ITEM_H + 4, 5, SELECTED_COLOR);
    }
  }

  else if (hold)
  {
    color = TFT_RED;
  }
  else
  {
    color = RGB565(91, 214, 110);
  }

  spr.fillRoundRect(x, y + MENU_ITEM_P, MENU_ITEM_w, MENU_ITEM_H, 5, color);
  spr.setCursor(x + 5, y + MENU_ITEM_P + 2);
  spr.print(item->text);
}

void change_offestx_action(menu_item *item, uint8_t action)
{
  if (LEFT_ACTION == action)
  {
    CURRENT_CAHNNEL.offset_x -= 100;
  }
  else if (RIGHT_ACTION == action)
  {
    CURRENT_CAHNNEL.offset_x += 100;
  }
  else if (DOWN_ACTION == action)
  {
    CURRENT_CAHNNEL.offset_x = 0;
  }

  sprintf(item->info, "x-offset: %d", (int)CURRENT_CAHNNEL.offset_x);
}

void ch_visible_item_action(menu_item *item, uint8_t action)
{
  item_action_next(item, action);
  CURRENT_CAHNNEL.visible = item->data % 2 == 0;
}

void change_measurement_vis(menu_item *item, uint8_t action)
{
  item_action_next(item, action);
  CURRENT_CAHNNEL.measurement_visible = item->data % 2 == 0;
}

void render_ac_dc(menu_item *item, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  item->text[0] = {0};
  strcpy(item->text, ac_dc_table[CURRENT_CAHNNEL.is_ac]);
  draw_menu_item_default(item, x, y);
}

// void render_fdiv(menu_item *item, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
// {
//   item->text[0] = {0};
//   strcpy(item->text, fdiv_table[CURRENT_CAHNNEL.is_10x]);
//   draw_menu_item_default(item, x, y);
// }

void render_sdiv(menu_item *item, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  item->text[0] = {0};
  strcpy(item->text, sdiv_table[CURRENT_CAHNNEL.sdiv].text);
  draw_menu_item_default(item, x, y);
}

void render_vis(menu_item *item, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  item->text[0] = {0};

  if (CURRENT_CAHNNEL.visible)
  {
    strcpy(item->text, "VIS");
  }
  else
  {
    strcpy(item->text, "HID");
  }

  draw_menu_item_default(item, x, y);
}

void render_sync_mode(menu_item *item, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  item->text[0] = {0};
  strcpy(item->text, sync_table[CURRENT_CAHNNEL._sync_mode]);
  draw_menu_item_default(item, x, y);
}

void render_trig_source(menu_item *item, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  item->text[0] = {0};
  strcpy(item->text, trig_source_table[CURRENT_CAHNNEL.trig_source]);
  draw_menu_item_default(item, x, y);
}

void render_tdiv(menu_item *item, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  item->text[0] = {0};
  strcpy(item->text, time_div_table[tdiv].text);
  draw_menu_item_default(item, x, y);
}

void render_trig_front(menu_item *item, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  item->text[0] = {0};

  if (CURRENT_CAHNNEL.is_trig_fall)
  {
    strcpy(item->text, "FALL");
  }
  else
  {
    strcpy(item->text, "UP");
  }

  draw_menu_item_default(item, x, y);
}

void render_ch(menu_item *item, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  item->text[0] = {0};

  if (selected_cahnnel == CH_A)
  {
    strcpy(item->text, "CHA");
  }
  else
  {
    strcpy(item->text, "CHB");
  }

  draw_menu_item_default(item, x, y);
}

void format_hz(uint32_t hertz, char *output)
{
  char float_str[16]{0};

  if (hertz >= 1e9)
  {
    dtostrf(double(hertz) / 1e9, 4, 2, float_str);
    sprintf(output, "%s GHZ", float_str);
  }
  else if (hertz >= 1e6)
  {
    dtostrf(double(hertz) / 1e6, 4, 2, float_str);
    sprintf(output, "%s MHz", float_str);
  }
  else if (hertz >= 1e3)
  {
    dtostrf(double(hertz) / 1e3, 4, 2, float_str);
    sprintf(output, "%s KHz", float_str);
  }
  else
  {
    dtostrf(double(hertz), 4, 2, float_str);
    sprintf(output, "%s Hz", float_str);
  }
}

void measurement_render(menu_item *item, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  item->text[0] = {0};

  auto metric = signal_metric[selected_cahnnel];
  char buf[256]{0};
  int vmin, vmax;

  if (!CURRENT_CAHNNEL.measurement_visible)
  {
    strcpy(item->text, "HID");
    goto render;
  }
  else
  {
    strcpy(item->text, "VIS");
  }

  spr.setTextColor(TFT_WHITE);
  spr.setCursor(0, OSIL_ZONE_Y);

  vmin = (int)ad9288_to_voltage(selected_cahnnel, metric.min_ad9288_v);
  vmax = (int)ad9288_to_voltage(selected_cahnnel, metric.max_ad9288_v);

  spr.printf(" Vmin %d\n", vmin);
  spr.printf(" Vmax %d\n", vmax);
  spr.printf(" Vmeam %d\n", (int)ad9288_to_voltage(selected_cahnnel, metric.mean));
  spr.printf(" Vpkpk %d\n", vmax - vmin);

  // spr.printf(" Vmeam %d\n", (int)metric.mean);
  format_hz(metric.freq, buf);
  spr.printf(" Freq %s\n", buf);

render:
  draw_menu_item_default(item, x, y);
}

void trig_level_render(menu_item *item, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  uint8_t tl = 0;

  if (selected_cahnnel == CH_A)
  {
    tl = pwm_trig_a.gel_level();
  }
  else
  {
    tl = pwm_trig_b.gel_level();
  }

  uint16_t cent_y = OSIL_ZONE_H - (float(tl) * (float(OSIL_ZONE_H) / (255.f * 0.65)));
  cent_y += OSIL_ZONE_Y;

  uint16_t color = RGB565(238, 225, 72);

  if (item->hold)
  {
    if (item->selected)
    {
      color = SELECTED_COLOR;
    }
    else
    {
      spr.fillTriangle(TFT_WIDTH_WR - 12, cent_y, (OSIL_ZONE_W + OSIL_ZONE_X - 2), cent_y - 10, (OSIL_ZONE_W + OSIL_ZONE_X - 2), cent_y + 10, SELECTED_COLOR);
    }
  }

  spr.fillTriangle(TFT_WIDTH_WR - 10, cent_y, (OSIL_ZONE_W + OSIL_ZONE_X - 4), cent_y - 8, (OSIL_ZONE_W + OSIL_ZONE_X - 4), cent_y + 8, color);

  // Serial.printf("trig y %d\n", cent_y);

  // draw_menu_item_default(item, x, y);
}

void ch_time_div_item_action(menu_item *item, uint8_t action)
{
  item_action_next(item, action);
  item->text[0] = {0};
  set_time_for_grid(item->data % SIZE(time_div_table));
}

void change_trig_level(menu_item *item, uint8_t action)
{
  item_action_next(item, action, 5);
  uint8_t ntl = item->data % 165;

  if (selected_cahnnel == CH_A)
  {
    pwm_trig_a.sel_level(ntl);
  }
  else
  {
    pwm_trig_b.sel_level(ntl);
  }
}

void messages_render(menu_item *item, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  static int8_t mes_idx = -1;
  uint32_t time = to_ms_since_boot(get_absolute_time());

  if (mes_idx == -1)
  {
    for (size_t i = 0; i < MAX_MESSAGES; i++)
    {
      if (messages[i].ready)
      {
        mes_idx = i;
        messages[mes_idx].start_print_time = time;
        break;
      }
    }
  }

  if (mes_idx == -1)
    return;

  if (time - messages[mes_idx].start_print_time > messages[mes_idx].time_to_print)
  {
    messages[mes_idx].ready = false;
    mes_idx = -1;
    return;
  }

  uint16_t x1 = TFT_WIDTH_WR * 0.2;
  uint16_t x2 = TFT_WIDTH_WR * 0.85;
  uint16_t y1 = TFT_HEIGHT_WR * 0.65;
  uint16_t y2 = TFT_HEIGHT_WR * 0.85;

  spr.fillRoundRect(x1 - 2, y1 - 2, (x2 - x1) + 4, (y2 - y1) + 4, 5, SELECTED_COLOR);
  spr.fillRoundRect(x1, y1, x2 - x1, y2 - y1, 5, BACK_COLOR);

  int16_t text_width = spr.textWidth(messages[mes_idx].content);

  spr.setCursor((TFT_WIDTH_WR * 0.5) - (text_width / 2), TFT_HEIGHT_WR * 0.7);
  spr.printf(messages[mes_idx].content);
}

void info_render(menu_item *item, uint16_t x, uint16_t y, uint16_t w, uint16_t h);

menu_item menu_items[] = {
    menu_item{"CH1", "select channel", change_ch_action, render_ch, NULL, 0},
    menu_item{"1X", "V/div", ch_sdiv_x_item_action, render_sdiv, NULL, 0},
    menu_item{"1us", "t/div", ch_time_div_item_action, render_tdiv, NULL, 0},
    menu_item{"VIS", "channel visible", ch_visible_item_action, render_vis, NULL, 0},

    menu_item{"TRIG", "trig mode", change_sync_mode_action, render_sync_mode, NULL, 0},
    menu_item{"FALL", "sync front", change_trig_front_action, render_trig_front, NULL, 0},
    // menu_item{"AC", ch_ac_dc_item_action, render_ac_dc, NULL, 0},
    // menu_item{"10X", ch_fdiv_x_item_action, render_fdiv, NULL, 0},

    menu_item{"RUN", "", hold_action, hold_render, NULL, 0},

    menu_item{"CHA", "trig source", change_sync_source_action, render_trig_source, NULL, 1},

    menu_item{"<->", "x-offset", change_offestx_action, NULL, NULL, 1},
    menu_item{"measurement", "meas vis", change_measurement_vis, measurement_render, NULL, 1},
    menu_item{"tl", "trig level", change_trig_level, trig_level_render, NULL, 1},

    menu_item{"info", "", NULL, info_render, NULL, 1},

    menu_item{"messages", "", NULL, messages_render, NULL, 0},
};

void info_render(menu_item *item, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  for (size_t i = 0; i < SIZE(menu_items); i++)
  {
    if (menu_items[i].hold || menu_items[i].selected)
    {
      int16_t text_width = spr.textWidth(menu_items[i].info);
      spr.setTextColor(TFT_WHITE);
      spr.setCursor(TFT_WIDTH_WR - text_width - 10, 5);
      spr.printf(menu_items[i].info);
      break;
    }
  }
}

bool push_message(char content[64], uint16_t time_to_print)
{
  for (size_t i = 0; i < MAX_MESSAGES; i++)
  {
    if (!messages[i].ready)
    {
      strcpy(messages[i].content, content);
      messages[i].ready = true;
      messages[i].time_to_print = time_to_print;
      return true;
    }
  }

  return false;
}

void __not_in_flash_func(logic_analyser_arm)(PIO pio, uint sm, uint dma_chan, uint32_t *capture_buf, size_t capture_size_words)
{
  pio_sm_set_enabled(pio, sm, 0);
  pio_sm_clear_fifos(pio, sm);
  pio_sm_restart(pio, sm);

  dma_channel_config c = dma_channel_get_default_config(dma_chan);
  channel_config_set_read_increment(&c, 0);
  channel_config_set_write_increment(&c, 1);
  channel_config_set_dreq(&c, pio_get_dreq(pio, sm, 0));
  // channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

  dma_channel_configure(dma_chan, &c,
                        capture_buf,        // Destination pointer
                        &pio->rxf[sm],      // Source pointer
                        capture_size_words, // Number of transfers
                        1                   // Start immediately
  );

  pio_sm_set_enabled(pio, sm, 1);
}

struct pwm_calc_result
{
  double dutycycle;
  double frequency;
  int div;
  int steps;
  int high;
};

pwm_calc_result calc_pwm(double frequency, double dutycycle)
{
  uint32_t pwm_clock = clock_get_hz(clk_sys);

  double norm = pwm_clock / frequency;
  int div = (int)(norm / 65536) + 1;
  double newf = pwm_clock / div;
  int steps = (int)(newf / frequency);
  int high = (int)(dutycycle * steps / 100.0);

  return {dutycycle, frequency, div, steps, high};
}

void init_ad9288_clk_ch(uint8_t clk_p)
{
  gpio_set_function(clk_p, GPIO_FUNC_PWM);

  // uint slice_num = pwm_gpio_to_slice_num(clk_p);
  // pwm_config config = pwm_get_default_config();
  // pwm_set_phase_correct(slice_num, 0);
  // pwm_config_set_clkdiv(&config, 1.f);
  // pwm_config_set_wrap(&config, 1);
  // pwm_init(slice_num, &config, 1);
  // pwm_set_chan_level(slice_num, pwm_gpio_to_channel(clk_p), 1);
}

void ad9288_ch_set_fdiv(uint8_t ch, float fdiv, uint32_t freq)
{
  uint8_t clk_p = ch ? AD9288_B_CLK : AD9288_A_CLK;
  // uint slice_num = pwm_gpio_to_slice_num(clk_p);
  // pwm_set_clkdiv(slice_num, fdiv);

  gpio_set_function(clk_p, GPIO_FUNC_PWM);

  pwm_calc_result par = calc_pwm(freq, 50);

  uint slice = pwm_gpio_to_slice_num(clk_p);
  pwm_set_clkdiv_mode(slice, PWM_DIV_FREE_RUNNING);
  pwm_set_clkdiv_int_frac(slice, par.div, 0);
  pwm_set_wrap(slice, par.steps);
  pwm_set_gpio_level(clk_p, par.high);
  pwm_set_enabled(slice, true);
}

void ad9288_set_sr(uint32_t sr)
{
  Serial.printf("Set sampling rate %d\n", int(sr));
  uint32_t pwm_clock = clock_get_hz(clk_sys);
  Serial.printf("PWM Clock Frequency: %d kHz\n", pwm_clock / 1000);

  if (pwm_clock < sr || sr < 0)
  {
    Serial.printf("Skip sampling rate %d\n", int(sr));
    return;
  }

  float fdiv = float(pwm_clock) / sr;
  pio_sm_set_clkdiv(ad9288_pio, ad9288_sm, fdiv);
  // pio_sm_set_clkdiv(trig_pio, trig_sm, fdiv);

  // if (sr > 1 * 1000 * 1000)
  // {
  ad9288_ch_set_fdiv(CH_A, fdiv, sr);
  ad9288_ch_set_fdiv(CH_B, fdiv, sr);
  // }
}

void set_time_for_grid(uint8_t tdiv_)
{
  tdiv = tdiv_;
  uint32_t sr = double(1) / (time_div_table[tdiv].div / double(GRID_SIZE));

  if (sr % 2 == 1)
    sr++;

  ad9288_set_sr(sr);
}

void __isr __not_in_flash_func(trig_intr_cb)(uint p, uint32_t m)
{

  gpio_set_irq_enabled(p, m, false);

  // uint32_t addr = mdma_hw->write_addr;
  uint8_t ch = p == 17;

  // ++trigged_count[p == 17];

  if (capture)
  {
    if (!is_triggered[ch])
    {
      is_triggered[ch] = 1;
    }
  }
}

void draw_grid()
{
  for (int i = 0; i < OSIL_ZONE_H; i += GRID_STEP)
  {
    for (int j = -1; j < OSIL_ZONE_W; j += GRID_SIZE)
    {
      spr.drawPixel(j, i + OSIL_ZONE_Y, TFT_WHITE);
    }
  }

  for (int i = 0; i < OSIL_ZONE_H; i += GRID_SIZE)
  {
    for (int j = -1; j < OSIL_ZONE_W; j += GRID_STEP)
    {
      spr.drawPixel(j, i + OSIL_ZONE_Y, TFT_WHITE);
    }
  }

  spr.drawFastVLine(OSIL_ZONE_W / 2, OSIL_ZONE_Y, OSIL_ZONE_H, TFT_WHITE);
  spr.drawFastHLine(0, OSIL_ZONE_Y + ((OSIL_ZONE_H) / 2), OSIL_ZONE_W, TFT_WHITE);
}

int16_t ad9288_to_voltage(uint8_t ch, uint8_t x)
{
  float y = (x - 127.f) * 4.01569;
  channel_info *info = &channels[ch];

  y = y * sdiv_table[info->sdiv].div;

  return y;
}

int8_t null_offset[CH_COUNT][SIZE(sdiv_table)] = {0};

bool is_null_calc = false;

int8_t get_channel_null_offset(uint8_t ch)
{
  if (is_null_calc)
    return 0;

  channel_info *info = &channels[ch];

  return null_offset[ch][info->sdiv];
}

bool null_auto_calc_cb(struct repeating_timer *t)
{
  channel_info *info;
  bool done = false;

  for (size_t ch = 0; ch < CH_COUNT; ch++)
  {
    info = &channels[ch];

    null_offset[ch][info->sdiv] = 127 - int16_t(signal_metric[ch].mean);

    if (info->sdiv < SIZE(sdiv_table) - 1)
      ch_set_sdiv(ch, info->sdiv + 1);
    else
      done = true;
  }

  if (done)
  {
    is_null_calc = false;
    EEPROM.put(0, null_offset);
    EEPROM.commit();
    push_message("Null calc done.", 2000);
    return false;
  }

  return true;
}

void null_auto_calc()
{
  if (is_null_calc)
    return;

  push_message("Null calc...", 2000);

  static struct repeating_timer timer;
  is_null_calc = true;

  for (size_t i = 0; i < CH_COUNT; i++)
  {
    ch_set_sdiv(i, 0);
  }

  add_repeating_timer_ms(500, null_auto_calc_cb, NULL, &timer);
}

inline uint8_t __not_in_flash_func(reverse_bits)(uint8_t b)
{
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

// #include <stdint.h>
// #include <stdio.h>

// // Функция для линейной интерполяции
// uint8_t linear_interpolate(uint8_t y0, uint8_t y1, float t) {
//     return y0 + (y1 - y0) * t;
// }

// // Функция для интерполяции массива
// void interpolate_array(uint8_t *input, uint8_t *output, int input_size, int output_size) {
//     float scale = (float)(input_size - 1) / (output_size - 1);

//     for (int i = 0; i < output_size; i++) {
//         float index = i * scale;
//         int lower_index = (int)index;
//         int upper_index = lower_index + 1;

//         if (upper_index >= input_size) {
//             upper_index = input_size - 1;
//         }

//         float t = index - lower_index;
//         output[i] = linear_interpolate(input[lower_index], input[upper_index], t);
//     }
// }

void __not_in_flash_func(parse_capture_buf)(const uint8_t *buf, uint32_t n_samples, uint8_t ch, int32_t trig_index)
{
  channel_info *info = &channels[ch];

  switch (info->_sync_mode)
  {
  case AUTO_SYNC:
    /* code */
    break;
  case NORM_SYNC:
    if (!info->is_triggered)
    {
      return;
    }
    break;
  case TRIG_SYNC:
    if (!info->is_triggered)
    {
      return;
    }
    break;
  default:
    break;
  }

  // uint16_t trig_index = ch;

  // // if (info->is_triggered)
  // // {
  // //   trig_index = info->is_triggered;

  // //   if (trig_index % CH_COUNT != ch)
  // //   {
  // //     ++trig_index;
  // //   }
  // // }
  // // else
  // // {
  // //   trig_index = ch;
  // // }

  // int16_t n_trig_index = trig_index; // + info->offset_x;

  // if (n_trig_index > 0 && n_trig_index < n_samples)
  // {
  //   trig_index = n_trig_index;
  // }

  trig_index += info->offset_x;

  if (trig_index < 0)
  {
    trig_index = 0;
  }
  else if (trig_index > n_samples - 320)
  {
    trig_index = n_samples - 320;
  }

  float k = sdiv_table[info->sdiv].k;

#define GET_FROM_BUF(x) (ch == CH_A ? int16_t(reverse_bits(buf[x])) + get_channel_null_offset(CH_A) : int16_t(buf[x]) + get_channel_null_offset(CH_B))
#define AD9288_V_TO_Y(v) max(min(((OSIL_ZONE_H) - ((((v - 128) * k) + 128) * 0.705)) + OSIL_ZONE_Y, OSIL_ZONE_Y + OSIL_ZONE_H), OSIL_ZONE_Y)

  uint8_t ch_ls_v = AD9288_V_TO_Y(GET_FROM_BUF(trig_index));
  uint8_t min_v = GET_FROM_BUF(trig_index);
  uint8_t max_v = GET_FROM_BUF(trig_index);

  struct
  {
    uint32_t count = 0;
    int32_t value = 0;
  } mean;

  float ch_ls_x = 0;
  int16_t ch_v;
  uint8_t step = time_div_table[tdiv].step;

  // uint32_t rbit = 0;
  // bool lat = 0;

  int mi = int(trig_index) - (160 * (CH_COUNT + 2) / step);

  if (mi > 0)
  {
    trig_index = mi;
  }

  for (uint32_t sample = trig_index; sample < n_samples; sample += (CH_COUNT + 2))
  {
    if (ch_ls_x >= TFT_WIDTH_WR && ch_ls_x < 0)
      break;
    ch_v = GET_FROM_BUF(sample);

    // Serial.printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(buf[sample]));
    // Serial.println();

    mean.value += ch_v;
    mean.count++;
    if (min_v > ch_v)
      min_v = ch_v;
    if (max_v < ch_v)
      max_v = ch_v;
    ch_v = AD9288_V_TO_Y(ch_v);

    spr.drawLine(ch_ls_x, ch_ls_v, ch_ls_x + step, ch_v, info->color);
    ch_ls_x += step;
    ch_ls_v = ch_v;
  }

  signal_metric[ch].mean = mean.value / mean.count;
  signal_metric[ch].min_ad9288_v = min_v;
  signal_metric[ch].max_ad9288_v = max_v;
}

struct auto_scan_
{
  channel_info *channel;
  bool trigger_set;
} auto_scan__;

bool buzzer_state = 1;

bool buzzer_cb(struct repeating_timer *t)
{
  buzzer_state != buzzer_state;
  digital_write(43, buzzer_state);
  return 1;
}

bool auto_scan_cb(struct repeating_timer *t)
{
  auto metric = &signal_metric[auto_scan__.channel->ch];

  if (metric->max_ad9288_v > 245 || metric->min_ad9288_v < 15)
  {
    if (auto_scan__.channel->sdiv < SIZE(sdiv_table))
    {
      ch_set_sdiv(auto_scan__.channel->ch, auto_scan__.channel->sdiv + 1);
      return true;
    }
  }
  else if (metric->max_ad9288_v - metric->min_ad9288_v < 40)
  {
    if (auto_scan__.channel->sdiv > 0)
    {
      ch_set_sdiv(auto_scan__.channel->ch, auto_scan__.channel->sdiv - 1);
      return true;
    }
  }

  if (!auto_scan__.trigger_set)
  {
    uint8_t trigger_mean = (metric->max_ad9288_v + metric->min_ad9288_v) / 1.75;

    if (auto_scan__.channel->ch == CH_A)
    {
      pwm_trig_a.sel_level(trigger_mean * 0.6);
    }
    else
    {
      pwm_trig_b.sel_level(trigger_mean * 0.6);
    }

    auto_scan__.trigger_set = true;
    return true;
  }

  double T = double(1) / (double(metric->freq) * 2);
  double sub = 10;
  uint8_t time_div = 0;

  for (size_t i = 1; i < SIZE(time_div_table); ++i)
  {
    double s = time_div_table[i].div;
    double sub__ = abs(T - s);

    if (sub__ < sub)
    {
      sub = sub__;
      time_div = i;
    }
  }

  set_time_for_grid(time_div);

  // Serial.printf("Mean %d\n", metric->mean);
  // int16_t max_voltage = max(abs(metric->max_voltage), abs(metric->min_voltage));
  push_message("Auto done", 2000);

  return false;
}

void auto_scan()
{
  uint8_t ch = selected_cahnnel;

  auto_scan__.channel = &channels[ch];
  auto_scan__.trigger_set = false;

  push_message("Auto", 2000);

  static struct repeating_timer timer;
  add_repeating_timer_ms(500, auto_scan_cb, NULL, &timer);
}

inline void draw_menu()
{
  uint8_t len = sizeof(menu_items) / sizeof(menu_items[0]);

  uint16_t bottom_y = TFT_HEIGHT_WR - MENU_ITEMS_ZONE_H;

  spr.fillRect(0, 0, MENU_ITEMS_ZONE_w, MENU_ITEMS_ZONE_H, BACK_COLOR);
  spr.fillRect(0, bottom_y, MENU_ITEMS_ZONE_w, MENU_ITEMS_ZONE_H, BACK_COLOR);

  spr.setTextColor(TFT_BLACK);

  uint16_t bx = MENU_ITEM_P;
  uint16_t tx = MENU_ITEM_P;

  for (size_t i = 0; i < len; ++i)
  {
    menu_item *item = &menu_items[i];

    uint16_t y = item->is_top ? 0 : bottom_y;
    uint16_t &x = item->is_top ? tx : bx;

    if (item->render)
    {
      item->render(item, x, y, MENU_ITEM_w, MENU_ITEM_H);
      goto end_draw_item;
    }

    draw_menu_item_default(item, x, y);

  end_draw_item:
    x += MENU_ITEM_w + MENU_ITEM_P;
  }

  spr.drawFastHLine(0, MENU_ITEMS_ZONE_H - 1, MENU_ITEMS_ZONE_w, TFT_WHITE);

  spr.drawFastHLine(0, bottom_y, MENU_ITEMS_ZONE_w, TFT_WHITE);
}

void null_auto_calc();

inline void parse_input(uint32_t time)
{
  uint8_t button_code = hc165.get_byte();

  enum
  {
    BTN_BOTTOM = 2,
    BTN_RIGHT = 1,
    BTN_TOP = 0,
    BTN_MID = 3,
    BTN_LEFT = 4,

    BTN_NUM
  };

  static struct
  {
    uint32_t pressed_time;
    uint8_t is_press;
  } buttons[BTN_NUM];

  // Serial.printf("Button "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(button_code));
  // Serial.println();

  for (uint8_t i = 0; i < BTN_NUM; ++i)
  {
    if (bitRead(button_code, i + 1))
    {
      if (!buttons[i].is_press)
      {
        buttons[i].pressed_time = time;
      }

      buttons[i].is_press = true;
    }
    else
    {
      buttons[i].is_press = false;
      buttons[i].pressed_time = NULL;
    }
  }

  static int16_t hold_menu_item = 0;
  menu_item *item = &menu_items[hold_menu_item];
  item->hold = 1;

  static uint32_t last_button_time = 0;

  if (!button_code || time - last_button_time < 200)
  {
    return;
  }

  last_button_time = time;

  if (buttons[BTN_LEFT].is_press)
  {
    if (item->selected)
    {
      if (item->function)
        item->function(item, LEFT_ACTION);
    }
    else
    {
      item->hold = 0;

      hold_menu_item = hold_menu_item - 1;

      if (hold_menu_item < 0)
      {
        hold_menu_item = SIZE(menu_items) - 1;
      }
    }
  }
  else if (buttons[BTN_RIGHT].is_press)
  {
    if (item->selected)
    {
      if (item->function)
        item->function(item, RIGHT_ACTION);
    }
    else
    {
      item->hold = 0;

      hold_menu_item = hold_menu_item + 1;

      if (hold_menu_item >= SIZE(menu_items))
      {
        hold_menu_item = 0;
      }
    }
  }
  else if (buttons[BTN_BOTTOM].is_press && buttons[BTN_MID].is_press)
  {
    null_auto_calc();
  }
  else if (buttons[BTN_BOTTOM].is_press)
  {
    if (item->selected)
    {
      if (item->function)
        item->function(item, DOWN_ACTION);
    }
  }
  else if (buttons[BTN_MID].is_press)
  {
    item->selected = !item->selected;

    if ((time - buttons[BTN_MID].pressed_time) > 2000 && buttons[BTN_MID].pressed_time != NULL)
    {
      buttons[BTN_MID].pressed_time = 0;
      push_message("Shutdown...", 2000);
      delay(500);
      digital_write(45, 0);
    }
  }
  else if (buttons[BTN_TOP].is_press)
  {
    auto_scan();
  }
}

inline void tft_write()
{
  tft.startWrite();
  spr.pushSprite(0, 0);
  // tft.pushImageDMA(0, 0, TFT_WIDTH_WR, TFT_HEIGHT_WR, spr_buffer);
  tft.endWrite();
}

void init_tft()
{
  tft.init();
  uint8_t dma_done = tft.initDMA();
  tft.setRotation(3);

  tft.setTextWrap(1, 1);

  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0, 2);

  spr.setColorDepth(8);

  spr_buffer = (uint16_t *)spr.createSprite(TFT_WIDTH_WR, TFT_HEIGHT_WR);

  spr.setTextSize(1);
  spr.setTextColor(TFT_LIGHTGREY);
  spr.setCursor(0, 0, 2);

  spr.fillRect(0, 0, TFT_WIDTH_WR, TFT_HEIGHT_WR, TFT_WHITE);

  tft_write();
}

void __not_in_flash_func(calc_trig_index)(const uint8_t *buf, uint32_t n_samples, uint32_t *triggered_index, uint32_t *freq)
{
#define TRIG_NOISE_FIXER 6
  bool last_trig[TRIG_NOISE_FIXER][CH_COUNT];

  for (uint8_t i = 0; i < TRIG_NOISE_FIXER; ++i)
  {
    for (uint8_t c = 0; c < CH_COUNT; ++c)
    {
      last_trig[i][c] = buf[2] & uint8_t(c + 1);
    }
  }

  // uint32_t T[CH_COUNT]{0};
  // uint32_t T_tmp[CH_COUNT]{0};
  // uint32_t T_tmp_count[CH_COUNT]{0};

  for (uint8_t c = 0; c < CH_COUNT; ++c)
  {
    triggered_index[c] = c;
    freq[c] = 0;

    bool is_fall = channels[c].is_trig_fall;

    for (size_t i = 2; i < n_samples; i += (CH_COUNT + 2))
    {
      /* code */

      bool trig = buf[i] & uint8_t(c + 1);

      bool first_cond = trig == !is_fall;

      if (first_cond)
      {
        for (uint8_t i = TRIG_NOISE_FIXER / 2; i < TRIG_NOISE_FIXER; ++i)
        {
          first_cond = first_cond && last_trig[i][c] == !is_fall;
        }
      }

      bool second_cond = 1;

      for (uint8_t i = 0; i < TRIG_NOISE_FIXER / 2; ++i)
      {
        second_cond = second_cond && last_trig[i][c] == is_fall;
      }

      if (first_cond && second_cond)
      {
        if (int(triggered_index[c]) - (768) < 0)
        {
          triggered_index[c] = i - (CH_COUNT - c);

          // ++T_tmp_count[c];
          // T[c] = T_tmp[c];
          // T_tmp[c] = 0;
        }

        // T_tmp[c] = 0;

        ++freq[c];
      }
      // else
      // {
      // ++T_tmp[c];
      // }

      // last_last_last_trig[c] = last_last_trig[c];
      // last_last_trig[c] = last_trig[c];

      for (uint8_t i = 1; i < TRIG_NOISE_FIXER; ++i)
      {
        last_trig[i - 1][c] = last_trig[i][c];
      }

      last_trig[TRIG_NOISE_FIXER - 1][c] = trig;
    }
  }

  uint32_t sr = double(1) / (time_div_table[tdiv].div / double(GRID_SIZE));

  if (sr % 2 == 1)
    sr++;

  // Serial.printf("%d\n", (int)freq[CH_A]);

  double capture_time = (CAPTURE_N_SAMPLES) * (double(1) / double(sr));

  for (uint8_t i = 0; i < CH_COUNT; i++)
  {
    freq[i] = (1. / capture_time) * double(freq[i]);

    uint8_t source = channels[i].trig_source;
    uint32_t trig_index = triggered_index[source];
    trig_index -= trig_index % 4;

    triggered_index[i] = trig_index + i;

    // if (i == CH_A)
    // {
    //   // float T_ = float(float(T[i]) / float(T_tmp_count[i]));
    //   float T_ = float(T[i]);

    //   Serial.printf("T %d\n", (int)T[i]);
    //   Serial.printf("sr %d\n", (int)sr);
    //   Serial.printf("freq %d\n", (int)(1.f / (1.f / float(sr) * T_)));
    // }
  }
};

void start_screen()
{
  spr.fillRect(0, 0, TFT_WIDTH_WR, TFT_HEIGHT_WR, BACK_COLOR);
  spr.setTextSize(2);

  char fn[] = "EchoSystem";
  uint16_t tw = tft.textWidth(fn) * 2;

  spr.setCursor((TFT_WIDTH_WR / 2) - (tw / 2), (TFT_HEIGHT_WR / 2) - 20);

  spr.print(fn);
  spr.setTextSize(1);

  char sn[] = "Company";
  uint16_t sw = spr.textWidth(sn);

  spr.setCursor((TFT_WIDTH_WR / 2) - (sw / 2), (TFT_HEIGHT_WR / 2) + 25);

  spr.print(sn);

  tft_write();

  delay(1000);
}

void core1_entry()
{
  init_tft();
  start_screen();

  FPS fps;

  uint32_t last_update_menu_time = 0;
  static queue_entry_t entry[CH_COUNT + 1];

  uint8_t overlay_oscil_count = 0;
  // Serial.printf("pio_claim_unused_sm %d\n", pio_claim_unused_sm(pio1, 0));

  // null_auto_calc();

  while (1)
  {
    uint32_t time = to_ms_since_boot(get_absolute_time());
    uint32_t fps_ = fps.tick(time);

    if (fps_)
    {
      Serial.printf("Fps %d\n", fps_);
    }

    if (time - last_update_menu_time > 12)
    {
      last_update_menu_time = time;
      parse_input(time);
      draw_menu();
    }

    if (!hold)
    {
      if (!queue_is_empty(&_queue))
      {
        queue_try_remove(&_queue, &entry[CH_COUNT]);
        queue_entry_t *entry__ = &entry[CH_COUNT];

        // CoreMutex m(&_mutex);
        // queue_entry_t *entry__ = (queue_entry_t *)_queue.data;

        for (size_t c = 0; c < CH_COUNT; ++c)
        {
          if (((channels[c]._sync_mode == NORM_SYNC || channels[c]._sync_mode == TRIG_SYNC) && entry__->is_triggered[c]) || channels[c]._sync_mode == AUTO_SYNC)
          {
            memcpy(&entry[c], entry__, sizeof(queue_entry_t));
          }
        }
      }
      else
      {
        Serial.println("empty");
      }
    }
    else
    {
      // queue_try_remove(&_queue, NULL);
    }

    for (size_t ch = 0; ch < CH_COUNT; ch++)
    {
      channels[ch].is_triggered = entry[ch].is_triggered[ch];

      if (!channels[ch].visible)
        continue;

      uint32_t triggered_index[CH_COUNT];
      uint32_t freq[CH_COUNT];

      calc_trig_index((uint8_t *)entry[ch].buffer, BUF_SIZE_WORDS * 4, triggered_index, freq);

      if (signal_metric[ch].freq_count > 8)
      {
        signal_metric[ch].freq = signal_metric[ch].freq_tmp / signal_metric[ch].freq_count;
        signal_metric[ch].freq_tmp = freq[ch];
        signal_metric[ch].freq_count = 1;
      }
      else
      {
        signal_metric[ch].freq_tmp += freq[ch];
        signal_metric[ch].freq_count++;
      }

      parse_capture_buf((uint8_t *)entry[ch].buffer, BUF_SIZE_WORDS * 4, ch, triggered_index[ch]);

      uint8_t trig__ = (channels[ch]._sync_mode == TRIG_SYNC && channels[ch].is_triggered);

      hold = hold || trig__;
    }

    tft_write();
    spr.fillRect(OSIL_ZONE_X, OSIL_ZONE_Y, OSIL_ZONE_W, OSIL_ZONE_H, BACK_COLOR);
    draw_grid();
    overlay_oscil_count = 0;
  }
}

static queue_entry_t ad9288_entry;

void read_ad9288_wait()
{
  dma_channel_wait_for_finish_blocking(ad9288_dma_chan);

  capture = 0;

  memcpy(ad9288_entry.is_triggered, (void *)is_triggered, sizeof(is_triggered));

  if (ad9288_entry.is_triggered[CH_A])
  {
    ad9288_entry.is_triggered[CH_A] -= (uint32_t)ad9288_entry.buffer;
  }
  if (ad9288_entry.is_triggered[CH_B])
  {
    ad9288_entry.is_triggered[CH_B] -= (uint32_t)ad9288_entry.buffer;
  }

  // memcpy((void *)ad9288_entry.freq, (void *)ch_freq, sizeof(ad9288_entry.freq));

  if (is_triggered[CH_A] || is_triggered[CH_B])
  {
    queue_add_blocking(&_queue, &ad9288_entry);
  }
  else
  {
    queue_try_add(&_queue, &ad9288_entry);
  }

  memset((void *)is_triggered, 0, sizeof(is_triggered));
}

void read_ad9288()
{

  logic_analyser_arm(ad9288_pio, ad9288_sm, ad9288_dma_chan, ad9288_entry.buffer, BUF_SIZE_WORDS);

  capture = 1;
  gpio_set_irq_enabled(16, GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(17, GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(16, GPIO_IRQ_EDGE_RISE, true);
  gpio_set_irq_enabled(17, GPIO_IRQ_EDGE_RISE, true);
  // memset((void *)is_triggered, 0, sizeof(is_triggered));
}

// bool __not_in_flash_func(freq_cb)(__unused struct repeating_timer *t)
// {
//   memcpy((void *)ch_freq, (void *)trigged_count, sizeof(ch_freq));

//   for (size_t i = 0; i < CH_COUNT; ++i)
//   {
//     ch_freq[i] *= 10;
//   }

//   memset((void *)trigged_count, 0, sizeof(trigged_count));

//   return true;
// }

void core_entry()
{
  bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

  ad9288_sm = pio_claim_unused_sm(ad9288_pio, 1);
  ad9288_dma_chan = dma_claim_unused_channel(1);
  mdma_hw = dma_channel_hw_addr(ad9288_dma_chan);

  uint16_t capture_prog_instr = pio_encode_in(pio_pins, CAPTURE_PIN_COUNT);
  struct pio_program capture_prog = {
      .instructions = &capture_prog_instr,
      .length = 1,
      .origin = -1};
  ad9288_offset = pio_add_program(ad9288_pio, &capture_prog);

  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_in_pins(&c, CAPTURE_PIN_BASE);
  sm_config_set_wrap(&c, ad9288_offset, ad9288_offset);

  sm_config_set_clkdiv(&c, 1.f);

  sm_config_set_in_shift(&c, 0, 1, bits_packed_per_word(CAPTURE_PIN_COUNT));
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
  pio_sm_init(ad9288_pio, ad9288_sm, ad9288_offset, &c);

  init_ad9288_clk_ch(AD9288_A_CLK);
  init_ad9288_clk_ch(AD9288_B_CLK);

  set_time_for_grid(tdiv);

  // static struct repeating_timer timer;
  // add_repeating_timer_ms(100, freq_cb, NULL, &timer);

  FPS fps;

  // static uint32_t buf__[64]{0};

  // uint dma_nc = dma_claim_unused_channel(1);
  // dma_channel_config channel_config = dma_channel_get_default_config(dma_nc);

  // Serial.printf("Dma done..\n");
  // delay(100);

  // trig_sm = pio_claim_unused_sm(trig_pio, 1);
  // Serial.printf("SM done..\n");
  // delay(100);

  // uint16_t trig_capture_prog_instr[] = {pio_encode_in(pio_pins, 1)};

  // Serial.printf("ADD prog done 1..\n");
  // delay(100);

  // struct pio_program trig_capture_prog = {
  //     .instructions = trig_capture_prog_instr,
  //     .length = 1,
  //     .origin = -1};

  // Serial.printf("ADD prog done 2..\n");
  // delay(100);

  // uint trig_offset = pio_add_program(trig_pio, &trig_capture_prog);

  // Serial.printf("ADD prog done..\n");
  // delay(100);

  // pio_sm_config trig_c = pio_get_default_sm_config();
  // sm_config_set_in_pins(&trig_c, 16);
  // sm_config_set_wrap(&trig_c, trig_offset, trig_offset);
  // sm_config_set_clkdiv(&trig_c, 1.f);

  // Serial.printf("ADD prog 1 done..\n");
  // delay(100);

  // sm_config_set_in_shift(&trig_c, 1, 1, 32);
  // sm_config_set_fifo_join(&trig_c, PIO_FIFO_JOIN_RX);
  // pio_sm_init(trig_pio, trig_sm, trig_offset, &trig_c);

  // Serial.printf("ALL done..\n");
  // delay(100);

  while (1)
  {

    // pio_sm_set_enabled(trig_pio, trig_sm, 0);
    // pio_sm_clear_fifos(trig_pio, trig_sm);
    // pio_sm_restart(trig_pio, trig_sm);

    // dma_channel_config c = dma_channel_get_default_config(dma_nc);
    // channel_config_set_read_increment(&c, 0);
    // channel_config_set_write_increment(&c, 1);
    // channel_config_set_dreq(&c, pio_get_dreq(trig_pio, trig_sm, 0));
    // // channel_config_set_transfer_data_size(&c, DMA_SIZE_8);

    // dma_channel_configure(dma_nc, &c,
    //                       buf__,                   // Destination pointer
    //                       &trig_pio->rxf[trig_sm], // Source pointer
    //                       64,                      // Number of transfers
    //                       1                        // Start immediately
    // );

    read_ad9288();

    // pio_sm_set_enabled(trig_pio, trig_sm, 1);

    // dma_channel_wait_for_finish_blocking(dma_nc);

    // // is_triggered[CH_A] = buf__[0];

    // for (size_t i = 0; i < 64; i++)
    // {
    //   for (size_t b = 1; b < 32; b++)
    //   {
    //     uint8_t pb = bitRead(buf__[i], b - 1);
    //     uint8_t nb = bitRead(buf__[i], b);

    //     if (pb == 0 && nb == 1)
    //     {
    //       is_triggered[CH_A] = (uint32_t)&ad9288_entry.buffer[(i * 16) + b];
    //       break;
    //     }
    //   }
    // }

    // // Serial.prit

    // // if (buf__[0])
    // if (is_triggered[CH_A])
    //   Serial.printf("Trig %d\n", int(is_triggered[CH_A] - (uint32_t)ad9288_entry.buffer));
    // // sleep_ms(100);

    read_ad9288_wait();
  }
}

void load_configs()
{
  EEPROM.get(0, null_offset);
}

void __not_in_flash_func(setup)()
{
  // stdio_init_all();
  // xosc_init();

  // vreg_set_voltage(VREG_VOLTAGE_1_30);
  // sleep_ms(1000);
  set_sys_clock_khz(cpu_freq_khz, 1);
  vreg_set_voltage(VREG_VOLTAGE_1_15);

  Serial.begin(115200);
  EEPROM.begin(256);

  // attachInterrupt(digitalPinToInterrupt(16), trig_a_intr, RISING);
  // attachInterrupt(digitalPinToInterrupt(17), , RISING);

  digital_write(AD9288_S2, 0);

  digital_write(40, 1);
  digital_write(45, 1);

  // digital_write(42, 1);

  // while (1)
  // {
  //   Serial.printf("Button "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(hc165.get_byte()));
  //   Serial.println();
  //   delay(100);
  // }

  // static struct repeating_timer timer;
  // add_repeating_timer_us((1.f / 2700.f) * 1'000'000, buzzer_cb, NULL, &timer);

  // while (1)
  // {
  //   delay(100);
  // }

  for (size_t ch = 0; ch < CH_COUNT; ++ch)
  {
    ch_set_ac_dc(ch, channels[ch].is_ac);
    ch_set_fdiv(ch, channels[ch].is_10x);
    ch_set_trig_front(ch, channels[ch].is_trig_fall);
    ch_set_sdiv(ch, channels[ch].sdiv);
  }

  pwm_trig_a.init(255, 130);
  pwm_trig_b.init(255, 130);

  queue_init(&_queue, sizeof(queue_entry_t), 1);

  load_configs();

  multicore_launch_core1(core1_entry);

  core_entry();
}

void __not_in_flash_func(loop)() {}
