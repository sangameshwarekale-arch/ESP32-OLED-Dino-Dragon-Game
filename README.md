#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"

#define I2C_PORT I2C_NUM_0
#define OLED_ADDR 0x3C
#define SDA_PIN 21
#define SCL_PIN 22

#define BUTTON_PIN 16
#define BUZZER_PIN 15
#define LED_PIN 18

#define SCREEN_W 128
#define SCREEN_H 64

#define GROUND_Y 54
#define DINO_X 18
#define DINO_W 16
#define DINO_H 16

#define MAX_OBS 3

static uint8_t fb[SCREEN_W * SCREEN_H / 8];

static int dino_y;
static int dino_y_q8;
static int velocity_q8;
static bool jumping;

static int score;
static int32_t high_score;
static int ground_scroll;
static bool game_over;
static bool inverted_theme;
static int64_t led_off_time;
static bool button_last;

typedef enum {
    OBS_SMALL_CACTUS,
    OBS_TALL_CACTUS,
    OBS_FLYING
} obstacle_type_t;

typedef struct {
    int x;
    int y;
    obstacle_type_t type;
    bool active;
} obstacle_t;

static obstacle_t obstacles[MAX_OBS];

static const uint16_t dino_sprite[16] = {
    0b0000011111000000,
    0b0000111111100000,
    0b0000110111110000,
    0b0000111111110000,
    0b0000111110000000,
    0b0001111111000000,
    0b1001111110000000,
    0b1111111110000000,
    0b0111111100000000,
    0b0011111000000000,
    0b0011111000000000,
    0b0010110100000000,
    0b0010010100000000,
    0b0110010110000000,
    0b0100000010000000,
    0b1100000011000000
};

static const uint16_t cactus_tall[16] = {
    0b0000011000000000,
    0b0000011000000000,
    0b0010011001000000,
    0b0010011001000000,
    0b0011011011000000,
    0b0001111110000000,
    0b0000011000000000,
    0b0000011000000000,
    0b0100011000000000,
    0b0100011000100000,
    0b0110011001100000,
    0b0011111111000000,
    0b0000011000000000,
    0b0000011000000000,
    0b0000111100000000,
    0b0000111100000000
};

static const uint16_t cactus_small[16] = {
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000011000000000,
    0b0000011000000000,
    0b0010011001000000,
    0b0011011011000000,
    0b0001111110000000,
    0b0000011000000000,
    0b0000011000000000,
    0b0000011000000000,
    0b0000011000000000,
    0b0000011000000000,
    0b0000011000000000,
    0b0000111100000000,
    0b0000111100000000
};

static const uint16_t bird_sprite[16] = {
    0b0000000000000000,
    0b0000000000000000,
    0b0000100000010000,
    0b0001110000111000,
    0b0011111001111100,
    0b0111111111111110,
    0b1111111111111111,
    0b0011111111111100,
    0b0001111111111000,
    0b0000111111110000,
    0b0000011111100000,
    0b0000001111000000,
    0b0000000110000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000
};

static const uint8_t font5x7[][5] = {
    {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36},
    {0x3E,0x41,0x41,0x41,0x22}, {0x7F,0x41,0x41,0x22,0x1C},
    {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F},
    {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01},
    {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F},
    {0x3E,0x41,0x41,0x41,0x3E}, {0x7F,0x09,0x09,0x09,0x06},
    {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01},
    {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F},
    {0x7F,0x20,0x18,0x20,0x7F}, {0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}
};

static uint32_t rand32(void)
{
    uint32_t t = (uint32_t)esp_timer_get_time();
    t ^= t << 13;
    t ^= t >> 17;
    t ^= t << 5;
    return t;
}

static void i2c_write(uint8_t control, const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, OLED_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, control, true);
    i2c_master_write(cmd, (uint8_t *)data, len, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
}

static void oled_cmd(uint8_t cmd)
{
    i2c_write(0x00, &cmd, 1);
}

static void oled_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };

    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);

    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t cmds[] = {
        0xAE, 0x20, 0x00, 0xB0, 0xC8, 0x00, 0x10, 0x40,
        0x81, 0x7F, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
        0x00, 0xD5, 0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB,
        0x40, 0x8D, 0x14, 0xAF
    };

    for (int i = 0; i < sizeof(cmds); i++) {
        oled_cmd(cmds[i]);
    }
}

static void oled_clear(void)
{
    memset(fb, inverted_theme ? 0xFF : 0x00, sizeof(fb));
}

static void pixel(int x, int y, bool on)
{
    if (x < 0 || x >= SCREEN_W || y < 0 || y >= SCREEN_H) return;

    bool draw_on = inverted_theme ? !on : on;

    if (draw_on) {
        fb[x + (y / 8) * SCREEN_W] |= 1 << (y & 7);
    } else {
        fb[x + (y / 8) * SCREEN_W] &= ~(1 << (y & 7));
    }
}

static void line_h(int x1, int x2, int y)
{
    for (int x = x1; x <= x2; x++) {
        pixel(x, y, true);
    }
}

static void rect(int x, int y, int w, int h, bool fill)
{
    for (int yy = y; yy < y + h; yy++) {
        for (int xx = x; xx < x + w; xx++) {
            if (fill || yy == y || yy == y + h - 1 || xx == x || xx == x + w - 1) {
                pixel(xx, yy, true);
            }
        }
    }
}

static void sprite16(int x, int y, const uint16_t *sprite)
{
    for (int row = 0; row < 16; row++) {
        for (int col = 0; col < 16; col++) {
            if (sprite[row] & (1 << (15 - col))) {
                pixel(x + col, y + row, true);
            }
        }
    }
}

static void oled_update(void)
{
    for (int page = 0; page < 8; page++) {
        oled_cmd(0xB0 + page);
        oled_cmd(0x00);
        oled_cmd(0x10);
        i2c_write(0x40, &fb[SCREEN_W * page], SCREEN_W);
    }
}

static void buzzer_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ledc_channel_config(&channel);
}

static void beep(int freq, int ms)
{
    ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, freq);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(pdMS_TO_TICKS(ms));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static bool button_down(void)
{
    return gpio_get_level(BUTTON_PIN) == 0;
}

static void button_edge_reset(void)
{
    button_last = button_down();
}

static void wait_button_release(void)
{
    while (button_down()) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    button_edge_reset();
}

static bool button_pressed_edge(void)
{
    bool now = button_down();
    bool pressed = now && !button_last;
    button_last = now;
    return pressed;
}

static void led_pulse_100ms(void)
{
    gpio_set_level(LED_PIN, 1);
    led_off_time = esp_timer_get_time() + 100000;
}

static void update_led_pulse(void)
{
    if (led_off_time != 0 && esp_timer_get_time() > led_off_time) {
        gpio_set_level(LED_PIN, 0);
        led_off_time = 0;
    }
}

static void gpio_init_all(void)
{
    gpio_config_t btn = {
        .pin_bit_mask = 1ULL << BUTTON_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&btn);

    gpio_config_t led = {
        .pin_bit_mask = 1ULL << LED_PIN,
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&led);
}

static void high_score_load(void)
{
    nvs_handle_t handle;

    if (nvs_open("dino_game", NVS_READONLY, &handle) == ESP_OK) {
        nvs_get_i32(handle, "high", &high_score);
        nvs_close(handle);
    }
}

static void high_score_save(void)
{
    nvs_handle_t handle;

    if (nvs_open("dino_game", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_i32(handle, "high", high_score);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

static void high_score_update(void)
{
    if (score > high_score) {
        high_score = score;
        high_score_save();
    }
}

static int random_obstacle_gap(void)
{
    return 35 + (rand32() % 85);
}

static void make_obstacles(void)
{
    for (int i = 0; i < MAX_OBS; i++) {
        obstacles[i].active = false;
    }

    uint32_t r = rand32();
    int count = 1;

    if ((r % 10) >= 7) count = 2;
    if ((r % 20) == 0) count = 3;

    bool flying = ((r % 5) == 0);

    for (int i = 0; i < count; i++) {
        obstacles[i].active = true;
        obstacles[i].x = SCREEN_W + random_obstacle_gap() + i * (18 + (rand32() % 10));

        if (flying && i == 0) {
            obstacles[i].type = OBS_FLYING;
            obstacles[i].y = 18;
        } else if ((rand32() % 2) == 0) {
            obstacles[i].type = OBS_SMALL_CACTUS;
            obstacles[i].y = GROUND_Y - 16;
        } else {
            obstacles[i].type = OBS_TALL_CACTUS;
            obstacles[i].y = GROUND_Y - 16;
        }
    }
}

static void draw_digit(int x, int y, int d)
{
    const uint8_t font[10][5] = {
        {0x3E,0x51,0x49,0x45,0x3E},
        {0x00,0x42,0x7F,0x40,0x00},
        {0x42,0x61,0x51,0x49,0x46},
        {0x21,0x41,0x45,0x4B,0x31},
        {0x18,0x14,0x12,0x7F,0x10},
        {0x27,0x45,0x45,0x45,0x39},
        {0x3C,0x4A,0x49,0x49,0x30},
        {0x01,0x71,0x09,0x05,0x03},
        {0x36,0x49,0x49,0x49,0x36},
        {0x06,0x49,0x49,0x29,0x1E}
    };

    if (d < 0 || d > 9) return;

    for (int col = 0; col < 5; col++) {
        for (int row = 0; row < 7; row++) {
            if (font[d][col] & (1 << row)) {
                pixel(x + col, y + row, true);
            }
        }
    }
}

static void draw_number(int x, int y, int n)
{
    char buf[12];
    snprintf(buf, sizeof(buf), "%d", n);

    for (int i = 0; buf[i]; i++) {
        draw_digit(x + i * 6, y, buf[i] - '0');
    }
}

static void draw_char_scaled(int x, int y, char c, int scale)
{
    if (c >= '0' && c <= '9') {
        const uint8_t digit_font[10][5] = {
            {0x3E,0x51,0x49,0x45,0x3E},
            {0x00,0x42,0x7F,0x40,0x00},
            {0x42,0x61,0x51,0x49,0x46},
            {0x21,0x41,0x45,0x4B,0x31},
            {0x18,0x14,0x12,0x7F,0x10},
            {0x27,0x45,0x45,0x45,0x39},
            {0x3C,0x4A,0x49,0x49,0x30},
            {0x01,0x71,0x09,0x05,0x03},
            {0x36,0x49,0x49,0x49,0x36},
            {0x06,0x49,0x49,0x29,0x1E}
        };

        int d = c - '0';
        for (int col = 0; col < 5; col++) {
            for (int row = 0; row < 7; row++) {
                if (digit_font[d][col] & (1 << row)) {
                    rect(x + col * scale, y + row * scale, scale, scale, true);
                }
            }
        }
        return;
    }

    if (c < 'A' || c > 'Z') return;

    const uint8_t *glyph = font5x7[c - 'A'];

    for (int col = 0; col < 5; col++) {
        for (int row = 0; row < 7; row++) {
            if (glyph[col] & (1 << row)) {
                rect(x + col * scale, y + row * scale, scale, scale, true);
            }
        }
    }
}

static int text_width_scaled(const char *text, int scale)
{
    int width = 0;

    for (int i = 0; text[i]; i++) {
        width += (text[i] == ' ') ? 4 * scale : 6 * scale;
    }

    return width;
}

static void draw_text_center_scaled(const char *text, int center_x, int y, int scale)
{
    int x = center_x - text_width_scaled(text, scale) / 2;

    for (int i = 0; text[i]; i++) {
        if (text[i] != ' ') {
            draw_char_scaled(x, y, text[i], scale);
        }

        x += (text[i] == ' ') ? 4 * scale : 6 * scale;
    }
}

static void animate_word_zoom(const char *text)
{
    for (int frame = 0; frame < 24; frame++) {
        oled_clear();

        int y;
        int scale;

        if (frame < 12) {
            y = -10 + frame * 3;
            scale = (frame < 6) ? 1 : 2;
        } else {
            y = 26 + (frame - 12) * 3;
            scale = (frame < 18) ? 2 : 1;
        }

        draw_text_center_scaled(text, SCREEN_W / 2, y, scale);
        oled_update();

        vTaskDelay(pdMS_TO_TICKS(35));
    }
}

static void start_animation(void)
{
    inverted_theme = false;

    beep(900, 60);
    animate_word_zoom("READY");

    beep(1200, 50);
    animate_word_zoom("3");

    beep(1000, 60);
    animate_word_zoom("2");

    beep(1200, 60);
    animate_word_zoom("1");

    beep(1800, 90);
    animate_word_zoom("GO");
}

static void reset_game(void)
{
    dino_y = GROUND_Y - DINO_H;
    dino_y_q8 = dino_y * 256;
    velocity_q8 = 0;
    jumping = false;
    score = 0;
    ground_scroll = 0;
    game_over = false;
    inverted_theme = false;
    led_off_time = 0;
    gpio_set_level(LED_PIN, 0);
    make_obstacles();
    button_edge_reset();
}

static bool box_hit(int ax, int ay, int aw, int ah, int bx, int by, int bw, int bh)
{
    return ax + aw > bx && ax < bx + bw && ay + ah > by && ay < by + bh;
}

static bool collision(void)
{
    int dino_left = DINO_X + 2;
    int dino_top = dino_y + 1;
    int dino_w = 12;
    int dino_h = 14;

    for (int i = 0; i < MAX_OBS; i++) {
        if (!obstacles[i].active) continue;

        int ox;
        int oy;
        int ow;
        int oh;

        if (obstacles[i].type == OBS_FLYING) {
            ox = obstacles[i].x + 1;
            oy = obstacles[i].y + 4;
            ow = 14;
            oh = 7;
        } else if (obstacles[i].type == OBS_SMALL_CACTUS) {
            ox = obstacles[i].x + 4;
            oy = obstacles[i].y + 5;
            ow = 8;
            oh = 11;
        } else {
            ox = obstacles[i].x + 4;
            oy = obstacles[i].y;
            ow = 8;
            oh = 16;
        }

        if (box_hit(dino_left, dino_top, dino_w, dino_h, ox, oy, ow, oh)) {
            return true;
        }
    }

    return false;
}

static void draw_obstacles(void)
{
    for (int i = 0; i < MAX_OBS; i++) {
        if (!obstacles[i].active) continue;

        if (obstacles[i].type == OBS_FLYING) {
            sprite16(obstacles[i].x, obstacles[i].y, bird_sprite);
        } else if (obstacles[i].type == OBS_SMALL_CACTUS) {
            sprite16(obstacles[i].x, obstacles[i].y, cactus_small);
        } else {
            sprite16(obstacles[i].x, obstacles[i].y, cactus_tall);
        }
    }
}

static void draw_game(void)
{
    oled_clear();

    line_h(0, SCREEN_W - 1, GROUND_Y);

    for (int x = -ground_scroll; x < SCREEN_W; x += 14) {
        pixel(x, GROUND_Y + 3, true);
        pixel(x + 4, GROUND_Y + 5, true);
    }

    sprite16(DINO_X, dino_y, dino_sprite);
    draw_obstacles();

    int shown_high = score > high_score ? score : high_score;
    draw_number(58, 2, shown_high);

    for (int y = 1; y < 10; y++) {
        pixel(88, y, true);
    }

    draw_number(96, 2, score);

    if (game_over) {
        rect(22, 22, 84, 22, false);
        draw_number(56, 30, score);
    }

    oled_update();
}

void app_main(void)
{
    gpio_init_all();
    buzzer_init();
    oled_init();

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    high_score_load();

    wait_button_release();
    start_animation();
    reset_game();

    while (1) {
        if (!game_over) {
            if (button_pressed_edge() && !jumping) {
                led_pulse_100ms();
                velocity_q8 = -1800;
                jumping = true;
                beep(1600, 25);
            }

            if (jumping) {
                dino_y_q8 += velocity_q8;
                velocity_q8 += 180;
                dino_y = dino_y_q8 / 256;

                if (dino_y >= GROUND_Y - DINO_H) {
                    dino_y = GROUND_Y - DINO_H;
                    dino_y_q8 = dino_y * 256;
                    velocity_q8 = 0;
                    jumping = false;
                }
            }

            bool all_passed = true;

            for (int i = 0; i < MAX_OBS; i++) {
                if (!obstacles[i].active) continue;

                obstacles[i].x -= 4;

                if (obstacles[i].x > -18) {
                    all_passed = false;
                }
            }

            if (all_passed) {
                score++;

                if (score % 10 == 0) {
                    inverted_theme = !inverted_theme;
                    beep(1900, 60);
                }

                make_obstacles();
            }

            if (collision()) {
                high_score_update();
                game_over = true;
                gpio_set_level(LED_PIN, 1);
                beep(350, 180);
                wait_button_release();
            }
        } else {
            if (button_pressed_edge()) {
                led_pulse_100ms();
                beep(1200, 50);
                wait_button_release();
                start_animation();
                reset_game();
            }
        }

        ground_scroll += 2;
        if (ground_scroll >= 14) {
            ground_scroll = 0;
        }

        draw_game();
        update_led_pulse();
        vTaskDelay(pdMS_TO_TICKS(35));
    }
}
