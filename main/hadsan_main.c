/*
 * main_clean.c
 * Single-file clean implementation for: Keypad (4x4) -> PIN -> I2C LCD (PCF8574) + MPU6050 + LEDs
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#define ROWS 4
#define COLS 4
static int scanRowPins[ROWS] = {37, 36, 35, 0};
static int colPins[COLS] = {45, 48, 47, 21};

#define LED_GREEN_GPIO 38
#define LED_RED_GPIO   39

#define LCD_I2C_PORT I2C_NUM_0
#define LCD_I2C_SDA  12
#define LCD_I2C_SCL  11
#define PCF8574_ADDR 0x27

#define MPU_I2C_NUM  I2C_NUM_1
#define MPU_I2C_SDA  8
#define MPU_I2C_SCL  18
#define MPU6050_ADDR 0x68

static const char *TAG = "main_clean";

static char keys[ROWS][COLS] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

// PCF8574
static uint8_t g_pcfaddr = PCF8574_ADDR;
static bool g_pcf_ok = false;

static esp_err_t pcf8574_write(uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (g_pcfaddr<<1)|I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(LCD_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    g_pcf_ok = (err == ESP_OK);
    return err;
}

// PCF to LCD mapping (typical)
#define P_RS (1<<7)
#define P_RW (1<<6)
#define P_EN (1<<5)
#define P_BL (1<<4)
#define P_D4 (1<<0)
#define P_D5 (1<<1)
#define P_D6 (1<<2)
#define P_D7 (1<<3)

static void lcd_write_nibble(uint8_t nibble, uint8_t ctrl) {
    uint8_t out = 0;
    if (nibble & 0x01) out |= P_D4;
    if (nibble & 0x02) out |= P_D5;
    if (nibble & 0x04) out |= P_D6;
    if (nibble & 0x08) out |= P_D7;
    out |= P_BL;
    out |= (ctrl & (P_RS|P_RW));
    pcf8574_write(out | P_EN);
    vTaskDelay(pdMS_TO_TICKS(1));
    pcf8574_write(out & ~P_EN);
    vTaskDelay(pdMS_TO_TICKS(1));
}

static void lcd_send(uint8_t b, uint8_t rs) {
    lcd_write_nibble((b>>4)&0x0F, rs?P_RS:0);
    lcd_write_nibble(b&0x0F, rs?P_RS:0);
}

void lcd_send_cmd(uint8_t cmd) { lcd_send(cmd, 0); }
void lcd_send_char(uint8_t c) { lcd_send(c, P_RS); }

void lcd_set_cursor(uint8_t col, uint8_t row) { uint8_t addr = row?0x40:0x00; lcd_send_cmd(0x80 | (addr + col)); }
void lcd_clear(void) { lcd_send_cmd(0x01); vTaskDelay(pdMS_TO_TICKS(2)); }

void lcd_init(void) {
    if (!g_pcf_ok) { ESP_LOGW(TAG, "PCF not ready, skipping LCD init"); return; }
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_write_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(2));
    lcd_write_nibble(0x02, 0);
    lcd_send_cmd(0x28); lcd_send_cmd(0x0C); lcd_send_cmd(0x01); vTaskDelay(pdMS_TO_TICKS(2)); lcd_send_cmd(0x06);
}

void lcd_print_lines(const char *l1, const char *l2) {
    if (!g_pcf_ok) { printf("LCD fallback L1:%s L2:%s\n", l1?l1:"", l2?l2:""); return; }
    lcd_clear(); lcd_set_cursor(0,0);
    int i=0; const char *s=l1?l1:""; while (*s && i<16) { lcd_send_char((uint8_t)*s); ++s; ++i; }
    lcd_set_cursor(0,1); i=0; s=l2?l2:""; while (*s && i<16) { lcd_send_char((uint8_t)*s); ++s; ++i; }
}

void lcd_print(const char *txt) { lcd_print_lines(txt, ""); }

// Backlight control and wake helper
void lcd_backlight_on(void) {
    if (!g_pcf_ok) { ESP_LOGW(TAG, "PCF not ready; cannot turn backlight on"); return; }
    pcf8574_write(P_BL);
}
void lcd_backlight_off(void) {
    if (!g_pcf_ok) { ESP_LOGW(TAG, "PCF not ready; cannot turn backlight off"); return; }
    pcf8574_write(0x00);
}

// Ensure LCD is awake and backlight is on, pulse EN once for safety
void pcf8574_wake(void) {
    if (!g_pcf_ok) {
        // Try to write backlight bit and see if it succeeds
        pcf8574_write(P_BL);
        if (!g_pcf_ok) { ESP_LOGW(TAG, "PCF write failure, cannot wake LCD"); return; }
    }
    // Turn backlight on
    pcf8574_write(P_BL);
    // Pulse EN to ensure the expander-controlled lines are applied
    pcf8574_write(P_BL | P_EN);
    vTaskDelay(pdMS_TO_TICKS(5));
    pcf8574_write(P_BL);
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGI(TAG, "PCF8574 wake/backlight: ON");
}

// I2C init helper
void i2c_master_init_port(i2c_port_t port, gpio_num_t sda, gpio_num_t scl) {
    i2c_config_t conf;
    memset(&conf, 0, sizeof(conf));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda; conf.scl_io_num = scl;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE; conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(port, &conf);
    i2c_driver_install(port, conf.mode, 0, 0, 0);
}

// ---------- MPU6050
static esp_err_t i2c_write_reg(i2c_port_t port, uint8_t dev, uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev<<1)|I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return r;
}

static esp_err_t i2c_read_block(i2c_port_t port, uint8_t dev, uint8_t reg, uint8_t *buf, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev<<1)|I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev<<1)|I2C_MASTER_READ, true);
    if (len>1) i2c_master_read(cmd, buf, len-1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, buf+len-1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return r;
}

void mpu6050_init(void) { i2c_write_reg(MPU_I2C_NUM, MPU6050_ADDR, 0x6B, 0x00); }
int mpu6050_read_all(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *t) {
    uint8_t b[14]; if (i2c_read_block(MPU_I2C_NUM, MPU6050_ADDR, 0x3B, b, 14) != ESP_OK) return -1;
    int16_t axr=((int16_t)b[0]<<8)|b[1]; int16_t ayr=((int16_t)b[2]<<8)|b[3]; int16_t azr=((int16_t)b[4]<<8)|b[5];
    int16_t tr=((int16_t)b[6]<<8)|b[7]; int16_t gxr=((int16_t)b[8]<<8)|b[9]; int16_t gyr=((int16_t)b[10]<<8)|b[11]; int16_t gzr=((int16_t)b[12]<<8)|b[13];
    *ax = axr/16384.0f; *ay = ayr/16384.0f; *az = azr/16384.0f; *gx = gxr/131.0f; *gy = gyr/131.0f; *gz = gzr/131.0f; *t = tr/340.0f + 36.53f; return 0;
}

// ---------- Keypad
char readKeyPadWithTimeout(int timeoutMs) {
    TickType_t tstart = xTaskGetTickCount();
    while ((xTaskGetTickCount() - tstart) < pdMS_TO_TICKS(timeoutMs)) {
        for (int r=0;r<ROWS;++r) {
            for (int j=0;j<ROWS;++j) gpio_set_level(scanRowPins[j], (j==r)?0:1);
            vTaskDelay(pdMS_TO_TICKS(8));
            for (int c=0;c<COLS;++c) {
                if (gpio_get_level(colPins[c]) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(40));
                    for (int j=0;j<ROWS;++j) gpio_set_level(scanRowPins[j], (j==r)?0:1);
                    vTaskDelay(pdMS_TO_TICKS(5));
                    if (gpio_get_level(colPins[c]) == 0) {
                        while (gpio_get_level(colPins[c]) == 0) vTaskDelay(pdMS_TO_TICKS(20));
                        return keys[r][c];
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return '\0';
}
char readKeyPadBlock(void) { return readKeyPadWithTimeout(0x7fffffff); }

// ---------- LEDs
void led_set_green(int on) { gpio_set_level(LED_GREEN_GPIO, on?1:0); }
void led_set_red(int on) { gpio_set_level(LED_RED_GPIO, on?1:0); }

// ---------- Setup
void setup_gpio(void) {
    for (int i=0;i<ROWS;++i) { gpio_set_direction(scanRowPins[i], GPIO_MODE_OUTPUT); gpio_set_level(scanRowPins[i], 1); }
    for (int i=0;i<COLS;++i) { gpio_set_direction(colPins[i], GPIO_MODE_INPUT); gpio_set_pull_mode(colPins[i], GPIO_PULLUP_ONLY); }
    gpio_set_direction(LED_GREEN_GPIO, GPIO_MODE_OUTPUT); gpio_set_level(LED_GREEN_GPIO, 0);
    gpio_set_direction(LED_RED_GPIO, GPIO_MODE_OUTPUT); gpio_set_level(LED_RED_GPIO, 0);
    i2c_master_init_port(LCD_I2C_PORT, LCD_I2C_SDA, LCD_I2C_SCL);
    i2c_master_init_port(MPU_I2C_NUM, MPU_I2C_SDA, MPU_I2C_SCL);
    // detect pcf and try to wake the LCD by turning on backlight
    pcf8574_write(P_BL);
    if (g_pcf_ok) { pcf8574_wake(); lcd_init(); }
    mpu6050_init();
}

// Display sensor data once on LCD
void show_sensor_data_once(void) {
    float ax,ay,az,gx,gy,gz,t;
    if (mpu6050_read_all(&ax,&ay,&az,&gx,&gy,&gz,&t) != 0) { lcd_print("MPU error"); vTaskDelay(pdMS_TO_TICKS(1500)); return; }
    char l1[17], l2[17];
    snprintf(l1, sizeof(l1), "A X:%.2f Y:%.2f", ax, ay);
    snprintf(l2, sizeof(l2), "Z:%.2f T:%.1fC", az, t);
    lcd_print_lines(l1,l2); vTaskDelay(pdMS_TO_TICKS(2500));
    snprintf(l1, sizeof(l1), "G X:%.2f Y:%.2f", gx, gy);
    snprintf(l2, sizeof(l2), "Z:%.2f", gz);
    lcd_print_lines(l1,l2); vTaskDelay(pdMS_TO_TICKS(2500));
}

void app_main(void) {
    setup_gpio();
    const char *prompt = "Enter Key Code:";
    const char *correctpin = "1234";
    char pinbuf[8]; int pinlen=0;
    while (1) {
        lcd_print(prompt);
        pinlen = 0; pinbuf[0]=0; bool submitted=false;
        while (!submitted) {
            char k = readKeyPadWithTimeout(5000);
            if (k == '\0') { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
            if (k >= '0' && k <= '9') { if (pinlen<7) { pinbuf[pinlen++]=k; pinbuf[pinlen]=0; char mask[17]; memset(mask,'*',pinlen); mask[pinlen]=0; lcd_print(mask); } }
            else if (k == '#') { if (pinlen>0) submitted=true; }
            else if (k == '*') { if (pinlen>0) { pinbuf[--pinlen]=0; char mask[17]; memset(mask,'*',pinlen); mask[pinlen]=0; lcd_print(mask); } }
        }
        if (pinlen == (int)strlen(correctpin) && strncmp(pinbuf, correctpin, pinlen) == 0) {
            lcd_print("Access Granted!"); led_set_green(1); vTaskDelay(pdMS_TO_TICKS(3000)); led_set_green(0);
            for (int i=0;i<3;i++) show_sensor_data_once();
        } else {
            lcd_print("Access Denied"); led_set_red(1); vTaskDelay(pdMS_TO_TICKS(2000)); led_set_red(0);
        }
    }
}
