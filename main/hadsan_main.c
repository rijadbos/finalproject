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

/* ============================================================
 * hadsan_main.c
 *
 * Simple demo application for ESP32 (ESP-IDF) that:
 * - Reads a 4x4 matrix keypad
 * - Displays prompts and sensor data on an I2C LCD (PCF8574-backed)
 * - Reads accelerometer/gyro/temp from an MPU6050 sensor over I2C
 * - Controls two status LEDs (GREEN/RED)
 *
 * This file contains configuration constants, init routines for GPIO
 * and I2C, helper functions for the keypad/LCD/MPU, and a simple
 * state-machine handling unlocking and displaying sensor data.
 *
 * Note: This is a demonstration implementation; adjust pin mappings
 * and timing for your particular hardware.
 *
 * ============================================================ */

/* ============================================================
CONFIGURATION
============================================================ */

// LEDs
// GPIO pins connected to on-board LEDs used as visual status indicators
#define GREEN_LED 38
#define RED_LED   39

// Keypad
// 4x4 keypad configuration: rows are driven as outputs; columns are inputs
#define ROWS 4
#define COLS 4
int scanRowPins[ROWS] = {37,36,35,0}; // Row output pins (scanned)
int colPins[COLS]     = {45,48,47,21}; // Column input pins (with pull-ups)
// Human-readable key mapping for the 4x4 matrix
char keys[ROWS][COLS] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

// LCD (PCF8574-backed HD44780 16x2 on I2C)
// Default I2C port and pins used for the LCD controller and PCF8574 expander
#define LCD_I2C_PORT I2C_NUM_0
#define LCD_I2C_SDA  5
#define LCD_I2C_SCL  4
#define PCF8574_ADDR 0x27
// PCF8574 bit mappings
#define PCF_RS (1<<0) // Register Select (0=command, 1=data)
#define PCF_EN (1<<2) // Enable pulse to latch data
#define PCF_BL (1<<3) // Backlight

// MPU6050 (I2C configuration and registers)
#define MPU_I2C_NUM  I2C_NUM_1
#define MPU_I2C_SDA  8
#define MPU_I2C_SCL  18
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

// Globals: whether a PCF8574-compatible I/O expander was detected
// and its address (defaults to 0x27)
static bool g_has_pcf = false;
static uint8_t g_pcf_addr = PCF8574_ADDR;

// Default code for the simple unlock demo
static const char *CORRECT_PIN = "1234";

/* ============================================================
KEYPAD FUNCTIONS
============================================================ */
// Initialize GPIOs used by the matrix keypad. Rows are outputs that will
// be driven low during scanning; columns are inputs with pull-ups and
// are read to detect key presses (active-low wiring).
void setup_gpio_keypad() {
    for (int i = 0; i < ROWS; i++) {
        gpio_set_direction(scanRowPins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(scanRowPins[i], 1); // idle high
    }
    for (int i = 0; i < COLS; i++) {
        gpio_set_direction(colPins[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(colPins[i], GPIO_PULLUP_ONLY); // button pulls to GND
    }
}

// Scan a single keypad scan cycle and return a new key character
// or '\0' if no new keypress was detected. It performs basic
// debouncing by returning the key only when it changes.
char scan_keypad_single() {
    static char last_key = '\0';
    char k = '\0';
    for (int i = 0; i < ROWS; i++) {
        // Make row 'i' active and others inactive
        uint8_t scanVal = ~(1 << i);
        for (int j = 0; j < ROWS; j++) gpio_set_level(scanRowPins[j], (scanVal >> j) & 1);
        vTaskDelay(pdMS_TO_TICKS(5)); // allow signals to settle
        for (int col = 0; col < COLS; col++) {
            // Column is active-low; 0 means pressed
            if (gpio_get_level(colPins[col]) == 0) {
                k = keys[i][col];
                break;
            }
        }
        if (k != '\0') break;
    }
    // Debounce / change-detection: only return when a different key is
    // observed than the last one we reported.
    if (k != '\0' && k != last_key) {
        last_key = k;
        return k;
    }
    if (k == '\0') last_key = '\0';
    return '\0';
}

/* ============================================================
LCD FUNCTIONS
============================================================ */
// Initialize a specified I2C master port (driver install) with provided
// SDA/SCL pins and standard 100kHz clock. Returns ESP_OK on success or
// appropriate driver error otherwise.
static esp_err_t i2c_master_init_port(i2c_port_t port, gpio_num_t sda, gpio_num_t scl) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    ESP_ERROR_CHECK(i2c_param_config(port, &conf));
    return i2c_driver_install(port, conf.mode, 0, 0, 0);
}

// Helper: write a single byte to the PCF8574 (I2C expander) on the LCD
static esp_err_t pcf_write_one(uint8_t addr, uint8_t data) {
    return i2c_master_write_to_device(LCD_I2C_PORT, addr, &data, 1, pdMS_TO_TICKS(100));
}

// Returns true if a PCF8574 device ACKs a zero-write on that address
static bool pcf_probe(uint8_t addr) { return pcf_write_one(addr, 0x00) == ESP_OK; }

// Low level: write 4 bits (nibble) to LCD via PCF8574 expander.
// nibble: lower 4 bits used; flags controls register select bit.
static void pcf_write_nibble(uint8_t nibble, uint8_t flags) {
    uint8_t out = PCF_BL; // keep backlight on
    // Map each nibble bit to corresponding PCF output (pins 4..7)
    if (nibble & 1) { out |= (1 << 4); }
    if (nibble & 2) { out |= (1 << 5); }
    if (nibble & 4) { out |= (1 << 6); }
    if (nibble & 8) { out |= (1 << 7); }
    if (flags & PCF_RS) out |= PCF_RS; // mark data vs command
    // Toggle EN to latch nibble
    pcf_write_one(g_pcf_addr, out | PCF_EN);
    vTaskDelay(pdMS_TO_TICKS(1));
    pcf_write_one(g_pcf_addr, out & ~PCF_EN);
    vTaskDelay(pdMS_TO_TICKS(1));
}

// High level LCD helpers (4-bit)... wrappers for commands and writes
static void lcd_send_byte(uint8_t b, bool rs) { pcf_write_nibble((b >> 4) & 0x0F, rs ? PCF_RS : 0); pcf_write_nibble(b & 0x0F, rs ? PCF_RS : 0); }
static void lcd_cmd(uint8_t cmd) { lcd_send_byte(cmd, false); }
static void lcd_putc(char c) { lcd_send_byte((uint8_t)c, true); }
static void lcd_clear() { lcd_cmd(0x01); vTaskDelay(pdMS_TO_TICKS(2)); }
static void lcd_set_cursor(uint8_t col, uint8_t row) { uint8_t addr = row ? 0x40 : 0x00; lcd_cmd(0x80 | (addr + col)); }
static void lcd_print_lines(const char *l1, const char *l2) {
    lcd_clear();
    lcd_set_cursor(0, 0);
    for (int i = 0; l1 && l1[i] && i < 16; i++) lcd_putc(l1[i]);
    lcd_set_cursor(0, 1);
    for (int i = 0; l2 && l2[i] && i < 16; i++) lcd_putc(l2[i]);
}
// Initialize HD44780 in 4-bit mode via the PCF8574 expander. This follows
// the typical initialization sequence recommended in the datasheet.
static void lcd_init() {
    vTaskDelay(pdMS_TO_TICKS(50));
    pcf_write_one(g_pcf_addr, 0); // clear outputs first
    vTaskDelay(pdMS_TO_TICKS(5));
    // Sequence to switch from 8-bit to 4-bit mode
    pcf_write_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(5));
    pcf_write_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(5));
    pcf_write_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(5));
    pcf_write_nibble(0x02, 0); vTaskDelay(pdMS_TO_TICKS(5));
    // Function set: 4-bit, 2-line
    lcd_cmd(0x28);
    // Display on, cursor/off, blink/off
    lcd_cmd(0x0C);
    // Clear display
    lcd_cmd(0x01); vTaskDelay(pdMS_TO_TICKS(2));
    // Entry mode set
    lcd_cmd(0x06);
}

/* ============================================================
MPU6050 FUNCTIONS
============================================================ */
// Wake the MPU6050 by clearing the sleep bit in power management register
static esp_err_t mpu_wake() {
    uint8_t cmd[2] = {MPU6050_PWR_MGMT_1, 0x00};
    return i2c_master_write_to_device(MPU_I2C_NUM, MPU6050_ADDR, cmd, 2, pdMS_TO_TICKS(200));
}
// Read accelerometer, gyro, and temperature data from MPU and convert to
// floats in meaningful units (approx). Returns ESP_OK on success.
static esp_err_t mpu_read_raw(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *temp) {
    uint8_t buf[14];
    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    esp_err_t r = i2c_master_write_read_device(MPU_I2C_NUM, MPU6050_ADDR, &reg, 1, buf, sizeof(buf), pdMS_TO_TICKS(200));
    if (r != ESP_OK) return r;
    int16_t axr = (buf[0] << 8) | buf[1];
    int16_t ayr = (buf[2] << 8) | buf[3];
    int16_t azr = (buf[4] << 8) | buf[5];
    int16_t tr  = (buf[6] << 8) | buf[7];
    int16_t gxr = (buf[8] << 8) | buf[9];
    int16_t gyr = (buf[10] << 8) | buf[11];
    int16_t gzr = (buf[12] << 8) | buf[13];
    *ax = axr / 16384.0f;
    *ay = ayr / 16384.0f;
    *az = azr / 16384.0f;
    *gx = gxr / 16384.0f;
    *gy = gyr / 16384.0f;
    *gz = gzr / 16384.0f;
    *temp = tr / 340.0f + 36.53f; // convert to degrees C
    return ESP_OK;
}

/* ============================================================
MAIN TASK
============================================================ */
// Simple system states used by the main state machine
typedef enum {STATE_POWERUP, STATE_UNLOCKED, STATE_DENIED} system_state_t;

// Application entry point
// Initializes peripherals and runs an event loop that accepts keypad
// input to validate a PIN. If PIN is correct, the device shows sensor
// readings; otherwise it denies access briefly.
void app_main() {
// Initialize LEDs
gpio_set_direction(GREEN_LED,GPIO_MODE_OUTPUT); 
gpio_set_level(GREEN_LED,0);
gpio_set_direction(RED_LED,GPIO_MODE_OUTPUT); 
gpio_set_level(RED_LED,0);

    // Initialize I2C buses used by LCD (I2C0) and MPU6050 (I2C1)
ESP_ERROR_CHECK(i2c_master_init_port(LCD_I2C_PORT,LCD_I2C_SDA,LCD_I2C_SCL));
ESP_ERROR_CHECK(i2c_master_init_port(MPU_I2C_NUM,MPU_I2C_SDA,MPU_I2C_SCL));

    // Configure keypad GPIOs
setup_gpio_keypad();

    // Probe for an I2C PCF8574 expander commonly used with 16x2 LCDs
uint8_t scan_list[]={0x27,0x3F};
for(int i=0;i<2;i++){ if(pcf_probe(scan_list[i])){ g_pcf_addr=scan_list[i]; g_has_pcf=true; break; } }
if(g_has_pcf) lcd_init();

    // Wake the MPU6050 (clear sleep bit)
mpu_wake();

    // Buffer storing up to a 4-digit PIN (NUL-terminated)
    char entered[5] = {0};
    int pos = 0;
    // Start in power-up state, showing prompt
    system_state_t state = STATE_POWERUP;

while(1){
    // POWERUP STATE
    if(state==STATE_POWERUP){
        printf("System running..\n");
        if(g_has_pcf) lcd_print_lines("Enter Key Code:","");
    }

    // Read keypad
    char k = scan_keypad_single();
    if(k!='\0' && state==STATE_POWERUP){
        if(k>='0' && k<='9' && pos<4){ entered[pos++]=k; entered[pos]=0; if(g_has_pcf) lcd_print_lines("Enter Key Code:",entered);}
        if (k == '#') { // submit
            if (strcmp(entered, CORRECT_PIN) == 0) {
                // Correct PIN: move to unlocked state and show confirmation
                state = STATE_UNLOCKED;
                if (g_has_pcf) lcd_print_lines("Access Granted!", "");
                // Flash green LED to signal success
                gpio_set_level(GREEN_LED, 1);
                vTaskDelay(pdMS_TO_TICKS(5000));
                gpio_set_level(GREEN_LED, 0);
            }else{
                state=STATE_DENIED;
                if(g_has_pcf) lcd_print_lines("Access Denied","");
                // Blink red LED for denied access
                gpio_set_level(RED_LED, 1);
                vTaskDelay(pdMS_TO_TICKS(2000));
                gpio_set_level(RED_LED, 0);
                memset(entered,0,5); pos=0; state=STATE_POWERUP;
            }
        }
        if (k == '*') { // clear entry
            pos = 0; memset(entered, 0, 5);
            if (g_has_pcf) lcd_print_lines("Enter Key Code:", "");
        }
    }

    // DISPLAY SENSOR DATA
    if(state==STATE_UNLOCKED){
        float ax,ay,az,gx,gy,gz,temp;
            // Read MPU sensor data and display these in 3 pages on LCD
            if (mpu_read_raw(&ax, &ay, &az, &gx, &gy, &gz, &temp) == ESP_OK) {
            char l1[17],l2[17];

            // Accel
            snprintf(l1,17,"AX:%0.2f AY:%0.2f",ax,ay);
            snprintf(l2,17,"AZ:%0.2f",az);
            if (g_has_pcf) lcd_print_lines(l1, l2);
            vTaskDelay(pdMS_TO_TICKS(3000));

            // Gyro (just show same for demo if no gyro, can expand later)
            snprintf(l1,17,"GX:%0.2f GY:%0.2f",gx,gy); // replace with gyro if available
            snprintf(l2,17,"GZ:%0.2f",gz);
            if (g_has_pcf) lcd_print_lines(l1, l2);
            vTaskDelay(pdMS_TO_TICKS(3000));

            // Temp
            snprintf(l1,17,"Temp:%0.1fC",temp);
            snprintf(l2,17,"Thank You!");
            if (g_has_pcf) lcd_print_lines(l1, l2);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
        memset(entered,0,5); pos=0; state=STATE_POWERUP;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}

}
