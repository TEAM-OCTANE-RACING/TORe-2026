
// ADS1115 channel 
//   AIN0 APPS S1  (normal polarity pot)
//   AIN1 APPS S2  (inverted polarity pot)
//   AIN2 BPPS     (brake pedal pot)
//
// GPIO 
//   GPIO 17 SDC closed signal  
//   GPIO 27 Starter button      
//   GPIO 22  Buzzer              
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <lgpio.h>          


long  get_time_ms(void);
float R2D_update(uint16_t adc[3], bool SDC_closed,
                 bool starter_btn_state, long now_ms);
bool  r2d_is_active(void);
bool  r2d_sound_active(void);
bool  r2d_apps_fault(void);
void  r2d_reset(void);


#define I2C_BUS          "/dev/i2c-1"
#define ADS1115_ADDR     0x48
#define ADS1115_REG_CONV 0x00
#define ADS1115_REG_CFG  0x01


#define CFG_AIN0  0xC083    // MUX=100 AIN0 
#define CFG_AIN1  0xD083    // MUX=101 AIN1 
#define CFG_AIN2  0xE083    // MUX=110 AIN2 

// Raw value the ADS1115 returns when input = exactly 3.3 V at PGA ±6.144 V:
//   raw = (3.3 / 6.144) * 32767 = 17589
#define RAW_AT_3V3  17589L


#define GPIO_CHIP        0    
#define PIN_SDC          17   // input  — SDC closed signal
#define PIN_STARTER      27   // input  — starter button
#define PIN_BUZZER       22   // output — R2D buzzer


static int i2c_fd  = -1;
static int gpio_h  = -1;     // lgpio chip handle


static int ads1115_open(void) {
    i2c_fd = open(I2C_BUS, O_RDWR);
    if (i2c_fd < 0) { perror("open /dev/i2c-1"); return -1; }
    if (ioctl(i2c_fd, I2C_SLAVE, ADS1115_ADDR) < 0) {
        perror("ioctl I2C_SLAVE"); close(i2c_fd); return -1;
    }
    return 0;
}

static void ads1115_close(void) {
    if (i2c_fd >= 0) { close(i2c_fd); i2c_fd = -1; }
}

static int ads1115_write_reg(uint8_t reg, uint16_t val) {
    uint8_t buf[3] = {reg, (val >> 8) & 0xFF, val & 0xFF};
    if (write(i2c_fd, buf, 3) != 3) { perror("I2C write"); return -1; }
    return 0;
}


static uint16_t ads1115_read_channel(uint16_t cfg) {
    if (ads1115_write_reg(ADS1115_REG_CFG, cfg) < 0) return 0;

    for (int i = 0; i < 20; i++) {
        usleep(1000);   // 1 ms per poll
        uint8_t ptr = ADS1115_REG_CFG;
        if (write(i2c_fd, &ptr, 1) != 1) break;
        uint8_t b[2] = {0};
        if (read(i2c_fd, b, 2) != 2) break;
        if (b[0] & 0x80) break;     
    }

    
    uint8_t ptr = ADS1115_REG_CONV;
    if (write(i2c_fd, &ptr, 1) != 1) { perror("I2C write ptr"); return 0; }
    uint8_t b[2] = {0};
    if (read(i2c_fd, b, 2) != 2)    { perror("I2C read");       return 0; }
    int16_t raw = (int16_t)((b[0] << 8) | b[1]);

    // Clamp: negative = below GND (noise), above RAW_AT_3V3 = over 3.3 V
    if (raw < 0)                        raw = 0;
    if (raw > (int16_t)RAW_AT_3V3)     raw = (int16_t)RAW_AT_3V3;

   
    return (uint16_t)((raw * 4095L) / RAW_AT_3V3);
}


static int gpio_init(void) {
    // Open gpiochip0 — returns a handle >= 0 on success
    gpio_h = lgGpiochipOpen(GPIO_CHIP);
    if (gpio_h < 0) {
        fprintf(stderr, "lgGpiochipOpen(%d) failed: %d\n", GPIO_CHIP, gpio_h);
        fprintf(stderr, "Make sure you are running as root (sudo)\n");
        return -1;
    }

    
    lgGpioSetUser(gpio_h, "fsae_system");

   
    int r = lgGpioClaimInput(gpio_h, LG_SET_PULL_DOWN, PIN_SDC);
    if (r < 0) {
        fprintf(stderr, "lgGpioClaimInput(SDC BCM%d) failed: %d\n", PIN_SDC, r);
        return -1;
    }

    r = lgGpioClaimInput(gpio_h, LG_SET_PULL_DOWN, PIN_STARTER);
    if (r < 0) {
        fprintf(stderr, "lgGpioClaimInput(STARTER BCM%d) failed: %d\n", PIN_STARTER, r);
        return -1;
    }

  
   
    r = lgGpioClaimOutput(gpio_h, 0, PIN_BUZZER, 0);
    if (r < 0) {
        fprintf(stderr, "lgGpioClaimOutput(BUZZER BCM%d) failed: %d\n", PIN_BUZZER, r);
        return -1;
    }

    printf("GPIO OK — SDC=BCM%d  STARTER=BCM%d  BUZZER=BCM%d\n",
           PIN_SDC, PIN_STARTER, PIN_BUZZER);
    return 0;
}

static void gpio_cleanup(void) {
    if (gpio_h >= 0) {
        lgGpioWrite(gpio_h, PIN_BUZZER, 0);  // ensure buzzer off on exit
        lgGpioFree(gpio_h, PIN_SDC);
        lgGpioFree(gpio_h, PIN_STARTER);
        lgGpioFree(gpio_h, PIN_BUZZER);
        lgGpiochipClose(gpio_h);
        gpio_h = -1;
    }
}


static bool gpio_read_pin(int pin) {
    int v = lgGpioRead(gpio_h, pin);
    return (v == 1);
}

// Drive buzzer output
static void buzzer_set(bool on) {
    lgGpioWrite(gpio_h, PIN_BUZZER, on ? 1 : 0);
}


int main(void) {
    printf("=== FSAE System Monitor ===\n");
    printf("AIN0=APPS_S1  AIN1=APPS_S2  AIN2=BPPS\n\n");

    if (ads1115_open() < 0) {
        fprintf(stderr, "ADS1115 init failed — check I2C wiring and 'sudo raspi-config'\n");
        return 1;
    }

    if (gpio_init() < 0) {
        fprintf(stderr, "GPIO init failed — exiting\n");
        ads1115_close();
        return 1;
    }

    printf("\n%-5s %-5s %-5s | %-5s %-5s %-5s | %-3s %-3s | %-4s | %-7s | %-6s | %s\n",
           "ADC1","ADC2","ADC0",
           "S1_V","S2_V","BP_V",
           "SDC","BTN",
           "APPS","TORQUE","BUZZER","STATE");
    printf("─────────────────────────────────────────────────────────────────────\n");

    while (1) {
      
        adc[1] = ads1115_read_channel(CFG_AIN0);   // APPS S1
        adc[2] = ads1115_read_channel(CFG_AIN1);   // APPS S2
        adc[0] = ads1115_read_channel(CFG_AIN2);   // BPPS (brake)

        bool sdc_closed  = gpio_read_pin(PIN_SDC);
        bool starter_btn = gpio_read_pin(PIN_STARTER);

        
        long now = get_time_ms();

       
        float torque = R2D_update(adc, sdc_closed, starter_btn, now);

       
        bool apps_fault = r2d_apps_fault();
        bool is_r2d     = r2d_is_active();
        bool buzzer_on  = r2d_sound_active();

       
        buzzer_set(buzzer_on);

       
        float S1_V = (adc[1] / 4095.0f) * 3.3f;
        float S2_V = (1.0f - adc[2] / 4095.0f) * 3.3f;
        float BP_V = (adc[0] / 4095.0f) * 3.3f;

        
        if      (apps_fault)  state = "APPS_FAULT ";
        else if (!sdc_closed) state = "SDC_OPEN   ";
        else if (!is_r2d)     state = "WAITING_R2D";
        else if (buzzer_on)   state = "R2D_BUZZER ";
        else                  state = "R2D_ACTIVE ";

       
        printf("\r%4u  %4u  %4u | %.2fV %.2fV %.2fV | %s %s | %-4s | %6.1fNm | %-6s | %s  ",
               adc[1], adc[2], adc[0],
               S1_V, S2_V, BP_V,
               sdc_closed  ? "CLS" : "OPN",
               starter_btn ? "ON " : "OFF",
               apps_fault  ? "FLT" : "OK",
               torque,
               buzzer_on   ? "ON " : "OFF",
               state);
        fflush(stdout);

        usleep(50000);   
    }

    
    gpio_cleanup();
    ads1115_close();
    return 0;
}
