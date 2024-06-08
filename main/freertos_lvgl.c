#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "extra/widgets/chart/lv_chart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl/lvgl.h"
#include "lvgl_helpers.h"
#include "math.h"
#include "stdio.h"
#include <signal.h>

#define TAG "MPU6050_LVGL"
#define LV_TICK_PERIOD_MS 1

#define who_am_i 0x75
#define I2C_MASTER_FREQ_HZ 100000
#define MPU6050_I2C_ADDR 0x70
#define MPU6050_I2C_CH1 0x01
#define MPU6050_ADD 0x68
#define MPU6050_PWR 0x6B
#define MPU6050_RAW_GRYO 0x43
#define MPU6050_ACK_VAL 0x1
#define MPU6050_NACK_VAL 0x0

uint8_t gyroXH;
uint8_t gyroXL;
uint8_t gyroYH;
uint8_t gyroYL;
uint8_t gyroZH;
uint8_t gyroZL;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;

uint8_t mpu6050_bits;

static lv_obj_t *sensor_data_label;

void i2c_gpio_conf();
void mpu6050_i2c_channel(int mux_channel, int mpu6050_channel);
void mpu6050_init(int mpu6050, int mpu6050_channel);
uint8_t mpu6050_whoAMI(int mpu6050);
void mpu6050_getRawGyro(int mpu6050);

static void lv_tick_task(void *arg);
static void gui_task(void *pvParameter);
static void mpu6050_task(void *pvParameter);

void app_main(void) {
  i2c_gpio_conf();
  // Create a task to handle LVGL
  xTaskCreate(gui_task, "gui_task", 4096 * 2, NULL, 5, NULL);
  xTaskCreate(mpu6050_task, "mpu6050_task", 4096 * 2, NULL, 5, NULL);
}

void i2c_gpio_conf() {
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = 21;
  conf.scl_io_num = 22;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  conf.clk_flags = 0;
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
  ESP_LOGI(TAG, "I2C Controller configured\r\n");

  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
  ESP_LOGI(TAG, "I2C Driver installed\r\n");
}

void mpu6050_i2c_channel(int mux_channel, int mpu6050_channel) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, mpu6050_channel, true);
  i2c_master_stop(cmd);
  if (i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000)) == ESP_OK) {
    ESP_LOGI(TAG, "Channel 0x%02x I2c Mux Selected", mpu6050_channel);
  } else {
    ESP_LOGI(TAG, "MPU6050 i2c Channel is not connected");
  }
  i2c_cmd_link_delete(cmd);
}

void mpu6050_init(int mpu6050, int mpu6050_channel) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (mpu6050 << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, MPU6050_PWR, true);
  i2c_master_write_byte(cmd, 0x00, true);
  i2c_master_stop(cmd);
  if (i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000)) == ESP_OK) {
    ESP_LOGI(TAG, "MPU6050 channel 0x%02x Initialized", mpu6050_channel);
  } else {
    ESP_LOGI(TAG, "MPU6050 is not connected");
  }
  i2c_cmd_link_delete(cmd);
}

uint8_t mpu6050_whoAMI(int mpu6050) {
  uint8_t mpu6050_buffer;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (mpu6050 << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, who_am_i, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (mpu6050 << 1) | I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, &mpu6050_buffer, MPU6050_NACK_VAL);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
  return mpu6050_buffer;
}

void mpu6050_getRawGyro(int mpu6050) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (mpu6050 << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, MPU6050_RAW_GRYO, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (mpu6050 << 1) | I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, &gyroXH, MPU6050_ACK_VAL);
  i2c_master_read_byte(cmd, &gyroXL, MPU6050_ACK_VAL);
  i2c_master_read_byte(cmd, &gyroYH, MPU6050_ACK_VAL);
  i2c_master_read_byte(cmd, &gyroYL, MPU6050_ACK_VAL);
  i2c_master_read_byte(cmd, &gyroZH, MPU6050_ACK_VAL);
  i2c_master_read_byte(cmd, &gyroZL, MPU6050_NACK_VAL);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);

  gyroX = (int16_t)((gyroXH << 8) | gyroXL);
  gyroY = (int16_t)((gyroYH << 8) | gyroYL);
  gyroZ = (int16_t)((gyroZH << 8) | gyroZL);

  float sensitivity = 131.0;
  double fmode(double x, double y);
  gyroX = gyroX / sensitivity;
  gyroY = gyroY / sensitivity;
  gyroZ = gyroZ / sensitivity;

  gyroX = fmod(gyroX + 180.0, 360.0) - 180.0;
  gyroY = fmod(gyroY + 180.0, 360.0) - 180.0;
  gyroZ = fmod(gyroZ + 180.0, 360.0) - 180.0;

  ESP_LOGI(TAG, "Gyro X: %d, Gyro Y: %d, Gyro Z: %d", gyroX, gyroY, gyroZ);
}

static void lv_tick_task(void *arg) {
  (void)arg;
  lv_tick_inc(LV_TICK_PERIOD_MS);
}

void update_sensor_data_label(int16_t gyroX, int16_t gyroY, int16_t gyroZ) {
  char text[50];
  snprintf(text, sizeof(text), "Gyro X: %d\nGyro Y: %d\nGyro Z: %d", gyroX,
           gyroY, gyroZ);
  lv_label_set_text(sensor_data_label, text);
}

static void gui_task(void *pvParameter) {
  (void)pvParameter;

      // Initialize LVGL
      lv_init();

  // Initialize I2C or SPI driver
  lvgl_driver_init();

  // Initialize display buffer
  static lv_color_t buf1[DISP_BUF_SIZE];
  static lv_color_t buf2[DISP_BUF_SIZE];
  static lv_disp_draw_buf_t draw_buf;
  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, DISP_BUF_SIZE);

  // Initialize display driver
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 240;
  disp_drv.ver_res = 320;
  disp_drv.flush_cb = disp_driver_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  // Create a task to handle LVGL tick
  const esp_timer_create_args_t periodic_timer_args = {
      .callback = &lv_tick_task, .name = "periodic_gui"};
  esp_timer_handle_t periodic_timer;
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(
      esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

  // Create a label to display sensor data
  sensor_data_label = lv_label_create(lv_scr_act());
  update_sensor_data_label(gyroX, gyroY, gyroZ);
  lv_obj_align(sensor_data_label, LV_ALIGN_CENTER, 0, 0);

  while (1) {
    // Update label text with new sensor data
    update_sensor_data_label(gyroX, gyroY, gyroZ);
    // Call lv_task_handler to handle LVGL tasks
    lv_task_handler();
    vTaskDelay(pdMS_TO_TICKS(100)); // Update every second
  }

}

void mpu6050_task(void *pvParameter) {
  mpu6050_i2c_channel(MPU6050_I2C_ADDR, MPU6050_I2C_CH1);
  mpu6050_init(MPU6050_ADD, MPU6050_I2C_CH1);

  while (1) {
    mpu6050_bits = mpu6050_whoAMI(MPU6050_ADD);
    ESP_LOGI(TAG, "MPU Name: 0x%02x", mpu6050_bits);
    ESP_LOGI(TAG, "***");
    mpu6050_getRawGyro(MPU6050_ADD);
    ESP_LOGI(TAG, "***");
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
