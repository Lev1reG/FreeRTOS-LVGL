#include "am2302_rmt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "lvgl_helpers.h"
#include "nvs_flash.h"

// Pin definitions
#define DHT22_GPIO 4

#define LV_TICK_PERIOD_MS 1

static const char *TAG = "DHT22";
static lv_obj_t *sensor_data_label;

// Global variables for storing sensor data
float temperature = 0.0;
float humidity = 0.0;
SemaphoreHandle_t sensor_semaphore;

// Function prototypes
void read_dht22_task(void *pvParameter);
void update_display_task(void *pvParameter);
static void lv_tick_task(void *arg);
void update_sensor_data_label(float temperature, float humidity);

void app_main() {
  // Create FreeRTOS tasks
  xTaskCreate(&read_dht22_task, "read_dht_task", 4096 * 2, NULL, 5, NULL);
  xTaskCreate(&update_display_task, "update_display_task", 4096 * 2, NULL, 5,
              NULL);
}

void read_dht22_task(void *pvParameters) {
  (void)pvParameters;

  am2302_config_t am2302_config = {
      .gpio_num = DHT22_GPIO,
  };
  am2302_rmt_config_t rmt_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
  };
  am2302_handle_t sensor = NULL;
  ESP_ERROR_CHECK(am2302_new_sensor_rmt(&am2302_config, &rmt_config, &sensor));

  while (1) {
    // The delay between each sensor read is required by the datasheet
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_ERROR_CHECK(am2302_read_temp_humi(sensor, &temperature, &humidity));
    ESP_LOGI(TAG, "Temperature: %.1f °C, Humidity: %.1f %%", temperature,
             humidity);
  }
}

void update_display_task(void *pvParameter) {
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
  disp_drv.hor_res = 320;
  disp_drv.ver_res = 240;
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
  update_sensor_data_label(temperature, humidity);
  lv_obj_align(sensor_data_label, LV_ALIGN_CENTER, 0, 0);

  while (1) {
    // Update label text with new sensor data
    update_sensor_data_label(temperature, humidity);
    // Call lv_task_handler to handle LVGL tasks
    lv_task_handler();
    vTaskDelay(pdMS_TO_TICKS(1000)); // Update every second
  }
}

static void lv_tick_task(void *arg) {
  (void)arg;
  lv_tick_inc(LV_TICK_PERIOD_MS);
}

void update_sensor_data_label(float temperature, float humidity) {
  char text[50];
  snprintf(text, sizeof(text), "Temperature: %.1f C\nHumidity: %.1f %%",
           temperature, humidity);
  lv_label_set_text(sensor_data_label, text);
}
