#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "get_started/lv_example_get_started.h"
#include "lv_examples.h"
#include "lvgl.h"
#include "lvgl_helpers.h"

#define TAG "lvgl_example"
#define LV_TICK_PERIOD_MS 1

static void lv_tick_task(void *arg);
static void gui_task(void *pvParameter);

void app_main(void) {
  // Create a task to handle LVGL
  xTaskCreate(gui_task, "gui_task", 4096 * 2, NULL, 5, NULL);
}

static void lv_tick_task(void *arg) {
  (void)arg;
  lv_tick_inc(LV_TICK_PERIOD_MS);
}

static void gui_task(void *pvParameter) {
  (void)pvParameter;

  // Initialize LVGl
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

 lv_obj_t *label = lv_label_create(lv_scr_act());
  lv_label_set_text(label, "Hello world!");
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

  while (1) {
    // Call lv_timer handler to handle LVGl tasks
    lv_task_handler();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
