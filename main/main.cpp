#include "motor.h"
#include <ctype.h>
#include <string.h>
#include "pid_controller.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"

//Motor with encoder parameters
#define IN_1 GPIO_NUM_26
#define IN_2 GPIO_NUM_27
#define EN_PIN GPIO_NUM_14
#define ENCODER_A GPIO_NUM_18
#define ENCODER_B GPIO_NUM_19
#define RATIO 34
#define CPR 48
#define DT_MS 25

void serial_read_task(void* speed);

extern "C" void app_main(void) {
    //Creates motor with encoder object
    Motor_with_Encoder motor(IN_1, IN_2, EN_PIN, LEDC_CHANNEL_0, LEDC_TIMER_0, ENCODER_A, ENCODER_B, RATIO, CPR, DT_MS);
    motor.set_open_loop_equation(-0.36120745442986935, 4.3087400593523455); //Sets open loop equation (y=mx+b)

    int speed = 0;
    xTaskCreate(serial_read_task, "serial_read_task", 4096, (void*)&speed, 5, NULL);

    //Main task loop
    while (1) {
        motor.set_open_loop_speed(speed, RPM); //Sets speed
        printf("Speed: %f\n", motor.get_speed()); //Prints speed
        vTaskDelay(pdMS_TO_TICKS(50)); //Delay 50ms
    }
}

void serial_read_task(void* speed) {
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);

    char data[6];
    while (1) {
        printf("Set a Goal Speed (-250<->250): ");
        scanf("%5s", data);
        printf("\nSpeed setted to: %s\n", data);
        *(int*)speed = atoi(data);
    }
}