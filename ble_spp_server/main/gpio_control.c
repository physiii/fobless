#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "ble_spp_server_demo.h" // Include the header file for the BLE server
#include "gpio_control.h"        // Include its own header

static const char *TAG = "GPIO_CONTROL";

/**
 * @brief Process the received integer and control GPIOs accordingly.
 * 
 * Bit Mapping:
 * Bit 0 (LSB) - Power
 * Bit 1       - Lock
 * Bit 2       - Unlock
 * Bit 3       - Horn
 * Bit 4       - Tailgate
 * 
 * Bit Logic:
 * 0 - Turn ON the GPIO
 * 1 - Turn OFF the GPIO
 * 
 * @param received_int The 8-bit integer representing GPIO states.
 */
void process_integer_and_control_gpio(int received_int)
{
    ESP_LOGI(TAG, "Processing received integer: 0x%02X", received_int);

    // Define an array of GPIOs corresponding to each bit
    const gpio_num_t gpio_pins[5] = {GPIO_POWER, GPIO_LOCK, GPIO_UNLOCK, GPIO_HORN, GPIO_TAILGATE};

    for (int bit = 0; bit < 5; bit++) {
        // Extract the bit value
        int bit_val = (received_int >> bit) & 0x01;

        // Determine the GPIO level based on bit value
        // 0 -> ON (1), 1 -> OFF (0)
        int gpio_level = (bit_val == 0) ? 1 : 0;

        // Set the GPIO level
        gpio_set_level(gpio_pins[bit], gpio_level);
        ESP_LOGI(TAG, "GPIO_%s set to %d", 
                 (bit == 0) ? "POWER" :
                 (bit == 1) ? "LOCK" :
                 (bit == 2) ? "UNLOCK" :
                 (bit == 3) ? "HORN" : "TAILGATE", 
                 gpio_level);
    }
}

void gpio_control_task(void *arg)
{
    cmd_msg_t cmd;

    for(;;){
        if(xQueueReceive(cmd_cmd_queue, &cmd, portMAX_DELAY)) {
            if(cmd.buffer != NULL && cmd.length > 0){
                // Convert buffer to integer
                int received_int = 0;
                for (int i = 0; i < cmd.length; i++) {
                    received_int = (received_int << 8) | cmd.buffer[i];
                }

                ESP_LOGI(TAG, "Received integer: 0x%02X", received_int);

                // Process the integer and control GPIOs
                process_integer_and_control_gpio(received_int);

                free(cmd.buffer);
            } else {
                ESP_LOGE(TAG, "Received invalid command buffer");
            }
        }
    }
    vTaskDelete(NULL);
}

void gpio_control_init(void)
{
    gpio_config_t io_conf;

    // Configure output GPIOs
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Configure input GPIOs (unchanged)
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "GPIOs configured. Output Pins: 0x%llx, Input Pins: 0x%llx", 
             GPIO_OUTPUT_PIN_SEL, GPIO_INPUT_PIN_SEL);

    // Initialize all output GPIOs to OFF (0) initially
    const gpio_num_t gpio_pins[5] = {GPIO_POWER, GPIO_LOCK, GPIO_UNLOCK, GPIO_HORN, GPIO_TAILGATE};
    for (int i = 0; i < 5; i++) {
        gpio_set_level(gpio_pins[i], 0);
    }
    ESP_LOGI(TAG, "All output GPIOs initialized to OFF.");

    // Create the gpio_control_task here
    xTaskCreate(gpio_control_task, "gpio_control_task", 4096, NULL, 10, NULL);
}
