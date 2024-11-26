#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#include "driver/gpio.h"  // Include to access gpio_num_t and GPIO_NUM_x

// Define GPIO pins for each function
#define GPIO_POWER     GPIO_NUM_2   // Power Control
#define GPIO_LOCK      GPIO_NUM_4   // Lock Control
#define GPIO_UNLOCK    GPIO_NUM_5   // Unlock Control
#define GPIO_HORN      GPIO_NUM_18  // Horn Control
#define GPIO_TAILGATE  GPIO_NUM_19  // Tailgate Control

// Create a bitmask for all output GPIOs
#define GPIO_OUTPUT_PIN_SEL  ((1ULL << GPIO_POWER) | \
                               (1ULL << GPIO_LOCK) | \
                               (1ULL << GPIO_UNLOCK) | \
                               (1ULL << GPIO_HORN) | \
                               (1ULL << GPIO_TAILGATE))

// Example input GPIOs (unchanged)
#define GPIO_INPUT_PIN_SEL   ((1ULL << GPIO_NUM_0) | (1ULL << GPIO_NUM_15))

// Function Prototypes
void gpio_control_init(void);
void process_integer_and_control_gpio(int received_int);
void gpio_control_task(void *arg);  // Add this line

#endif // GPIO_CONTROL_H
