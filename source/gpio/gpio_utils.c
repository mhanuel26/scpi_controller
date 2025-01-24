
#include "hardware/gpio.h"

#include "gpio_utils.h"

// supported pins
uint pins[] = {PICO_DEFAULT_LED_PIN, 22};


uint32_t pinCount() {
    return sizeof(pins)/sizeof(pins[0]);
}

void initPins() {
    for (uint32_t i = 0; i < pinCount(); i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], 1);
        gpio_put(pins[i], 0);
    }
}

void setPinAt(uint32_t index, bool on) {
    gpio_put(pins[index], on);
}
