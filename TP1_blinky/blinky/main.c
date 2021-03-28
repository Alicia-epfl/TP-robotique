#include <stm32f4xx.h>
#include <system_clock_config.h>
#include <gpio.h>
#include <main.h>

// Init function required by __libc_init_array
void _init(void) {}

int main(void)
{
    SystemClock_Config();

    // Enable GPIOD peripheral clock
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIODEN;

    // LED used init
    gpio_config_output_opendrain(LED_USED);
    gpio_clear(LED_USED);

    while (1) {
        ;
    }
}
