#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "pio_blink.pio.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "wm8960/sparkfun_wm8960_RegisterMap.cpp"

void pio_blink_program_init(PIO pio, uint sm, uint offset, uint pin, float freq) {
   pio_sm_config c = pio_blink_program_get_default_config(offset);

   //map the state machine's set pin to pin we want to blink
   sm_config_set_set_pins(&c, pin, 1);

   //set this pin's gpio function
   pio_gpio_init(pio, pin);

   //set the pin direction to output of pio
   pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);

   //set the machine clock rate
   sm_config_set_clkdiv(&c, clock_get_hz(clk_sys) / freq);

   //load our config and jump to the start of the program
   pio_sm_init(pio, sm, offset, &c);

   //enable the state machine
   pio_sm_set_enabled(pio, sm, true);
}

int main() {
    PIO pio = pio0;
    uint sm = 0;

    uint offset = pio_add_program(pio, &pio_blink_program);

    float freq = 2500;

    //pio_blink_program_init(pio, sm, offset, 25, freq); // blink default LED pin

    // will blink the led only if a codec is detected via i2c
    i2c_init(i2c0, 100*1000);
    gpio_set_function(0, GPIO_FUNC_I2C); // SDA
    gpio_set_function(1, GPIO_FUNC_I2C); // SCL
    gpio_pull_up(0); // Enable pull-up on SDA
    gpio_pull_up(1); // Enable pull-up on SCL

    sleep_ms(500);

    WM8960 codec; // Assuming you have a WM8960 class defined for codec control

    if (codec.begin(i2c0)) {
        pio_blink_program_init(pio, sm, offset, 25, freq);
    }
    
    while (true) {
    tight_loop_contents(); // Low-power idle
    }
}