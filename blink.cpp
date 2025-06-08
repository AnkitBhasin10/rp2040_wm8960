#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "pio_blink.pio.h"
#include "hardware/clocks.h"
#include "pico/audio_i2s.h" // Include the header for audio_i2s_t
#include "hardware/i2c.h"
#include "wm8960/sparkfun_wm8960_RegisterMap.cpp"
#include "pico/audio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"

#define PIN_I2S_DATA 9
#define PIN_I2S_BCLK 10
#define PIN_I2S_LRCLK 11

#define AUDIO_SAMPLERATE 44100
#define TONE_FREQ 440.0f
#define SINE_SAMPLES 256

WM8960 codec;
static int16_t sine_buf[SINE_SAMPLES * 2];

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

    if (codec.begin(i2c0)) {
        pio_blink_program_init(pio, sm, offset, 25, freq);

        codec.enableDacLeft();
        codec.enableDacRight();
        codec.setDacLeftDigitalVolumeDB(0.0f);
        codec.setDacRightDigitalVolumeDB(0.0f);

    // Fill sine buffer (stereo)
    for (int i = 0; i < SINE_SAMPLES; i++) {
        float sample = sinf(2.0f * M_PI * (float)i * TONE_FREQ / AUDIO_SAMPLERATE);
        int16_t val = (int16_t)(sample * 30000);
        sine_buf[2*i] = val;
        sine_buf[2*i + 1] = val;
    }

    uint sm = pio_claim_unused_sm(pio, true);

    audio_i2s_config_t config = {
        .data_pin = PIN_I2S_DATA,
        .clock_pin_base = PIN_I2S_BCLK,
        .dma_channel =0,
        .pio_sm = sm
    };

    audio_format_t format = {
        .sample_freq = AUDIO_SAMPLERATE,
        .format = AUDIO_BUFFER_FORMAT_PCM_S16,
        .channel_count = 2
    };

    audio_buffer_format_t producer_format = {
        .format = &format,
        .sample_stride = 4 // 16-bit stereo = 2 bytes * 2 channels
    };

    audio_

    while(true) {
        
    }
    
    }

    while (true) {
    tight_loop_contents(); // Low-power idle
    }
}