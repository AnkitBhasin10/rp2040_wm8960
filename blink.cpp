#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "pio_blink.pio.h"
#include "hardware/clocks.h"
#include "pico/audio_i2s.h" // Include the header for audio_i2s_t
#include "hardware/i2c.h"
#include "wm8960/sparkfun_wm8960.h"
#include "pico/audio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "math.h"

#define PIN_I2S_DATA 9
#define PIN_I2S_BCLK 10

#define SINE_WAVE_TABLE_LEN 2048
#define SAMPLES_PER_BUFFER 256

WM8960 codec;

static int16_t sine_wave_table[SINE_WAVE_TABLE_LEN];

void error_blink(int code) {
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    while (true) {
        for (int i = 0; i < code; i++) {
            gpio_put(25, 1);
            sleep_ms(200);
            gpio_put(25, 0);
            sleep_ms(200);
        }
        sleep_ms(1000);
    }
}

struct audio_buffer_pool *initAudio() {

    static audio_format_t audio_format = {
        .sample_freq = 44100,
        .format = AUDIO_BUFFER_FORMAT_PCM_S16,
        .channel_count = 1,
    };

    static struct audio_buffer_format producer_format = {
        .format = &audio_format,
        .sample_stride = 2
    };


    static audio_buffer_pool *producer_pool = audio_new_producer_pool(&producer_format, 3, 
        SAMPLES_PER_BUFFER);

        if(!producer_pool) {
            error_blink(5);
        }

    bool __unused ok;    

    const struct audio_format *output_format;

    struct audio_i2s_config config = {
        .data_pin = PIN_I2S_DATA,
        .clock_pin_base = PIN_I2S_BCLK,
        .dma_channel = 0,
        .pio_sm = 1
    };

    output_format = audio_i2s_setup(&audio_format, &config);
    if(!output_format) {
        error_blink(10);
    }

    ok = audio_i2s_connect(producer_pool);
    assert(ok);
    audio_i2s_set_enabled(true);

    return producer_pool;
};

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
    uint led_sm = 0;

    uint offset = pio_add_program(pio, &pio_blink_program);

    float freq = 2500;

    //error_blink(3);

    //pio_blink_program_init(pio, sm, offset, 25, freq); // blink default LED pin

    // will blink the led only if a codec is detected via i2c
    i2c_init(i2c0, 100*1000);
    gpio_set_function(0, GPIO_FUNC_I2C); // SDA
    gpio_set_function(1, GPIO_FUNC_I2C); // SCL
    gpio_pull_up(0); // Enable pull-up on SDA
    gpio_pull_up(1); // Enable pull-up on SCL

    sleep_ms(500);

    // if(codec.begin(i2c0)) {
    //     pio_blink_program_init(pio, led_sm, offset, 25, freq);
    // }
        codec.initializeCodec();

        for (int i = 0; i < SINE_WAVE_TABLE_LEN; i++) {
        sine_wave_table[i] = 32767 * cosf(i * 2 * (float) (M_PI / SINE_WAVE_TABLE_LEN));
        }

        struct audio_buffer_pool *ap = initAudio();
        uint32_t step = 0x200000;
        uint32_t pos = 0;
        uint32_t pos_max = 0x10000 * SINE_WAVE_TABLE_LEN;
        uint vol = 128;

        while(true) {
            int c = getchar_timeout_us(0);
            if (c >= 0) {
                if (c == '-' && vol) vol -= 4;
                if ((c == '=' || c == '+') && vol < 255) vol += 4;
                if (c == '[' && step > 0x10000) step -= 0x10000;
                if (c == ']' && step < (SINE_WAVE_TABLE_LEN / 16) * 0x20000) step += 0x10000;
                if (c == 'q') break; 

                struct audio_buffer *buffer = take_audio_buffer(ap, true);
                int16_t *samples = (int16_t *) buffer->buffer->bytes;
                for (uint i = 0; i < buffer->max_sample_count; i++) {
                    samples[2*i]     = (vol * sine_wave_table[pos >> 16u]) >> 8u; // Left
                    samples[2*i + 1] = (vol * sine_wave_table[pos >> 16u]) >> 8u; // Right
                    pos += step;
                    if (pos >= pos_max) pos -= pos_max;
                }
                buffer->sample_count = buffer->max_sample_count;
                give_audio_buffer(ap, buffer);
            }
        }

    while (true) {
    tight_loop_contents(); // Low-power idle
    }
}