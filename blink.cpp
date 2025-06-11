#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "pico/audio_i2s.h"
#include "wm8960/wm8960.h"
#include "math.h"

// Hardware Configuration
#define PIN_I2S_DATA 9
#define PIN_I2S_BCLK 10
#define PIN_I2C_SDA 0
#define PIN_I2C_SCL 1
#define LED_PIN 25

// Audio Configuration
#define SAMPLE_RATE 44100
#define WAVE_TABLE_LEN 2048
#define SAMPLES_PER_BUFFER 256
#define INITIAL_FREQUENCY 440.0f  // 440Hz sine wave
#define INITIAL_VOLUME 0.8f   
#define INITIAL_FREQ 440.0f    // 80% volume
#define INITIAL_VOL 0.7f

static int16_t sine_wave_table[WAVE_TABLE_LEN];

static int16_t wave_table[WAVE_TABLE_LEN];
enum waveform_type { SINE, SQUARE, SAW, TRIANGLE, NOISE };
static enum waveform_type current_wave = TRIANGLE;

// Error handling
void error_blink(int code) {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        for (int i = 0; i < code; i++) {
            gpio_put(LED_PIN, 1);
            sleep_ms(200);
            gpio_put(LED_PIN, 0);
            sleep_ms(200);
        }
        sleep_ms(1000);
    }
}

// Initialize audio subsystem
struct audio_buffer_pool *init_audio() {
    static audio_format_t audio_format = {
        .sample_freq = SAMPLE_RATE,
        .format = AUDIO_BUFFER_FORMAT_PCM_S16,
        .channel_count = 2,  // Stereo output
    };

    static struct audio_buffer_format producer_format = {
        .format = &audio_format,
        .sample_stride = 4  // 2 bytes per sample * 2 channels
    };

    struct audio_buffer_pool *producer_pool = audio_new_producer_pool(
        &producer_format, 
        3,  // Number of buffers
        SAMPLES_PER_BUFFER
    );

    if (!producer_pool) {
        error_blink(5);  // Audio pool initialization failed
    }

    // I2S configuration
    struct audio_i2s_config config = {
        .data_pin = PIN_I2S_DATA,
        .clock_pin_base = PIN_I2S_BCLK,
        .dma_channel = 0,
        .pio_sm = 1
    };

    const struct audio_format *output_format = audio_i2s_setup(&audio_format, &config);
    if (!output_format) {
        error_blink(10);  // I2S setup failed
    }

    if (!audio_i2s_connect(producer_pool)) {
        error_blink(15);  // I2S connection failed
    }

    audio_i2s_set_enabled(true);
    return producer_pool;
}

// Initialize WM8960 codec
void init_codec() {
    i2c_init(i2c0, 400000);  // 400kHz I2C
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    // Initialize codec with desired sample rate and bit depth
    WM8960 codec(i2c0, SAMPLE_RATE, 16);
    
    // Configure audio paths and volumes
    codec.set_volume(INITIAL_VOLUME);
    codec.set_headphone(INITIAL_VOLUME);
    codec.set_speaker(INITIAL_VOLUME);
    
    // Additional codec configuration if needed
    // codec.enable_dac(true);
    // codec.set_input_source(WM8960_INPUT_MIC);
    
    sleep_ms(100);  // Allow time for codec to initialize
}

// Generate sine wave lookup table
void generate_sine_table() {
    for (int i = 0; i < WAVE_TABLE_LEN; i++) {
        sine_wave_table[i] = 32767 * sinf(i * 2 * (float)(M_PI / WAVE_TABLE_LEN));
    }
}

void generate_waveform(enum waveform_type wave) {
    switch(wave) {
        case SINE:
            for (int i = 0; i < WAVE_TABLE_LEN; i++) {
                wave_table[i] = 32767 * sinf(i * 2 * (float)(M_PI / WAVE_TABLE_LEN));
            }
            break;
            
        case SQUARE:
            for (int i = 0; i < WAVE_TABLE_LEN; i++) {
                wave_table[i] = (i < WAVE_TABLE_LEN/2) ? 32767 : -32767;
            }
            break;
            
        case SAW:
            for (int i = 0; i < WAVE_TABLE_LEN; i++) {
                wave_table[i] = (i * 65536 / WAVE_TABLE_LEN) - 32768;
            }
            break;
            
        case TRIANGLE:
            for (int i = 0; i < WAVE_TABLE_LEN; i++) {
                if (i < WAVE_TABLE_LEN/2) {
                    wave_table[i] = (i * 65536 / (WAVE_TABLE_LEN/2)) - 32768;
                } else {
                    wave_table[i] = 32768 - ((i - WAVE_TABLE_LEN/2) * 65536 / (WAVE_TABLE_LEN/2));
                }
            }
            break;
            
        case NOISE:
            for (int i = 0; i < WAVE_TABLE_LEN; i++) {
                wave_table[i] = (rand() % 65536) - 32768;
            }
            break;
    }
}

// [Keep the init_audio() and init_codec() functions from previous example]

int main() {
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    init_codec();
    generate_waveform(SINE);  // Start with sine wave
    struct audio_buffer_pool *ap = init_audio();

    uint32_t step = (uint32_t)((INITIAL_FREQ * WAVE_TABLE_LEN * 65536) / SAMPLE_RATE);
    uint32_t pos = 0;
    uint8_t vol = (uint8_t)(INITIAL_VOL * 255);
    bool sound_on = true;

    while (true) {

        // Generate audio buffer
        struct audio_buffer *buffer = take_audio_buffer(ap, true);
        if (buffer) {
            int16_t *samples = (int16_t *)buffer->buffer->bytes;
            
            for (uint i = 0; i < buffer->max_sample_count; i++) {
                if (sound_on) {
                    int16_t sample = (vol * wave_table[pos >> 16u]) >> 8u;
                    samples[2*i] = sample;     // Left
                    samples[2*i + 1] = sample; // Right
                    pos += step;
                    if (pos >= WAVE_TABLE_LEN * 65536) pos -= WAVE_TABLE_LEN * 65536;
                } else {
                    samples[2*i] = 0;     // Left
                    samples[2*i + 1] = 0; // Right
                }
            }
            
            buffer->sample_count = buffer->max_sample_count;
            give_audio_buffer(ap, buffer);
        }

        // Add some effects - uncomment what you want to try
        
        //1. Tremolo effect
        // static uint32_t tremolo_pos = 0;
        // vol = (sinf(tremolo_pos * 0.01f) * 127) + 128;
        // tremolo_pos++;
        
        // 2. Frequency sweep
        // static bool sweep_up = true;
        // if (sweep_up) {
        //     step += 100;
        //     if (step > 0x50000) sweep_up = false;
        // } else {
        //     step -= 100;
        //     if (step < 0x10000) sweep_up = true;
        // }
        
        // 3. Pulsing sound
        static uint32_t pulse_counter = 0;
        sound_on = (pulse_counter % 100) < 50;
        pulse_counter++;
    }
}