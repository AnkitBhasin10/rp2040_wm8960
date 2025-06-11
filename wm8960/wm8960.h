#ifndef WM8960_H
#define WM8960_H

#include <cstdint>
#include <cmath>
#include <algorithm>
#include <tuple>
#include "hardware/i2c.h"

// Constants for various gain/level ranges
constexpr float BOOST_GAIN_MIN = -12.00f;
constexpr float BOOST_GAIN_MAX = 6.0f;
constexpr float MIC_GAIN_MIN = -17.25f;
constexpr float MIC_GAIN_MAX = 30.00f;
constexpr float ADC_VOLUME_MIN = -97.00f;
constexpr float ADC_VOLUME_MAX = 30.00f;
constexpr float DAC_VOLUME_MIN = -127.00f;
constexpr float DAC_VOLUME_MAX = 0.00f;
constexpr float ALC_TARGET_MIN = -22.50f;
constexpr float ALC_TARGET_MAX = -1.50f;
constexpr float ALC_MAX_GAIN_MIN = -12.00f;
constexpr float ALC_MAX_GAIN_MAX = 30.00f;
constexpr float ALC_MIN_GAIN_MIN = -17.25f;
constexpr float ALC_MIN_GAIN_MAX = 24.75f;
constexpr float GATE_THRESHOLD_MIN = -76.50f;
constexpr float GATE_THRESHOLD_MAX = -30.00f;
constexpr float OUTPUT_VOLUME_MIN = -21.00f;
constexpr float OUTPUT_VOLUME_MAX = 0.00f;
constexpr float AMP_VOLUME_MIN = -73.00f;
constexpr float AMP_VOLUME_MAX = 6.00f;

// Time constants
constexpr float ALC_ATTACK_TIME_MIN = 0.006f;
constexpr float ALC_ATTACK_TIME_MAX = 6.140f;
constexpr float ALC_DECAY_TIME_MIN = 0.024f;
constexpr float ALC_DECAY_TIME_MAX = 24.580f;
constexpr float ALC_HOLD_TIME_MIN = 0.00267f;
constexpr float ALC_HOLD_TIME_MAX = 43.691f;

// Helper functions
inline float constrain(float value, float min_val, float max_val) {
    return std::max(min_val, std::min(value, max_val));
}

inline float map_range(float value, float from_min, float from_max, float to_min, float to_max) {
    return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min;
}

class WM8960_Advanced;

class Input {
public:
    static constexpr uint8_t DISABLED = 0b000;
    static constexpr uint8_t MIC1 = 0b001;
    static constexpr uint8_t MIC2 = 0b011;
    static constexpr uint8_t MIC3 = 0b101;
    static constexpr uint8_t LINE2 = 0b010;
    static constexpr uint8_t LINE3 = 0b100;
};

class Mic_Input {
public:
    static constexpr uint8_t VMID = 0;
    static constexpr uint8_t INPUT2 = 1;
    static constexpr uint8_t INPUT3 = 2;
};

class Vmid_Mode {
public:
    static constexpr uint8_t DISABLED = 0;
    static constexpr uint8_t PLAYBACK = 1;  // 2X50KOHM
    static constexpr uint8_t LOWPOWER = 2;  // 2X250KOHM
    static constexpr uint8_t FASTSTART = 3; // 2X5KOHM
};

class WM8960_Advanced {
public:
    WM8960_Advanced(i2c_inst_t* i2c, uint8_t address = 0x1A);
    ~WM8960_Advanced();

    void reset();

    // Power management
    bool get_power() const;
    void set_power(bool value);
    uint8_t get_vmid() const;
    void set_vmid(uint8_t value);
    
    // Input
    bool get_left_input() const;
    void set_left_input(bool value);
    bool get_right_input() const;
    void set_right_input(bool value);
    bool get_input() const;
    void set_input(bool value);

    // MIC
    bool get_left_mic() const;
    void set_left_mic(bool value);
    bool get_right_mic() const;
    void set_right_mic(bool value);
    bool get_mic() const;
    void set_mic(bool value);
    
    bool get_left_mic_inverting_input() const;
    void set_left_mic_inverting_input(bool value);
    bool get_right_mic_inverting_input() const;
    void set_right_mic_inverting_input(bool value);
    bool get_mic_inverting_input() const;
    void set_mic_inverting_input(bool value);
    
    uint8_t get_left_mic_input() const;
    void set_left_mic_input(uint8_t value);
    uint8_t get_right_mic_input() const;
    void set_right_mic_input(uint8_t value);
    uint8_t get_mic_input() const;
    void set_mic_input(uint8_t value);
    
    bool get_left_mic_boost() const;
    void set_left_mic_boost(bool value);
    bool get_right_mic_boost() const;
    void set_right_mic_boost(bool value);
    bool get_mic_boost() const;
    void set_mic_boost(bool value);
    
    float get_left_mic_boost_gain() const;
    void set_left_mic_boost_gain(float value);
    float get_right_mic_boost_gain() const;
    void set_right_mic_boost_gain(float value);
    float get_mic_boost_gain() const;
    void set_mic_boost_gain(float value);
    
    float get_left_mic_volume() const;
    void set_left_mic_volume(float value);
    float get_right_mic_volume() const;
    void set_right_mic_volume(float value);
    float get_mic_volume() const;
    void set_mic_volume(float value);
    
    bool get_left_mic_zero_cross() const;
    void set_left_mic_zero_cross(bool value);
    bool get_right_mic_zero_cross() const;
    void set_right_mic_zero_cross(bool value);
    bool get_mic_zero_cross() const;
    void set_mic_zero_cross(bool value);
    
    bool get_left_mic_mute() const;
    void set_left_mic_mute(bool value);
    bool get_right_mic_mute() const;
    void set_right_mic_mute(bool value);
    bool get_mic_mute() const;
    void set_mic_mute(bool value);
    
    // Boost Mixer
    float get_left_input2_boost() const;
    void set_left_input2_boost(float value);
    float get_right_input2_boost() const;
    void set_right_input2_boost(float value);
    float get_input2_boost() const;
    void set_input2_boost(float value);
    
    float get_left_input3_boost() const;
    void set_left_input3_boost(float value);
    float get_right_input3_boost() const;
    void set_right_input3_boost(float value);
    float get_input3_boost() const;
    void set_input3_boost(float value);
    
    // MIC Bias
    bool get_mic_bias() const;
    void set_mic_bias(bool value);
    bool get_mic_bias_voltage() const;
    void set_mic_bias_voltage(bool value);
    
    // ADC
    bool get_left_adc() const;
    void set_left_adc(bool value);
    bool get_right_adc() const;
    void set_right_adc(bool value);
    bool get_adc() const;
    void set_adc(bool value);
    
    float get_left_adc_volume() const;
    void set_left_adc_volume(float value);
    float get_right_adc_volume() const;
    void set_right_adc_volume(float value);
    float get_adc_volume() const;
    void set_adc_volume(float value);
    
    // ALC
    bool get_left_alc() const;
    void set_left_alc(bool value);
    bool get_right_alc() const;
    void set_right_alc(bool value);
    bool get_alc() const;
    void set_alc(bool value);
    
    float get_alc_target() const;
    void set_alc_target(float value);
    float get_alc_max_gain() const;
    void set_alc_max_gain(float value);
    float get_alc_min_gain() const;
    void set_alc_min_gain(float value);
    
    float get_alc_hold_time() const;
    void set_alc_hold_time(float value);
    float get_alc_decay_time() const;
    void set_alc_decay_time(float value);
    float get_alc_attack_time() const;
    void set_alc_attack_time(float value);
    
    bool get_alc_limiter() const;
    void set_alc_limiter(bool value);
    
    // Noise Gate
    bool get_noise_gate() const;
    void set_noise_gate(bool value);
    float get_noise_gate_threshold() const;
    void set_noise_gate_threshold(float value);
    
    // DAC
    bool get_left_dac() const;
    void set_left_dac(bool value);
    bool get_right_dac() const;
    void set_right_dac(bool value);
    bool get_dac() const;
    void set_dac(bool value);
    
    float get_left_dac_volume() const;
    void set_left_dac_volume(float value);
    float get_right_dac_volume() const;
    void set_right_dac_volume(float value);
    float get_dac_volume() const;
    void set_dac_volume(float value);
    
    bool get_dac_mute() const;
    void set_dac_mute(bool value);
    bool get_dac_soft_mute() const;
    void set_dac_soft_mute(bool value);
    bool get_dac_slow_soft_mute() const;
    void set_dac_slow_soft_mute(bool value);
    bool get_dac_attenuation() const;
    void set_dac_attenuation(bool value);
    
    // 3D Enhance
    bool get_enhance() const;
    void set_enhance(bool value);
    bool get_enhance_filter_lpf() const;
    void set_enhance_filter_lpf(bool value);
    bool get_enhance_filter_hpf() const;
    void set_enhance_filter_hpf(bool value);
    
    float get_enhance_depth() const;
    void set_enhance_depth(float value);
    
    // Output Mixer
    bool get_left_output() const;
    void set_left_output(bool value);
    bool get_right_output() const;
    void set_right_output(bool value);
    bool get_output() const;
    void set_output(bool value);
    
    bool get_left_dac_output() const;
    void set_left_dac_output(bool value);
    bool get_right_dac_output() const;
    void set_right_dac_output(bool value);
    bool get_dac_output() const;
    void set_dac_output(bool value);
    
    bool get_left_input3_output() const;
    void set_left_input3_output(bool value);
    bool get_right_input3_output() const;
    void set_right_input3_output(bool value);
    bool get_input3_output() const;
    void set_input3_output(bool value);
    
    float get_left_input3_output_volume() const;
    void set_left_input3_output_volume(float value);
    float get_right_input3_output_volume() const;
    void set_right_input3_output_volume(float value);
    float get_input3_output_volume() const;
    void set_input3_output_volume(float value);
    
    bool get_left_mic_output() const;
    void set_left_mic_output(bool value);
    bool get_right_mic_output() const;
    void set_right_mic_output(bool value);
    bool get_mic_output() const;
    void set_mic_output(bool value);
    
    float get_left_mic_output_volume() const;
    void set_left_mic_output_volume(float value);
    float get_right_mic_output_volume() const;
    void set_right_mic_output_volume(float value);
    float get_mic_output_volume() const;
    void set_mic_output_volume(float value);
    
    bool get_mono_output() const;
    void set_mono_output(bool value);
    bool get_mono_left_mix() const;
    void set_mono_left_mix(bool value);
    bool get_mono_right_mix() const;
    void set_mono_right_mix(bool value);
    bool get_mono_mix() const;
    void set_mono_mix(bool value);
    
    bool get_mono_output_attenuation() const;
    void set_mono_output_attenuation(bool value);
    
    // Amplifier - Headphones
    bool get_left_headphone() const;
    void set_left_headphone(bool value);
    bool get_right_headphone() const;
    void set_right_headphone(bool value);
    bool get_headphone() const;
    void set_headphone(bool value);
    
    bool get_headphone_standby() const;
    void set_headphone_standby(bool value);
    
    float get_left_headphone_volume() const;
    void set_left_headphone_volume(float value);
    float get_right_headphone_volume() const;
    void set_right_headphone_volume(float value);
    float get_headphone_volume() const;
    void set_headphone_volume(float value);
    
    bool get_left_headphone_zero_cross() const;
    void set_left_headphone_zero_cross(bool value);
    bool get_right_headphone_zero_cross() const;
    void set_right_headphone_zero_cross(bool value);
    bool get_headphone_zero_cross() const;
    void set_headphone_zero_cross(bool value);
    
    // Amplifier - Speakers
    bool get_left_speaker() const;
    void set_left_speaker(bool value);
    bool get_right_speaker() const;
    void set_right_speaker(bool value);
    bool get_speaker() const;
    void set_speaker(bool value);
    
    float get_left_speaker_volume() const;
    void set_left_speaker_volume(float value);
    float get_right_speaker_volume() const;
    void set_right_speaker_volume(float value);
    float get_speaker_volume() const;
    void set_speaker_volume(float value);
    
    bool get_left_speaker_zero_cross() const;
    void set_left_speaker_zero_cross(bool value);
    bool get_right_speaker_zero_cross() const;
    void set_right_speaker_zero_cross(bool value);
    bool get_speaker_zero_cross() const;
    void set_speaker_zero_cross(bool value);
    
    float get_speaker_dc_gain() const;
    void set_speaker_dc_gain(float value);
    float get_speaker_ac_gain() const;
    void set_speaker_ac_gain(float value);
    
    // Digital Audio Interface Control
    bool get_loopback() const;
    void set_loopback(bool value);
    bool get_pll() const;
    void set_pll(bool value);
    bool get_pll_prescale_div2() const;
    void set_pll_prescale_div2(bool value);
    uint8_t get_pll_n() const;
    void set_pll_n(uint8_t value);
    uint32_t get_pll_k() const;
    void set_pll_k(uint32_t value);
    
    bool get_clock_fractional_mode() const;
    void set_clock_fractional_mode(bool value);
    bool get_clock_from_pll() const;
    void set_clock_from_pll(bool value);
    
    bool get_system_clock_div2() const;
    void set_system_clock_div2(bool value);
    
    float get_adc_clock_divider() const;
    void set_adc_clock_divider(float value);
    float get_dac_clock_divider() const;
    void set_dac_clock_divider(float value);
    float get_base_clock_divider() const;
    void set_base_clock_divider(float value);
    float get_amp_clock_divider() const;
    void set_amp_clock_divider(float value);
    
    bool get_master_mode() const;
    void set_master_mode(bool value);
    uint8_t get_bit_depth() const;
    void set_bit_depth(uint8_t value);
    
    bool get_word_select_invert() const;
    void set_word_select_invert(bool value);
    bool get_adc_channel_swap() const;
    void set_adc_channel_swap(bool value);
    
    bool get_vref_output() const;
    void set_vref_output(bool value);
    
    float get_power_supply() const;
    void set_power_supply(float value);
    
    bool get_gpio_output() const;
    void set_gpio_output(bool value);
    uint8_t get_gpio_output_mode() const;
    void set_gpio_output_mode(uint8_t value);
    bool get_gpio_output_invert() const;
    void set_gpio_output_invert(bool value);
    
    float get_gpio_clock_divider() const;
    void set_gpio_clock_divider(float value);
    
    int get_sample_rate() const;
    void set_sample_rate(int value);

private:
    i2c_inst_t* _i2c;
    uint8_t _address;
    int _sample_rate;
    
    uint16_t _registers[56]; // 56 registers in WM8960
    
    void _reset_registers();
    void _write_register(uint8_t reg, uint16_t value);
    uint16_t _read_register(uint8_t reg);
    
    static uint8_t _get_mic_boost_gain(float value);
    static uint8_t _get_speaker_boost_gain(float value);
    
    // Helper functions for bit manipulation
    void _set_bit(uint8_t reg, uint8_t bit, bool value);
    bool _get_bit(uint8_t reg, uint8_t bit) const;
    void _set_bits(uint8_t reg, uint8_t mask, uint8_t shift, uint8_t value);
    uint8_t _get_bits(uint8_t reg, uint8_t mask, uint8_t shift) const;
    void error_blink(int code);
};

class WM8960 {
public:
    WM8960(i2c_inst_t* i2c, int sample_rate = 44100, int bit_depth = 16);
    
    // Digital Interface
    int get_sample_rate() const;
    int get_bit_depth() const;
    
    // Input
    uint8_t get_input() const;
    void set_input(uint8_t value);
    
    float get_gain() const;
    void set_gain(float value);
    
    float get_monitor() const;
    void set_monitor(float value);
    
    bool get_loopback() const;
    void set_loopback(bool value);
    
    // Output
    float get_volume() const;
    void set_volume(float value);
    
    float get_headphone() const;
    void set_headphone(float value);
    
    float get_speaker() const;
    void set_speaker(float value);
    
    // 3D Enhance
    float get_enhance() const;
    void set_enhance(float value);
    
    // ALC & Noise Gate
    bool get_alc() const;
    void set_alc(bool value);
    
    std::tuple<float, float, float, float> get_alc_gain() const;
    void set_alc_gain(const std::tuple<float, float, float, float>& value);
    
    std::tuple<float, float, float> get_alc_time() const;
    void set_alc_time(const std::tuple<float, float, float>& value);

private:
    WM8960_Advanced _codec;
    uint8_t _input;
    float _gain;
};

#endif // WM8960_H