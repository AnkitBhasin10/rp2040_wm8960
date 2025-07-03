#include "wm8960.h"
#include "pico/stdlib.h"
#include <cstring>
#include <array>
#include <cmath>

// Static constants
static constexpr std::array<float, 4> _MIC_BOOST_GAIN = {0.0f, 13.0f, 20.0f, 29.0f};
static constexpr std::array<float, 6> _SPEAKER_BOOST_GAIN = {0.0f, 2.1f, 2.9f, 3.6f, 4.5f, 5.1f};

// Clock Divider Tables
static constexpr std::array<float, 14> _BCLKDIV = {1.0f, 1.5f, 2.0f, 3.0f, 4.0f, 5.5f, 6.0f, 8.0f, 11.0f, 12.0f, 16.0f, 22.0f, 24.0f, 32.0f};
static constexpr std::array<float, 8> _DCLKDIV = {1.5f, 2.0f, 3.0f, 4.0f, 6.0f, 8.0f, 12.0f, 16.0f};
static constexpr std::array<float, 7> _ADCDACDIV = {1.0f, 1.5f, 2.0f, 3.0f, 4.0f, 5.5f, 6.0f};
static constexpr std::array<float, 6> _OPCLKDIV = {1.0f, 2.0f, 3.0f, 4.0f, 5.5f, 6.0f};

// Default Register Map
static constexpr std::array<uint16_t, 56> _REG_DEFAULTS = {
    0x0097,  // R0 (0x00)
    0x0097,  // R1 (0x01)
    0x0000,  // R2 (0x02)
    0x0000,  // R3 (0x03)
    0x0000,  // R4 (0x04)
    0x0008,  // R5 (0x05)
    0x0000,  // R6 (0x06)
    0x000A,  // R7 (0x07)
    0x01C0,  // R8 (0x08)
    0x0000,  // R9 (0x09)
    0x00FF,  // R10 (0x0a)
    0x00FF,  // R11 (0x0b)
    0x0000,  // R12 (0x0C) RESERVED
    0x0000,  // R13 (0x0D) RESERVED
    0x0000,  // R14 (0x0E) RESERVED
    0x0000,  // R15 (0x0F) RESERVED
    0x0000,  // R16 (0x10)
    0x007B,  // R17 (0x11)
    0x0100,  // R18 (0x12)
    0x0032,  // R19 (0x13)
    0x0000,  // R20 (0x14)
    0x00C3,  // R21 (0x15)
    0x00C3,  // R22 (0x16)
    0x01C0,  // R23 (0x17)
    0x0000,  // R24 (0x18)
    0x0000,  // R25 (0x19)
    0x0000,  // R26 (0x1A)
    0x0000,  // R27 (0x1B)
    0x0000,  // R28 (0x1C)
    0x0000,  // R29 (0x1D)
    0x0000,  // R30 (0x1E) RESERVED
    0x0000,  // R31 (0x1F) RESERVED
    0x0100,  // R32 (0x20)
    0x0100,  // R33 (0x21)
    0x0050,  // R34 (0x22)
    0x0000,  // R35 (0x23) RESERVED
    0x0000,  // R36 (0x24) RESERVED
    0x0050,  // R37 (0x25)
    0x0000,  // R38 (0x26)
    0x0000,  // R39 (0x27)
    0x0000,  // R40 (0x28)
    0x0000,  // R41 (0x29)
    0x0040,  // R42 (0x2A)
    0x0000,  // R43 (0x2B)
    0x0000,  // R44 (0x2C)
    0x0050,  // R45 (0x2D)
    0x0050,  // R46 (0x2E)
    0x0000,  // R47 (0x2F)
    0x0002,  // R48 (0x30)
    0x0037,  // R49 (0x31)
    0x0000,  // R50 (0x32) RESERVED
    0x0080,  // R51 (0x33)
    0x0008,  // R52 (0x34)
    0x0031,  // R53 (0x35)
    0x0026,  // R54 (0x36)
    0x00E9   // R55 (0x37)
};

// Helper functions
uint8_t WM8960_Advanced::_get_mic_boost_gain(float value) {
    for (int i = _MIC_BOOST_GAIN.size() - 1; i >= 0; i--) {
        if (value >= _MIC_BOOST_GAIN[i]) {
            return i;
        }
    }
    return 0;
}

uint8_t WM8960_Advanced::_get_speaker_boost_gain(float value) {
    for (int i = _SPEAKER_BOOST_GAIN.size() - 1; i >= 0; i--) {
        if (value >= _SPEAKER_BOOST_GAIN[i]) {
            return i;
        }
    }
    return 0;
}

// Bit manipulation helpers
void WM8960_Advanced::_set_bit(uint8_t reg, uint8_t bit, bool value) {
    if (value) {
        _registers[reg] |= (1 << bit);
    } else {
        _registers[reg] &= ~(1 << bit);
    }
    _write_register(reg, _registers[reg]);
}

bool WM8960_Advanced::_get_bit(uint8_t reg, uint8_t bit) const {
    return (_registers[reg] >> bit) & 0x01;
}

void WM8960_Advanced::_set_bits(uint8_t reg, uint8_t mask, uint8_t shift, uint8_t value) {
    _registers[reg] &= ~(mask << shift);
    _registers[reg] |= (value & mask) << shift;
    _write_register(reg, _registers[reg]);
}

uint8_t WM8960_Advanced::_get_bits(uint8_t reg, uint8_t mask, uint8_t shift) const {
    return (_registers[reg] >> shift) & mask;
}

// WM8960_Advanced implementation
WM8960_Advanced::WM8960_Advanced(i2c_inst_t* i2c, uint8_t address) 
    : _i2c(i2c), _address(address), _sample_rate(0) {
    _reset_registers();
    reset();
    
    // General setup
    set_power(true);
    set_vmid(Vmid_Mode::PLAYBACK);
}

WM8960_Advanced::~WM8960_Advanced() {
    // Cleanup if needed
}

void WM8960_Advanced::_reset_registers() {
    std::copy(_REG_DEFAULTS.begin(), _REG_DEFAULTS.end(), _registers);
}

void WM8960_Advanced::reset() {
    // Set reset bit
    _registers[0x0F] |= (1 << 7);
    _write_register(0x0F, _registers[0x0F]);
    
    // Reset all registers to defaults
    _reset_registers();
    
    // Write all registers
    for (uint8_t i = 0; i < 56; i++) {
        _write_register(i, _registers[i]);
    }
}

void WM8960_Advanced::_write_register(uint8_t reg, uint16_t value) {
    uint8_t control_byte[2];
    control_byte[0] = (reg << 1) | (value >> 8);
    control_byte[1] =  value & 0xFF;
    
    i2c_write_blocking(_i2c, _address, control_byte, 2, false);
}

void WM8960_Advanced::error_blink(int code) {
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    while(true) {
        for(int i=0; i<code; i++) {
            gpio_put(25, 1);
            sleep_ms(500);
            gpio_put(25,0);
            sleep_ms(500);
        }
        sleep_ms(1000);
    }
}

uint16_t WM8960_Advanced::_read_register(uint8_t reg) {
    uint8_t data[2];
    uint8_t reg_addr = (reg << 1);
    
    i2c_write_blocking(_i2c, _address, &reg_addr, 1, true);
    i2c_read_blocking(_i2c, _address, data, 2, false);
    
    return (data[0] << 8) | data[1];
}

// Power management
bool WM8960_Advanced::get_power() const { return _get_bit(0x19, 6); }
void WM8960_Advanced::set_power(bool value) { _set_bit(0x19, 6, value); }

uint8_t WM8960_Advanced::get_vmid() const { return _get_bits(0x19, 0x03, 7); }
void WM8960_Advanced::set_vmid(uint8_t value) { _set_bits(0x19, 0x03, 7, value); }

// Input
bool WM8960_Advanced::get_left_input() const { return _get_bit(0x19, 5); }
void WM8960_Advanced::set_left_input(bool value) { _set_bit(0x19, 5, value); }

bool WM8960_Advanced::get_right_input() const { return _get_bit(0x19, 4); }
void WM8960_Advanced::set_right_input(bool value) { _set_bit(0x19, 4, value); }

bool WM8960_Advanced::get_input() const { return get_left_input() && get_right_input(); }
void WM8960_Advanced::set_input(bool value) { set_left_input(value); set_right_input(value); }

// MIC
bool WM8960_Advanced::get_left_mic() const { return _get_bit(0x2F, 5); }
void WM8960_Advanced::set_left_mic(bool value) { _set_bit(0x2F, 5, value); }

bool WM8960_Advanced::get_right_mic() const { return _get_bit(0x2F, 4); }
void WM8960_Advanced::set_right_mic(bool value) { _set_bit(0x2F, 4, value); }

bool WM8960_Advanced::get_mic() const { return get_left_mic() && get_right_mic(); }
void WM8960_Advanced::set_mic(bool value) { set_left_mic(value); set_right_mic(value); }

bool WM8960_Advanced::get_left_mic_inverting_input() const { return _get_bit(0x20, 8); }
void WM8960_Advanced::set_left_mic_inverting_input(bool value) { _set_bit(0x20, 8, value); }

bool WM8960_Advanced::get_right_mic_inverting_input() const { return _get_bit(0x21, 8); }
void WM8960_Advanced::set_right_mic_inverting_input(bool value) { _set_bit(0x21, 8, value); }

bool WM8960_Advanced::get_mic_inverting_input() const { 
    return get_left_mic_inverting_input() && get_right_mic_inverting_input(); 
}
void WM8960_Advanced::set_mic_inverting_input(bool value) { 
    set_left_mic_inverting_input(value); 
    set_right_mic_inverting_input(value); 
}

uint8_t WM8960_Advanced::get_left_mic_input() const {
    if (_get_bit(0x20, 6)) return Mic_Input::INPUT2;
    if (_get_bit(0x20, 7)) return Mic_Input::INPUT3;
    return Mic_Input::VMID;
}

void WM8960_Advanced::set_left_mic_input(uint8_t value) {
    _set_bit(0x20, 6, value == Mic_Input::INPUT2);
    _set_bit(0x20, 7, value == Mic_Input::INPUT3);
}

uint8_t WM8960_Advanced::get_right_mic_input() const {
    if (_get_bit(0x21, 6)) return Mic_Input::INPUT2;
    if (_get_bit(0x21, 7)) return Mic_Input::INPUT3;
    return Mic_Input::VMID;
}

void WM8960_Advanced::set_right_mic_input(uint8_t value) {
    _set_bit(0x21, 6, value == Mic_Input::INPUT2);
    _set_bit(0x21, 7, value == Mic_Input::INPUT3);
}

uint8_t WM8960_Advanced::get_mic_input() const { return get_left_mic_input(); }
void WM8960_Advanced::set_mic_input(uint8_t value) { 
    set_left_mic_input(value); 
    set_right_mic_input(value); 
}

bool WM8960_Advanced::get_left_mic_boost() const { return _get_bit(0x20, 3); }
void WM8960_Advanced::set_left_mic_boost(bool value) { _set_bit(0x20, 3, value); }

bool WM8960_Advanced::get_right_mic_boost() const { return _get_bit(0x21, 3); }
void WM8960_Advanced::set_right_mic_boost(bool value) { _set_bit(0x21, 3, value); }

bool WM8960_Advanced::get_mic_boost() const { return get_left_mic_boost() && get_right_mic_boost(); }
void WM8960_Advanced::set_mic_boost(bool value) { 
    set_left_mic_boost(value); 
    set_right_mic_boost(value); 
}

float WM8960_Advanced::get_left_mic_boost_gain() const {
    return _MIC_BOOST_GAIN[_get_bits(0x20, 0x03, 4)];
}

void WM8960_Advanced::set_left_mic_boost_gain(float value) {
    _set_bits(0x20, 0x03, 4, _get_mic_boost_gain(value));
}

float WM8960_Advanced::get_right_mic_boost_gain() const {
    return _MIC_BOOST_GAIN[_get_bits(0x21, 0x03, 4)];
}

void WM8960_Advanced::set_right_mic_boost_gain(float value) {
    _set_bits(0x21, 0x03, 4, _get_mic_boost_gain(value));
}

float WM8960_Advanced::get_mic_boost_gain() const {
    return std::max(get_left_mic_boost_gain(), get_right_mic_boost_gain());
}

void WM8960_Advanced::set_mic_boost_gain(float value) {
    uint8_t gain = _get_mic_boost_gain(value);
    _set_bits(0x20, 0x03, 4, gain);
    _set_bits(0x21, 0x03, 4, gain);
}

float WM8960_Advanced::get_left_mic_volume() const {
    if (!_get_bit(0x00, 8)) return 0.0f; // Not set
    return map_range(_get_bits(0x00, 0x3F, 0), 0, 63, MIC_GAIN_MIN, MIC_GAIN_MAX);
}

void WM8960_Advanced::set_left_mic_volume(float value) {
    _set_bits(0x00, 0x3F, 0, round(map_range(value, MIC_GAIN_MIN, MIC_GAIN_MAX, 0, 63)));
    _set_bit(0x00, 8, true);
}

float WM8960_Advanced::get_right_mic_volume() const {
    if (!_get_bit(0x01, 8)) return 0.0f; // Not set
    return map_range(_get_bits(0x01, 0x3F, 0), 0, 63, MIC_GAIN_MIN, MIC_GAIN_MAX);
}

void WM8960_Advanced::set_right_mic_volume(float value) {
    _set_bits(0x01, 0x3F, 0, round(map_range(value, MIC_GAIN_MIN, MIC_GAIN_MAX, 0, 63)));
    _set_bit(0x01, 8, true);
}

float WM8960_Advanced::get_mic_volume() const {
    return std::max(get_left_mic_volume(), get_right_mic_volume());
}

void WM8960_Advanced::set_mic_volume(float value) {
    uint8_t vol = round(map_range(value, MIC_GAIN_MIN, MIC_GAIN_MAX, 0, 63));
    _set_bits(0x00, 0x3F, 0, vol);
    _set_bits(0x01, 0x3F, 0, vol);
    _set_bit(0x00, 8, true);
    _set_bit(0x01, 8, true);
}

bool WM8960_Advanced::get_left_mic_zero_cross() const { return _get_bit(0x00, 6); }
void WM8960_Advanced::set_left_mic_zero_cross(bool value) { _set_bit(0x00, 6, value); }

bool WM8960_Advanced::get_right_mic_zero_cross() const { return _get_bit(0x01, 6); }
void WM8960_Advanced::set_right_mic_zero_cross(bool value) { _set_bit(0x01, 6, value); }

bool WM8960_Advanced::get_mic_zero_cross() const { 
    return get_left_mic_zero_cross() && get_right_mic_zero_cross(); 
}
void WM8960_Advanced::set_mic_zero_cross(bool value) { 
    set_left_mic_zero_cross(value); 
    set_right_mic_zero_cross(value); 
}

bool WM8960_Advanced::get_left_mic_mute() const { return _get_bit(0x00, 7); }
void WM8960_Advanced::set_left_mic_mute(bool value) { 
    _set_bit(0x00, 7, value); 
    _set_bit(0x00, 8, true); 
}

bool WM8960_Advanced::get_right_mic_mute() const { return _get_bit(0x01, 7); }
void WM8960_Advanced::set_right_mic_mute(bool value) { 
    _set_bit(0x01, 7, value); 
    _set_bit(0x01, 8, true); 
}

bool WM8960_Advanced::get_mic_mute() const { return get_left_mic_mute() && get_right_mic_mute(); }
void WM8960_Advanced::set_mic_mute(bool value) { 
    set_left_mic_mute(value); 
    set_right_mic_mute(value); 
}

// Boost Mixer
float WM8960_Advanced::get_left_input2_boost() const {
    uint8_t val = _get_bits(0x2B, 0x07, 1);
    return val == 0 ? BOOST_GAIN_MIN - 1.0f : map_range(val, 1, 7, BOOST_GAIN_MIN, BOOST_GAIN_MAX);
}

void WM8960_Advanced::set_left_input2_boost(float value) {
    _set_bits(0x2B, 0x07, 1, value < BOOST_GAIN_MIN ? 0 : round(map_range(value, BOOST_GAIN_MIN, BOOST_GAIN_MAX, 1, 7)));
}

float WM8960_Advanced::get_right_input2_boost() const {
    uint8_t val = _get_bits(0x2C, 0x07, 1);
    return val == 0 ? BOOST_GAIN_MIN - 1.0f : map_range(val, 1, 7, BOOST_GAIN_MIN, BOOST_GAIN_MAX);
}

void WM8960_Advanced::set_right_input2_boost(float value) {
    _set_bits(0x2C, 0x07, 1, value < BOOST_GAIN_MIN ? 0 : round(map_range(value, BOOST_GAIN_MIN, BOOST_GAIN_MAX, 1, 7)));
}

float WM8960_Advanced::get_input2_boost() const {
    return std::max(get_left_input2_boost(), get_right_input2_boost());
}

void WM8960_Advanced::set_input2_boost(float value) {
    uint8_t boost = value < BOOST_GAIN_MIN ? 0 : round(map_range(value, BOOST_GAIN_MIN, BOOST_GAIN_MAX, 1, 7));
    _set_bits(0x2B, 0x07, 1, boost);
    _set_bits(0x2C, 0x07, 1, boost);
}

float WM8960_Advanced::get_left_input3_boost() const {
    uint8_t val = _get_bits(0x2B, 0x07, 4);
    return val == 0 ? BOOST_GAIN_MIN - 1.0f : map_range(val, 1, 7, BOOST_GAIN_MIN, BOOST_GAIN_MAX);
}

void WM8960_Advanced::set_left_input3_boost(float value) {
    _set_bits(0x2B, 0x07, 4, value < BOOST_GAIN_MIN ? 0 : round(map_range(value, BOOST_GAIN_MIN, BOOST_GAIN_MAX, 1, 7)));
}

float WM8960_Advanced::get_right_input3_boost() const {
    uint8_t val = _get_bits(0x2C, 0x07, 4);
    return val == 0 ? BOOST_GAIN_MIN - 1.0f : map_range(val, 1, 7, BOOST_GAIN_MIN, BOOST_GAIN_MAX);
}

void WM8960_Advanced::set_right_input3_boost(float value) {
    _set_bits(0x2C, 0x07, 4, value < BOOST_GAIN_MIN ? 0 : round(map_range(value, BOOST_GAIN_MIN, BOOST_GAIN_MAX, 1, 7)));
}

float WM8960_Advanced::get_input3_boost() const {
    return std::max(get_left_input3_boost(), get_right_input3_boost());
}

void WM8960_Advanced::set_input3_boost(float value) {
    uint8_t boost = value < BOOST_GAIN_MIN ? 0 : round(map_range(value, BOOST_GAIN_MIN, BOOST_GAIN_MAX, 1, 7));
    _set_bits(0x2B, 0x07, 4, boost);
    _set_bits(0x2C, 0x07, 4, boost);
}

// MIC Bias
bool WM8960_Advanced::get_mic_bias() const { return _get_bit(0x19, 1); }
void WM8960_Advanced::set_mic_bias(bool value) { _set_bit(0x19, 1, value); }

bool WM8960_Advanced::get_mic_bias_voltage() const { return _get_bit(0x30, 0); }
void WM8960_Advanced::set_mic_bias_voltage(bool value) { _set_bit(0x30, 0, value); }

// ADC
bool WM8960_Advanced::get_left_adc() const { return _get_bit(0x19, 3); }
void WM8960_Advanced::set_left_adc(bool value) { _set_bit(0x19, 3, value); }

bool WM8960_Advanced::get_right_adc() const { return _get_bit(0x19, 2); }
void WM8960_Advanced::set_right_adc(bool value) { _set_bit(0x19, 2, value); }

bool WM8960_Advanced::get_adc() const { return get_left_adc() && get_right_adc(); }
void WM8960_Advanced::set_adc(bool value) { set_left_adc(value); set_right_adc(value); }

float WM8960_Advanced::get_left_adc_volume() const {
    uint8_t val = _get_bits(0x15, 0xFF, 0);
    return val == 0 ? ADC_VOLUME_MIN - 1.0f : map_range(val, 1, 255, ADC_VOLUME_MIN, ADC_VOLUME_MAX);
}

void WM8960_Advanced::set_left_adc_volume(float value) {
    _set_bits(0x15, 0xFF, 0, value < ADC_VOLUME_MIN ? 0 : round(map_range(value, ADC_VOLUME_MIN, ADC_VOLUME_MAX, 1, 255)));
    _set_bit(0x15, 8, true);
}

float WM8960_Advanced::get_right_adc_volume() const {
    uint8_t val = _get_bits(0x16, 0xFF, 0);
    return val == 0 ? ADC_VOLUME_MIN - 1.0f : map_range(val, 1, 255, ADC_VOLUME_MIN, ADC_VOLUME_MAX);
}

void WM8960_Advanced::set_right_adc_volume(float value) {
    _set_bits(0x16, 0xFF, 0, value < ADC_VOLUME_MIN ? 0 : round(map_range(value, ADC_VOLUME_MIN, ADC_VOLUME_MAX, 1, 255)));
    _set_bit(0x16, 8, true);
}

float WM8960_Advanced::get_adc_volume() const {
    return std::max(get_left_adc_volume(), get_right_adc_volume());
}

void WM8960_Advanced::set_adc_volume(float value) {
    uint8_t vol = value < ADC_VOLUME_MIN ? 0 : round(map_range(value, ADC_VOLUME_MIN, ADC_VOLUME_MAX, 1, 255));
    _set_bits(0x15, 0xFF, 0, vol);
    _set_bits(0x16, 0xFF, 0, vol);
    _set_bit(0x15, 8, true);
    _set_bit(0x16, 8, true);
}

// ALC
bool WM8960_Advanced::get_left_alc() const { return _get_bit(0x11, 7); }
void WM8960_Advanced::set_left_alc(bool value) { _set_bit(0x11, 7, value); }

bool WM8960_Advanced::get_right_alc() const { return _get_bit(0x11, 8); }
void WM8960_Advanced::set_right_alc(bool value) { _set_bit(0x11, 8, value); }

bool WM8960_Advanced::get_alc() const { return get_left_alc() && get_right_alc(); }
void WM8960_Advanced::set_alc(bool value) { set_left_alc(value); set_right_alc(value); }

float WM8960_Advanced::get_alc_target() const {
    return map_range(_get_bits(0x11, 0x0F, 0), 0, 15, ALC_TARGET_MIN, ALC_TARGET_MAX);
}

void WM8960_Advanced::set_alc_target(float value) {
    _set_bits(0x11, 0x0F, 0, round(map_range(value, ALC_TARGET_MIN, ALC_TARGET_MAX, 0, 15)));
}

float WM8960_Advanced::get_alc_max_gain() const {
    return map_range(_get_bits(0x11, 0x07, 4), 0, 7, ALC_MAX_GAIN_MIN, ALC_MAX_GAIN_MAX);
}

void WM8960_Advanced::set_alc_max_gain(float value) {
    _set_bits(0x11, 0x07, 4, round(map_range(value, ALC_MAX_GAIN_MIN, ALC_MAX_GAIN_MAX, 0, 7)));
}

float WM8960_Advanced::get_alc_min_gain() const {
    return map_range(_get_bits(0x12, 0x07, 4), 0, 7, ALC_MIN_GAIN_MIN, ALC_MIN_GAIN_MAX);
}

void WM8960_Advanced::set_alc_min_gain(float value) {
    _set_bits(0x12, 0x07, 4, round(map_range(value, ALC_MIN_GAIN_MIN, ALC_MIN_GAIN_MAX, 0, 7)));
}

float WM8960_Advanced::get_alc_hold_time() const {
    uint8_t val = _get_bits(0x12, 0x0F, 0);
    return val == 0 ? 0.0f : ALC_HOLD_TIME_MIN * powf(2.0f, val - 1);
}

void WM8960_Advanced::set_alc_hold_time(float value) {
    uint8_t hold = 0;
    if (value > 0.0f) {
        hold = round(log2f(constrain(value, ALC_HOLD_TIME_MIN, ALC_HOLD_TIME_MAX) / ALC_HOLD_TIME_MIN)) + 1;
    }
    _set_bits(0x12, 0x0F, 0, hold);
}

float WM8960_Advanced::get_alc_decay_time() const {
    return ALC_DECAY_TIME_MIN * powf(2.0f, _get_bits(0x13, 0x0F, 4));
}

void WM8960_Advanced::set_alc_decay_time(float value) {
    uint8_t decay = round(log2f(constrain(value, ALC_DECAY_TIME_MIN, ALC_DECAY_TIME_MAX) / ALC_DECAY_TIME_MIN));
    _set_bits(0x13, 0x0F, 4, decay);
}

float WM8960_Advanced::get_alc_attack_time() const {
    return ALC_ATTACK_TIME_MIN * powf(2.0f, _get_bits(0x13, 0x0F, 0));
}

void WM8960_Advanced::set_alc_attack_time(float value) {
    uint8_t attack = round(log2f(constrain(value, ALC_ATTACK_TIME_MIN, ALC_ATTACK_TIME_MAX) / ALC_ATTACK_TIME_MIN));
    _set_bits(0x13, 0x0F, 0, attack);
}

bool WM8960_Advanced::get_alc_limiter() const { return _get_bit(0x13, 8); }
void WM8960_Advanced::set_alc_limiter(bool value) { _set_bit(0x13, 8, value); }

// Noise Gate
bool WM8960_Advanced::get_noise_gate() const { return _get_bit(0x14, 0); }
void WM8960_Advanced::set_noise_gate(bool value) { _set_bit(0x14, 0, value); }

float WM8960_Advanced::get_noise_gate_threshold() const {
    return map_range(_get_bits(0x14, 0x1F, 3), 0, 31, GATE_THRESHOLD_MIN, GATE_THRESHOLD_MAX);
}

void WM8960_Advanced::set_noise_gate_threshold(float value) {
    _set_bits(0x14, 0x1F, 3, round(map_range(value, GATE_THRESHOLD_MIN, GATE_THRESHOLD_MAX, 0, 31)));
}

// DAC
bool WM8960_Advanced::get_left_dac() const { return _get_bit(0x1A, 8); }
void WM8960_Advanced::set_left_dac(bool value) { _set_bit(0x1A, 8, value); }

bool WM8960_Advanced::get_right_dac() const { return _get_bit(0x1A, 7); }
void WM8960_Advanced::set_right_dac(bool value) { _set_bit(0x1A, 7, value); }

bool WM8960_Advanced::get_dac() const { return get_left_dac() && get_right_dac(); }
void WM8960_Advanced::set_dac(bool value) { set_left_dac(value); set_right_dac(value); }

float WM8960_Advanced::get_left_dac_volume() const {
    return map_range(_get_bits(0x0A, 0xFF, 0), 1, 255, DAC_VOLUME_MIN, DAC_VOLUME_MAX);
}

void WM8960_Advanced::set_left_dac_volume(float value) {
    _set_bits(0x0A, 0xFF, 0, round(map_range(value, DAC_VOLUME_MIN, DAC_VOLUME_MAX, 1, 255)));
    _set_bit(0x0A, 8, true);
}

float WM8960_Advanced::get_right_dac_volume() const {
    return map_range(_get_bits(0x0B, 0xFF, 0), 1, 255, DAC_VOLUME_MIN, DAC_VOLUME_MAX);
}

void WM8960_Advanced::set_right_dac_volume(float value) {
    _set_bits(0x0B, 0xFF, 0, round(map_range(value, DAC_VOLUME_MIN, DAC_VOLUME_MAX, 1, 255)));
    _set_bit(0x0B, 8, true);
}

float WM8960_Advanced::get_dac_volume() const {
    return std::max(get_left_dac_volume(), get_right_dac_volume());
}

void WM8960_Advanced::set_dac_volume(float value) {
    uint8_t vol = round(map_range(value, DAC_VOLUME_MIN, DAC_VOLUME_MAX, 1, 255));
    _set_bits(0x0A, 0xFF, 0, vol);
    _set_bits(0x0B, 0xFF, 0, vol);
    _set_bit(0x0A, 8, true);
    _set_bit(0x0B, 8, true);
}

bool WM8960_Advanced::get_dac_mute() const { return _get_bit(0x05, 3); }
void WM8960_Advanced::set_dac_mute(bool value) { _set_bit(0x05, 3, value); }

bool WM8960_Advanced::get_dac_soft_mute() const { return _get_bit(0x06, 3); }
void WM8960_Advanced::set_dac_soft_mute(bool value) { _set_bit(0x06, 3, value); }

bool WM8960_Advanced::get_dac_slow_soft_mute() const { return _get_bit(0x06, 2); }
void WM8960_Advanced::set_dac_slow_soft_mute(bool value) { _set_bit(0x06, 2, value); }

bool WM8960_Advanced::get_dac_attenuation() const { return _get_bit(0x05, 7); }
void WM8960_Advanced::set_dac_attenuation(bool value) { _set_bit(0x05, 7, value); }

// 3D Enhance
bool WM8960_Advanced::get_enhance() const { return _get_bit(0x10, 0); }
void WM8960_Advanced::set_enhance(bool value) { _set_bit(0x10, 0, value); }

bool WM8960_Advanced::get_enhance_filter_lpf() const { return _get_bit(0x10, 6); }
void WM8960_Advanced::set_enhance_filter_lpf(bool value) { _set_bit(0x10, 6, value); }

bool WM8960_Advanced::get_enhance_filter_hpf() const { return _get_bit(0x10, 5); }
void WM8960_Advanced::set_enhance_filter_hpf(bool value) { _set_bit(0x10, 5, value); }

float WM8960_Advanced::get_enhance_depth() const {
    return _get_bits(0x10, 0x0F, 1) / 15.0f;
}

void WM8960_Advanced::set_enhance_depth(float value) {
    _set_bits(0x10, 0x0F, 1, round(map_range(value, 0.0f, 1.0f, 0, 15)));
}

// Output Mixer
bool WM8960_Advanced::get_left_output() const { return _get_bit(0x2F, 3); }
void WM8960_Advanced::set_left_output(bool value) { _set_bit(0x2F, 3, value); }

bool WM8960_Advanced::get_right_output() const { return _get_bit(0x2F, 2); }
void WM8960_Advanced::set_right_output(bool value) { _set_bit(0x2F, 2, value); }

bool WM8960_Advanced::get_output() const { return get_left_output() && get_right_output(); }
void WM8960_Advanced::set_output(bool value) { set_left_output(value); set_right_output(value); }

bool WM8960_Advanced::get_left_dac_output() const { return _get_bit(0x22, 8); }
void WM8960_Advanced::set_left_dac_output(bool value) { _set_bit(0x22, 8, value); }

bool WM8960_Advanced::get_right_dac_output() const { return _get_bit(0x25, 8); }
void WM8960_Advanced::set_right_dac_output(bool value) { _set_bit(0x25, 8, value); }

bool WM8960_Advanced::get_dac_output() const { return get_left_dac_output() && get_right_dac_output(); }
void WM8960_Advanced::set_dac_output(bool value) { 
    set_left_dac_output(value); 
    set_right_dac_output(value); 
}

bool WM8960_Advanced::get_left_input3_output() const { return _get_bit(0x22, 7); }
void WM8960_Advanced::set_left_input3_output(bool value) { _set_bit(0x22, 7, value); }

bool WM8960_Advanced::get_right_input3_output() const { return _get_bit(0x25, 7); }
void WM8960_Advanced::set_right_input3_output(bool value) { _set_bit(0x25, 7, value); }

bool WM8960_Advanced::get_input3_output() const { return get_left_input3_output() && get_right_input3_output(); }
void WM8960_Advanced::set_input3_output(bool value) { 
    set_left_input3_output(value); 
    set_right_input3_output(value); 
}

float WM8960_Advanced::get_left_input3_output_volume() const {
    return map_range(_get_bits(0x22, 0x07, 4), 0, 7, OUTPUT_VOLUME_MAX, OUTPUT_VOLUME_MIN);
}

void WM8960_Advanced::set_left_input3_output_volume(float value) {
    _set_bits(0x22, 0x07, 4, round(map_range(value, OUTPUT_VOLUME_MIN, OUTPUT_VOLUME_MAX, 7, 0)));
}

float WM8960_Advanced::get_right_input3_output_volume() const {
    return map_range(_get_bits(0x25, 0x07, 4), 0, 7, OUTPUT_VOLUME_MAX, OUTPUT_VOLUME_MIN);
}

void WM8960_Advanced::set_right_input3_output_volume(float value) {
    _set_bits(0x25, 0x07, 4, round(map_range(value, OUTPUT_VOLUME_MIN, OUTPUT_VOLUME_MAX, 7, 0)));
}

float WM8960_Advanced::get_input3_output_volume() const {
    return std::max(get_left_input3_output_volume(), get_right_input3_output_volume());
}

void WM8960_Advanced::set_input3_output_volume(float value) {
    uint8_t vol = round(map_range(value, OUTPUT_VOLUME_MIN, OUTPUT_VOLUME_MAX, 7, 0));
    _set_bits(0x22, 0x07, 4, vol);
    _set_bits(0x25, 0x07, 4, vol);
}

bool WM8960_Advanced::get_left_mic_output() const { return _get_bit(0x2D, 7); }
void WM8960_Advanced::set_left_mic_output(bool value) { _set_bit(0x2D, 7, value); }

bool WM8960_Advanced::get_right_mic_output() const { return _get_bit(0x2E, 7); }
void WM8960_Advanced::set_right_mic_output(bool value) { _set_bit(0x2E, 7, value); }

bool WM8960_Advanced::get_mic_output() const { return get_left_mic_output() && get_right_mic_output(); }
void WM8960_Advanced::set_mic_output(bool value) { 
    set_left_mic_output(value); 
    set_right_mic_output(value); 
}

float WM8960_Advanced::get_left_mic_output_volume() const {
    return map_range(_get_bits(0x2D, 0x07, 4), 0, 7, OUTPUT_VOLUME_MAX, OUTPUT_VOLUME_MIN);
}

void WM8960_Advanced::set_left_mic_output_volume(float value) {
    _set_bits(0x2D, 0x07, 4, round(map_range(value, OUTPUT_VOLUME_MIN, OUTPUT_VOLUME_MAX, 7, 0)));
}

float WM8960_Advanced::get_right_mic_output_volume() const {
    return map_range(_get_bits(0x2E, 0x07, 4), 0, 7, OUTPUT_VOLUME_MAX, OUTPUT_VOLUME_MIN);
}

void WM8960_Advanced::set_right_mic_output_volume(float value) {
    _set_bits(0x2E, 0x07, 4, round(map_range(value, OUTPUT_VOLUME_MIN, OUTPUT_VOLUME_MAX, 7, 0)));
}

float WM8960_Advanced::get_mic_output_volume() const {
    return std::max(get_left_mic_output_volume(), get_right_mic_output_volume());
}

void WM8960_Advanced::set_mic_output_volume(float value) {
    uint8_t vol = round(map_range(value, OUTPUT_VOLUME_MIN, OUTPUT_VOLUME_MAX, 7, 0));
    _set_bits(0x2D, 0x07, 4, vol);
    _set_bits(0x2E, 0x07, 4, vol);
}

bool WM8960_Advanced::get_mono_output() const { return _get_bit(0x1A, 1); }
void WM8960_Advanced::set_mono_output(bool value) { _set_bit(0x1A, 1, value); }

bool WM8960_Advanced::get_mono_left_mix() const { return _get_bit(0x26, 7); }
void WM8960_Advanced::set_mono_left_mix(bool value) { _set_bit(0x26, 7, value); }

bool WM8960_Advanced::get_mono_right_mix() const { return _get_bit(0x27, 7); }
void WM8960_Advanced::set_mono_right_mix(bool value) { _set_bit(0x27, 7, value); }

bool WM8960_Advanced::get_mono_mix() const { return get_mono_left_mix() && get_mono_right_mix(); }
void WM8960_Advanced::set_mono_mix(bool value) { 
    set_mono_left_mix(value); 
    set_mono_right_mix(value); 
}

bool WM8960_Advanced::get_mono_output_attenuation() const { return _get_bit(0x2A, 6); }
void WM8960_Advanced::set_mono_output_attenuation(bool value) { _set_bit(0x2A, 6, value); }

// Amplifier - Headphones
bool WM8960_Advanced::get_left_headphone() const { return _get_bit(0x1A, 6); }
void WM8960_Advanced::set_left_headphone(bool value) { _set_bit(0x1A, 6, value); }

bool WM8960_Advanced::get_right_headphone() const { return _get_bit(0x1A, 5); }
void WM8960_Advanced::set_right_headphone(bool value) { _set_bit(0x1A, 5, value); }

bool WM8960_Advanced::get_headphone() const { return get_left_headphone() && get_right_headphone(); }
void WM8960_Advanced::set_headphone(bool value) { 
    set_left_headphone(value); 
    set_right_headphone(value); 
}

bool WM8960_Advanced::get_headphone_standby() const { return _get_bit(0x1C, 0); }
void WM8960_Advanced::set_headphone_standby(bool value) { _set_bit(0x1C, 0, value); }

float WM8960_Advanced::get_left_headphone_volume() const {
    uint8_t val = _get_bits(0x02, 0x7F, 0);
    return val < 48 ? AMP_VOLUME_MIN - 1.0f : map_range(val, 48, 127, AMP_VOLUME_MIN, AMP_VOLUME_MAX);
}

void WM8960_Advanced::set_left_headphone_volume(float value) {
    _set_bits(0x02, 0x7F, 0, value >= AMP_VOLUME_MIN ? round(map_range(value, AMP_VOLUME_MIN, AMP_VOLUME_MAX, 48, 127)) : 0);
    _set_bit(0x02, 8, true);
}

float WM8960_Advanced::get_right_headphone_volume() const {
    uint8_t val = _get_bits(0x03, 0x7F, 0);
    return val < 48 ? AMP_VOLUME_MIN - 1.0f : map_range(val, 48, 127, AMP_VOLUME_MIN, AMP_VOLUME_MAX);
}

void WM8960_Advanced::set_right_headphone_volume(float value) {
    _set_bits(0x03, 0x7F, 0, value >= AMP_VOLUME_MIN ? round(map_range(value, AMP_VOLUME_MIN, AMP_VOLUME_MAX, 48, 127)) : 0);
    _set_bit(0x03, 8, true);
}

float WM8960_Advanced::get_headphone_volume() const {
    float left = get_left_headphone_volume();
    float right = get_right_headphone_volume();
    
    if (left < AMP_VOLUME_MIN || right < AMP_VOLUME_MIN) {
        if (left < AMP_VOLUME_MIN && right < AMP_VOLUME_MIN) {
            return AMP_VOLUME_MIN - 1.0f;
        }
        return left < AMP_VOLUME_MIN ? right : left;
    }
    return std::max(left, right);
}

void WM8960_Advanced::set_headphone_volume(float value) {
    uint8_t vol = value >= AMP_VOLUME_MIN ? round(map_range(value, AMP_VOLUME_MIN, AMP_VOLUME_MAX, 48, 127)) : 0;
    _set_bits(0x02, 0x7F, 0, vol);
    _set_bits(0x03, 0x7F, 0, vol);
    _set_bit(0x02, 8, true);
    _set_bit(0x03, 8, true);
}

bool WM8960_Advanced::get_left_headphone_zero_cross() const { return _get_bit(0x02, 7); }
void WM8960_Advanced::set_left_headphone_zero_cross(bool value) { _set_bit(0x02, 7, value); }

bool WM8960_Advanced::get_right_headphone_zero_cross() const { return _get_bit(0x03, 7); }
void WM8960_Advanced::set_right_headphone_zero_cross(bool value) { _set_bit(0x03, 7, value); }

bool WM8960_Advanced::get_headphone_zero_cross() const { 
    return get_left_headphone_zero_cross() && get_right_headphone_zero_cross(); 
}
void WM8960_Advanced::set_headphone_zero_cross(bool value) { 
    set_left_headphone_zero_cross(value); 
    set_right_headphone_zero_cross(value); 
}

// Amplifier - Speakers
bool WM8960_Advanced::get_left_speaker() const { 
    return _get_bit(0x1A, 4) && _get_bit(0x31, 6); 
}

void WM8960_Advanced::set_left_speaker(bool value) { 
    _set_bit(0x1A, 4, value); 
    _set_bit(0x31, 6, value); 
}

bool WM8960_Advanced::get_right_speaker() const { 
    return _get_bit(0x1A, 3) && _get_bit(0x31, 7); 
}

void WM8960_Advanced::set_right_speaker(bool value) { 
    _set_bit(0x1A, 3, value); 
    _set_bit(0x31, 7, value); 
}

bool WM8960_Advanced::get_speaker() const { return get_left_speaker() && get_right_speaker(); }
void WM8960_Advanced::set_speaker(bool value) { 
    set_left_speaker(value); 
    set_right_speaker(value); 
}

float WM8960_Advanced::get_left_speaker_volume() const {
    uint8_t val = _get_bits(0x28, 0x7F, 0);
    return val < 48 ? AMP_VOLUME_MIN - 1.0f : map_range(val, 48, 127, AMP_VOLUME_MIN, AMP_VOLUME_MAX);
}

void WM8960_Advanced::set_left_speaker_volume(float value) {
    _set_bits(0x28, 0x7F, 0, value >= AMP_VOLUME_MIN ? round(map_range(value, AMP_VOLUME_MIN, AMP_VOLUME_MAX, 48, 127)) : 0);
    _set_bit(0x28, 8, true);
}

float WM8960_Advanced::get_right_speaker_volume() const {
    uint8_t val = _get_bits(0x29, 0x7F, 0);
    return val < 48 ? AMP_VOLUME_MIN - 1.0f : map_range(val, 48, 127, AMP_VOLUME_MIN, AMP_VOLUME_MAX);
}

void WM8960_Advanced::set_right_speaker_volume(float value) {
    _set_bits(0x29, 0x7F, 0, value >= AMP_VOLUME_MIN ? round(map_range(value, AMP_VOLUME_MIN, AMP_VOLUME_MAX, 48, 127)) : 0);
    _set_bit(0x29, 8, true);
}

float WM8960_Advanced::get_speaker_volume() const {
    float left = get_left_speaker_volume();
    float right = get_right_speaker_volume();
    
    if (left < AMP_VOLUME_MIN || right < AMP_VOLUME_MIN) {
        if (left < AMP_VOLUME_MIN && right < AMP_VOLUME_MIN) {
            return AMP_VOLUME_MIN - 1.0f;
        }
        return left < AMP_VOLUME_MIN ? right : left;
    }
    return std::max(left, right);
}

void WM8960_Advanced::set_speaker_volume(float value) {
    uint8_t vol = value >= AMP_VOLUME_MIN ? round(map_range(value, AMP_VOLUME_MIN, AMP_VOLUME_MAX, 48, 127)) : 0;
    _set_bits(0x28, 0x7F, 0, vol);
    _set_bits(0x29, 0x7F, 0, vol);
    _set_bit(0x28, 8, true);
    _set_bit(0x29, 8, true);
}

bool WM8960_Advanced::get_left_speaker_zero_cross() const { return _get_bit(0x28, 7); }
void WM8960_Advanced::set_left_speaker_zero_cross(bool value) { _set_bit(0x28, 7, value); }

bool WM8960_Advanced::get_right_speaker_zero_cross() const { return _get_bit(0x29, 7); }
void WM8960_Advanced::set_right_speaker_zero_cross(bool value) { _set_bit(0x29, 7, value); }

bool WM8960_Advanced::get_speaker_zero_cross() const { 
    return get_left_speaker_zero_cross() && get_right_speaker_zero_cross(); 
}
void WM8960_Advanced::set_speaker_zero_cross(bool value) { 
    set_left_speaker_zero_cross(value); 
    set_right_speaker_zero_cross(value); 
}

float WM8960_Advanced::get_speaker_dc_gain() const {
    return _SPEAKER_BOOST_GAIN[_get_bits(0x33, 0x07, 3)];
}

void WM8960_Advanced::set_speaker_dc_gain(float value) {
    _set_bits(0x33, 0x07, 3, _get_speaker_boost_gain(value));
}

float WM8960_Advanced::get_speaker_ac_gain() const {
    return _SPEAKER_BOOST_GAIN[_get_bits(0x33, 0x07, 0)];
}

void WM8960_Advanced::set_speaker_ac_gain(float value) {
    _set_bits(0x33, 0x07, 0, _get_speaker_boost_gain(value));
}

// Digital Audio Interface Control
bool WM8960_Advanced::get_loopback() const { return _get_bit(0x09, 0); }
void WM8960_Advanced::set_loopback(bool value) { _set_bit(0x09, 0, value); }

bool WM8960_Advanced::get_pll() const { return _get_bit(0x1A, 0); }
void WM8960_Advanced::set_pll(bool value) { _set_bit(0x1A, 0, value); }

bool WM8960_Advanced::get_pll_prescale_div2() const { return _get_bit(0x34, 4); }
void WM8960_Advanced::set_pll_prescale_div2(bool value) { _set_bit(0x34, 4, value); }

uint8_t WM8960_Advanced::get_pll_n() const { return _get_bits(0x34, 0x0F, 0); }
void WM8960_Advanced::set_pll_n(uint8_t value) { _set_bits(0x34, 0x0F, 0, value); }

uint32_t WM8960_Advanced::get_pll_k() const {
    return (_get_bits(0x35, 0x3F, 0) << 18) | (_get_bits(0x36, 0x1FF, 0) << 9) | _get_bits(0x37, 0x1FF, 0);
}

void WM8960_Advanced::set_pll_k(uint32_t value) {
    _set_bits(0x35, 0x3F, 0, (value >> 18) & 0x3F);
    _set_bits(0x36, 0x1FF, 0, (value >> 9) & 0x1FF);
    _set_bits(0x37, 0x1FF, 0, value & 0x1FF);
}

bool WM8960_Advanced::get_clock_fractional_mode() const { return _get_bit(0x34, 5); }
void WM8960_Advanced::set_clock_fractional_mode(bool value) { _set_bit(0x34, 5, value); }

bool WM8960_Advanced::get_clock_from_pll() const { return _get_bit(0x04, 0); }
void WM8960_Advanced::set_clock_from_pll(bool value) { _set_bit(0x04, 0, value); }

bool WM8960_Advanced::get_system_clock_div2() const { return _get_bits(0x04, 0x03, 1) == 0x02; }
void WM8960_Advanced::set_system_clock_div2(bool value) { _set_bits(0x04, 0x03, 1, value ? 0x02 : 0x00); }

float WM8960_Advanced::get_adc_clock_divider() const {
    uint8_t idx = _get_bits(0x04, 0x07, 6);
    return idx < _ADCDACDIV.size() ? _ADCDACDIV[idx] : 1.0f;
}

void WM8960_Advanced::set_adc_clock_divider(float value) {
    value = round(value * 2.0f) / 2.0f;
    for (size_t i = 0; i < _ADCDACDIV.size(); i++) {
        if (value == _ADCDACDIV[i]) {
            _set_bits(0x04, 0x07, 6, i);
            return;
        }
    }
}

float WM8960_Advanced::get_dac_clock_divider() const {
    uint8_t idx = _get_bits(0x04, 0x07, 3);
    return idx < _ADCDACDIV.size() ? _ADCDACDIV[idx] : 1.0f;
}

void WM8960_Advanced::set_dac_clock_divider(float value) {
    value = round(value * 2.0f) / 2.0f;
    for (size_t i = 0; i < _ADCDACDIV.size(); i++) {
        if (value == _ADCDACDIV[i]) {
            _set_bits(0x04, 0x07, 3, i);
            return;
        }
    }
}

float WM8960_Advanced::get_base_clock_divider() const {
    uint8_t idx = _get_bits(0x08, 0x0F, 0);
    return idx < _BCLKDIV.size() ? _BCLKDIV[idx] : 1.0f;
}

void WM8960_Advanced::set_base_clock_divider(float value) {
    value = round(value * 2.0f) / 2.0f;
    for (size_t i = 0; i < _BCLKDIV.size(); i++) {
        if (value == _BCLKDIV[i]) {
            _set_bits(0x08, 0x0F, 0, i);
            return;
        }
    }
}

float WM8960_Advanced::get_amp_clock_divider() const {
    uint8_t idx = _get_bits(0x08, 0x07, 6);
    return idx < _DCLKDIV.size() ? _DCLKDIV[idx] : 1.5f;
}

void WM8960_Advanced::set_amp_clock_divider(float value) {
    value = round(value * 2.0f) / 2.0f;
    for (size_t i = 0; i < _DCLKDIV.size(); i++) {
        if (value == _DCLKDIV[i]) {
            _set_bits(0x08, 0x07, 6, i);
            return;
        }
    }
}

bool WM8960_Advanced::get_master_mode() const { return _get_bit(0x07, 6); }
void WM8960_Advanced::set_master_mode(bool value) { _set_bit(0x07, 6, value); }

uint8_t WM8960_Advanced::get_bit_depth() const {
    uint8_t val = _get_bits(0x07, 0x03, 2);
    return val == 3 ? 32 : 16 + 4 * val;
}

void WM8960_Advanced::set_bit_depth(uint8_t value) {
    _set_bits(0x07, 0x03, 2, (std::min(value, (uint8_t)28) - 16) / 4);
}

bool WM8960_Advanced::get_word_select_invert() const { return _get_bit(0x07, 4); }
void WM8960_Advanced::set_word_select_invert(bool value) { _set_bit(0x07, 4, value); }

bool WM8960_Advanced::get_adc_channel_swap() const { return _get_bit(0x07, 8); }
void WM8960_Advanced::set_adc_channel_swap(bool value) { _set_bit(0x07, 8, value); }

bool WM8960_Advanced::get_vref_output() const { return !_get_bit(0x1B, 6); }
void WM8960_Advanced::set_vref_output(bool value) { _set_bit(0x1B, 6, !value); }

float WM8960_Advanced::get_power_supply() const {
    uint8_t val = _get_bits(0x17, 0x03, 6);
    return (std::min(val, (uint8_t)2) - 1) * 0.6f + 2.7f;
}

void WM8960_Advanced::set_power_supply(float value) {
    _set_bits(0x17, 0x03, 6, (uint8_t)((constrain(value, 2.7f, 3.3f) - 2.7f) / 0.6f) * 2 + 1);
}

bool WM8960_Advanced::get_gpio_output() const { return _get_bit(0x09, 6); }
void WM8960_Advanced::set_gpio_output(bool value) { _set_bit(0x09, 6, value); }

uint8_t WM8960_Advanced::get_gpio_output_mode() const { return _get_bits(0x30, 0x07, 4); }
void WM8960_Advanced::set_gpio_output_mode(uint8_t value) { _set_bits(0x30, 0x07, 4, value); }

bool WM8960_Advanced::get_gpio_output_invert() const { return _get_bit(0x30, 7); }
void WM8960_Advanced::set_gpio_output_invert(bool value) { _set_bit(0x30, 7, value); }

float WM8960_Advanced::get_gpio_clock_divider() const {
    uint8_t idx = _get_bits(0x08, 0x07, 6);
    return idx < _OPCLKDIV.size() ? _OPCLKDIV[idx] : 1.0f;
}

void WM8960_Advanced::set_gpio_clock_divider(float value) {
    value = round(value * 2.0f) / 2.0f;
    for (size_t i = 0; i < _OPCLKDIV.size(); i++) {
        if (value == _OPCLKDIV[i]) {
            _set_bits(0x08, 0x07, 6, i);
            return;
        }
    }
}

int WM8960_Advanced::get_sample_rate() const { return _sample_rate; }

void WM8960_Advanced::set_sample_rate(int value) {
    // MCLK = 24 MHz
    set_pll(true);  // Needed for class-d amp clock
    set_clock_fractional_mode(true);
    set_clock_from_pll(true);

    set_pll_prescale_div2(true);
    set_system_clock_div2(true);
    set_base_clock_divider(4.0f);
    set_amp_clock_divider(16.0f);

   if (value == 44100 || value == 48000) {
        // Optimal settings for standard rates
        set_pll_n(8);
        set_pll_k(0x3126E8);
        set_adc_clock_divider(1.0f);
        set_dac_clock_divider(1.0f);
        set_base_clock_divider(4.0f);
    }
    else {
        return; // Invalid sample rate
    }

    _sample_rate = value;
}

// WM8960 implementation
WM8960::WM8960(i2c_inst_t* i2c, int sample_rate, int bit_depth)
    : _codec(i2c), _input(Input::DISABLED), _gain(0.0f) {

    _codec.set_power(false);
    sleep_ms(10);
    
    _codec.set_power(true);
     sleep_ms(10);

    // Digital Interface
    _codec.set_sample_rate(sample_rate);
    _codec.set_bit_depth(bit_depth);

    // ADC
    _codec.set_adc(true);
    _codec.set_input(true);
    _codec.set_mic_boost_gain(0.0f);
    _codec.set_mic_zero_cross(true);

    // DAC
    _codec.set_dac(true);
    _codec.set_dac_output(true);

    // Output
    _codec.set_output(true);
    _codec.set_mono_output(true);
    _codec.set_headphone_zero_cross(true);
    _codec.set_speaker_zero_cross(true);

     // Configure with reduced default gains
    _codec.set_mic_boost_gain(13.0f); // Moderate boost instead of 0
    _codec.set_adc_volume(-12.0f); // Start with lower ADC gain
    _codec.set_dac_volume(0.5f); // Moderate DAC output
    
    // Enable DC filters
    _codec.set_enhance_filter_hpf(true);
    _codec.set_enhance_filter_lpf(true);
}

int WM8960::get_sample_rate() const { return _codec.get_sample_rate(); }
int WM8960::get_bit_depth() const { return _codec.get_bit_depth(); }

uint8_t WM8960::get_input() const { return _input; }

void WM8960::set_input(uint8_t value) {
    bool mic = value & 0b001;

    // Configure microphone amplifier
    _codec.set_mic(mic);
    _codec.set_mic_inverting_input(mic);
    _codec.set_mic_input((value & 0b110) >> 1);
    _codec.set_mic_mute(!mic);
    _codec.set_mic_boost(mic);

    _input = value;

    // Reset gain values
    set_gain(_gain);
}

float WM8960::get_gain() const { return _gain; }

void WM8960::set_gain(float value) {
    // Add logarithmic scaling for more natural gain adjustment
    float log_value = log10f(value * 9.0f + 1.0f); // Convert to logarithmic scale
    
    bool mic = _input & 0b001;
    if (mic) {
        // More gradual mic gain adjustment
        float mic_gain = map_range(log_value, 0.0f, 1.0f, MIC_GAIN_MIN, MIC_GAIN_MAX/2);
        _codec.set_mic_volume(mic_gain);
    } else {
        // Input boost controls with smoother transition
        if (_input & 0b010) {
            float boost = map_range(log_value, 0.0f, 1.0f, BOOST_GAIN_MIN, BOOST_GAIN_MAX/2);
            _codec.set_input2_boost(boost);
        }
        // Similar for input3...
    }
    _gain = constrain(value, 0.0f, 1.0f);
}

float WM8960::get_monitor() const {
    if (!_codec.get_mic_output()) return 0.0f;
    return map_range(
        _codec.get_mic_output_volume(),
        OUTPUT_VOLUME_MIN,
        OUTPUT_VOLUME_MAX,
        0.0f,
        1.0f
    );
}

void WM8960::set_monitor(float value) {
    if (value <= 0.0f) {
        _codec.set_mic_output(false);
    } else if (!_codec.get_mic_output()) {
        _codec.set_mic_output(true);
    }
    _codec.set_mic_output_volume(map_range(
        value, 0.0f, 1.0f, OUTPUT_VOLUME_MIN, OUTPUT_VOLUME_MAX
    ));
}

bool WM8960::get_loopback() const { 
    return _codec.get_master_mode() && _codec.get_gpio_output() && _codec.get_loopback(); 
}

void WM8960::set_loopback(bool value) {
    _codec.set_master_mode(value);
    _codec.set_gpio_output(value);
    _codec.set_loopback(value);
}

float WM8960::get_volume() const {
    if (_codec.get_dac_mute()) return 0.0f;
    return map_range(_codec.get_dac_volume(), DAC_VOLUME_MIN, DAC_VOLUME_MAX, 0.0f, 1.0f);
}

void WM8960::set_volume(float value) {
    if (value <= 0.0f) {
        _codec.set_dac_mute(true);
    } else if (_codec.get_dac_mute()) {
        _codec.set_dac_mute(false);
    }
    _codec.set_dac_volume(map_range(value, 0.0f, 1.0f, DAC_VOLUME_MIN, DAC_VOLUME_MAX));
}

float WM8960::get_headphone() const {
    if (!_codec.get_headphone()) return 0.0f;
    float value = _codec.get_headphone_volume();
    return value < AMP_VOLUME_MIN ? 0.0f : map_range(value, AMP_VOLUME_MIN, AMP_VOLUME_MAX, 0.0f, 1.0f);
}

void WM8960::set_headphone(float value) {
    if (value <= 0.0f) {
        _codec.set_headphone(false);
    } else if (!_codec.get_headphone()) {
        _codec.set_headphone(true);
    }
    _codec.set_headphone_volume(value > 0.0f ? 
        map_range(value, 0.0f, 1.0f, AMP_VOLUME_MIN, AMP_VOLUME_MAX) : 
        AMP_VOLUME_MIN - 1.0f
    );
}

float WM8960::get_speaker() const {
    if (!_codec.get_speaker()) return 0.0f;
    return map_range(_codec.get_speaker_volume(), AMP_VOLUME_MIN, AMP_VOLUME_MAX, 0.0f, 1.0f);
}

void WM8960::set_speaker(float value) {
    if (value <= 0.0f) {
        _codec.set_speaker(false);
    } else if (!_codec.get_speaker()) {
        _codec.set_speaker(true);
    }
    _codec.set_speaker_volume(map_range(value, 0.0f, 1.0f, AMP_VOLUME_MIN, AMP_VOLUME_MAX));
}

float WM8960::get_enhance() const {
    if (!_codec.get_enhance()) return 0.0f;
    return _codec.get_enhance_depth();
}

void WM8960::set_enhance(float value) {
    if (value <= 0.0f) {
        _codec.set_enhance(false);
    } else if (!_codec.get_enhance()) {
        _codec.set_enhance(true);
    }
    _codec.set_enhance_depth(value);
}

bool WM8960::get_alc() const { return _codec.get_alc(); }
void WM8960::set_alc(bool value) { _codec.set_alc(value); }

std::tuple<float, float, float, float> WM8960::get_alc_gain() const {
    return std::make_tuple(
        map_range(_codec.get_alc_target(), ALC_TARGET_MIN, ALC_TARGET_MAX, 0.0f, 1.0f),
        map_range(_codec.get_alc_max_gain(), ALC_MAX_GAIN_MIN, ALC_MAX_GAIN_MAX, 0.0f, 1.0f),
        map_range(_codec.get_alc_min_gain(), ALC_MIN_GAIN_MIN, ALC_MIN_GAIN_MAX, 0.0f, 1.0f),
        _codec.get_noise_gate() ? 
            map_range(_codec.get_noise_gate_threshold(), GATE_THRESHOLD_MIN, GATE_THRESHOLD_MAX, 0.0f, 1.0f) : 
            0.0f
    );
}

void WM8960::set_alc_gain(const std::tuple<float, float, float, float>& value) {
    if (std::tuple_size<std::remove_reference_t<decltype(value)>>::value < 4) return;

    _codec.set_alc_target(map_range(std::get<0>(value), 0.0f, 1.0f, ALC_TARGET_MIN, ALC_TARGET_MAX));
    _codec.set_alc_max_gain(map_range(std::get<1>(value), 0.0f, 1.0f, ALC_MAX_GAIN_MIN, ALC_MAX_GAIN_MAX));
    _codec.set_alc_min_gain(map_range(std::get<2>(value), 0.0f, 1.0f, ALC_MIN_GAIN_MIN, ALC_MIN_GAIN_MAX));

    if (std::get<3>(value) <= 0.0f) {
        _codec.set_noise_gate(false);
    } else if (!_codec.get_noise_gate()) {
        _codec.set_noise_gate(true);
    }
    _codec.set_noise_gate_threshold(map_range(
        std::get<3>(value), 0.0f, 1.0f, GATE_THRESHOLD_MIN, GATE_THRESHOLD_MAX
    ));
}

std::tuple<float, float, float> WM8960::get_alc_time() const {
    return std::make_tuple(
        _codec.get_alc_attack_time(),
        _codec.get_alc_decay_time(),
        _codec.get_alc_hold_time()
    );
}

void WM8960::set_alc_time(const std::tuple<float, float, float>& value) {
    if (std::tuple_size<std::remove_reference_t<decltype(value)>>::value < 3) return;

    _codec.set_alc_attack_time(std::get<0>(value));
    _codec.set_alc_decay_time(std::get<1>(value));
    _codec.set_alc_hold_time(std::get<2>(value));
}

void WM8960::set_sample_rate_on_fly(uint32_t sample_rate) {

    if(_codec.get_sample_rate() == sample_rate) return;
    // Store previous state
    bool was_enabled = _codec.get_power();
    
    // Power down to change clocks safely
    _codec.set_power(false);
    sleep_ms(10);
    
    // Configure new sample rate
    _codec.set_sample_rate(sample_rate);
    
    // Restore power state
    if(was_enabled) {
        _codec.set_power(true);
        sleep_ms(10);
    }
}