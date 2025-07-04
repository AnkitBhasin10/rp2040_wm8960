// #include "sparkfun_wm8960.h"
// #include "wm8960.h"
// #include <pico/stdlib.h>
// #include "hardware/i2c.h"
// #include <stdint.h>
// #include <stdbool.h>
// #include <cmath>

// #define DISABLED 0b000 //disconnect all inputs
// #define MIC1 0b001 //connect Input 1 to mic amplifier in sigle-ended mode
// #define MIC2 0b011 //connect Input 1 and 2 to mic amplifier in differential mode
// #define MIC3 0b101 //connect Input 1 and 3 to mic amplifier in differential mode
// #define LINE2 0b010 //connect input 2 as a line input
// #define LINE3 0b100 //connect input 3 as a line input

// // Default Register Map
// constexpr std::array<uint16_t, 56> _REG_DEFAULTS = {
//     0x0097,  // R0 (0x00)
//     0x0097,  // R1 (0x01)
//     0x0000,  // R2 (0x02)
//     0x0000,  // R3 (0x03)
//     0x0000,  // R4 (0x04)
//     0x0008,  // R5 (0x05)
//     0x0000,  // R6 (0x06)
//     0x000A,  // R7 (0x07)
//     0x01C0,  // R8 (0x08)
//     0x0000,  // R9 (0x09)
//     0x00FF,  // R10 (0x0a)
//     0x00FF,  // R11 (0x0b)
//     0x0000,  // R12 (0x0C) RESERVED
//     0x0000,  // R13 (0x0D) RESERVED
//     0x0000,  // R14 (0x0E) RESERVED
//     0x0000,  // R15 (0x0F) RESERVED
//     0x0000,  // R16 (0x10)
//     0x007B,  // R17 (0x11)
//     0x0100,  // R18 (0x12)
//     0x0032,  // R19 (0x13)
//     0x0000,  // R20 (0x14)
//     0x00C3,  // R21 (0x15)
//     0x00C3,  // R22 (0x16)
//     0x01C0,  // R23 (0x17)
//     0x0000,  // R24 (0x18)
//     0x0000,  // R25 (0x19)
//     0x0000,  // R26 (0x1A)
//     0x0000,  // R27 (0x1B)
//     0x0000,  // R28 (0x1C)
//     0x0000,  // R29 (0x1D)
//     0x0000,  // R30 (0x1E) RESERVED
//     0x0000,  // R31 (0x1F) RESERVED
//     0x0100,  // R32 (0x20)
//     0x0100,  // R33 (0x21)
//     0x0050,  // R34 (0x22)
//     0x0000,  // R35 (0x23) RESERVED
//     0x0000,  // R36 (0x24) RESERVED
//     0x0050,  // R37 (0x25)
//     0x0000,  // R38 (0x26)
//     0x0000,  // R39 (0x27)
//     0x0000,  // R40 (0x28)
//     0x0000,  // R41 (0x29)
//     0x0040,  // R42 (0x2A)
//     0x0000,  // R43 (0x2B)
//     0x0000,  // R44 (0x2C)
//     0x0050,  // R45 (0x2D)
//     0x0050,  // R46 (0x2E)
//     0x0000,  // R47 (0x2F)
//     0x0002,  // R48 (0x30)
//     0x0037,  // R49 (0x31)
//     0x0000,  // R50 (0x32) RESERVED
//     0x0080,  // R51 (0x33)
//     0x0008,  // R52 (0x34)
//     0x0031,  // R53 (0x35)
//     0x0026,  // R54 (0x36)
//     0x00E9   // R55 (0x37)
// };

// WM8960::WM8960() {}

// void WM8960::initializeCodec()
// {
  

//   sleep_ms(100);
//   //reset register to default values
// }

// void WM8960::error_blink(int code) {
//     gpio_init(25);
//     gpio_set_dir(25, GPIO_OUT);
//     while (true) {
//         for (int i = 0; i < code; i++) {
//             gpio_put(25, 1);
//             sleep_ms(200);
//             gpio_put(25, 0);
//             sleep_ms(200);
//         }
//         sleep_ms(1000); // Add a pause between error code blinks
//     }
// }

// bool WM8960::begin(i2c_inst_t *i2cPort)
// {
//   	_i2cPort = i2cPort;
//   	return isConnected();
// }

// //Returns true if I2C device ack's
// bool WM8960::isConnected()
// {
//   	uint8_t codec_detected = 0; 
//     int result = i2c_write_blocking(_i2cPort, _deviceAddress, &codec_detected, 1, true); 

//     return result >= 0;
// }

// // writeRegister(uint8_t reg, uint16_t value)
// // General-purpose write to a register
// // Returns 1 if successful, 0 if something failed (I2C error)
// // The WM8960 has 9 bit registers.
// // To write a register, we must do the following
// // Send 3 bytes
// // Byte0 = device address + read/write bit
// // Control_byte_1 = register-to-write address (7-bits) plus 9th bit of data
// // Control_byte_2 = remaining 8 bits of register data
// bool WM8960::writeRegister(uint8_t reg, uint16_t value)
// {
//     uint8_t control_byte[2];
//     control_byte[0] = (reg << 1) | (value >> 8);
//     control_byte[1] =  value & 0xFF;
    
//     int result = i2c_write_blocking(i2c0, _deviceAddress, control_byte, 2, false);

//     error_blink(result);
    
//     return true;
// }

// // writeRegisterBit
// // Writes a 0 or 1 to the desired bit in the desired register
// bool WM8960::_writeRegisterBit(uint8_t registerAddress, uint8_t bitNumber, bool bitValue)
// {
//     // Get the local copy of the register
//     uint16_t regvalue = _registerLocalCopy[registerAddress]; 

//     if(bitValue == 1) 
//     {
//       regvalue |= (1<<bitNumber); // Set only the bit we want  
//     }
//     else {
//       regvalue &= ~(1<<bitNumber); // Clear only the bit we want  
//     }

//     // Write modified value to device
//     // If successful, update local copy
//     if (WM8960::writeRegister(registerAddress, regvalue)) 
//     {
//         _registerLocalCopy[registerAddress] = regvalue; 
//         return true;
//     }
//   return false;
// }

// // writeRegisterMultiBits
// // This function writes data into the desired bits within the desired register
// // Some settings require more than just flipping a single bit within a register.
// // For these settings use this more advanced register write helper function.
// // 
// // For example, to change the LIN2BOOST setting to +6dB,
// // I need to write a setting of 7 (aka +6dB) to the bits [3:1] in the 
// // WM8960_REG_INPUT_BOOST_MIXER_1 register. Like so...
// // _writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_1, 3, 1, 7);
// bool WM8960::_writeRegisterMultiBits(uint8_t registerAddress, uint8_t settingMsbNum, uint8_t settingLsbNum, uint8_t setting)
// {
//   uint8_t numOfBits = (settingMsbNum - settingLsbNum) + 1;

//   // Get the local copy of the register
//   uint16_t regvalue = _registerLocalCopy[registerAddress]; 

//   for(int i = 0 ; i < numOfBits ; i++)
//   {
//       regvalue &= ~(1 << (settingLsbNum + i)); // Clear bits we care about 
//   }

//   // Shift and set the bits from in incoming desired setting value
//   regvalue |= (setting << settingLsbNum); 

//   // Write modified value to device
//   // If successful, update local copy
//   if (WM8960::writeRegister(registerAddress, regvalue)) 
//   {
//       _registerLocalCopy[registerAddress] = regvalue; 
//       return true;
//   }
//   return false;
// }

// // enableVREF
// // Necessary for all other functions of the CODEC
// // VREF is a single bit we can flip in Register 25 (19h), WM8960_REG_PWR_MGMT_1
// // VREF is bit 6, 0 = power down, 1 = power up
// // Returns 1 if successful, 0 if something failed (I2C error)
// bool WM8960::enableVREF()
// { 
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 6, 1);
// }

// // disableVREF
// // Use this to save power
// // VREF is a single bit we can flip in Register 25 (19h), WM8960_REG_PWR_MGMT_1
// // VREF is bit 6, 0 = power down, 1 = power up
// // Returns 1 if successful, 0 if something failed (I2C error)
// bool WM8960::disableVREF()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 6, 0);
// }

// // reset
// // Use this to reset all registers to their default state
// // Note, this can also be done by cycling power to the device
// // Returns 1 if successful, 0 if something failed (I2C error)
// void WM8960::reset() {
//     // Set reset bit
//     _registers[0x0F] |= (1 << 7);
//     writeRegister(0x0F, _registers[0x0F]);
    
//     //Reset all registers to defaults
//     _reset_registers();
    
//     //Write all registers
//     for (uint8_t i = 0; i < 56; i++) {
//         writeRegister(i, _registers[i]);
//     }
// }

// void WM8960::_reset_registers() {
//     std::copy(_REG_DEFAULTS.begin(), _REG_DEFAULTS.end(), _registers);
// }

// bool WM8960::enableAINL()
// { 
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 5, 1);
// }

// bool WM8960::disableAINL()
// { 
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 5, 0);
// }

// bool WM8960::enableAINR()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 4, 1);
// }

// bool WM8960::disableAINR()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 4, 0);
// }

// bool WM8960::enableLMIC()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 5, 1);
// }

// bool WM8960::disableLMIC()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 5, 0);
// }

// bool WM8960::enableRMIC()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 4, 1);
// }

// bool WM8960::disableRMIC()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 4, 0);
// }

// bool WM8960::enableLMICBOOST()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 5, 1);
// }

// bool WM8960::disableLMICBOOST()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 5, 0);
// }

// bool WM8960::enableRMICBOOST()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 4, 1);
// }

// bool WM8960::disableRMICBOOST()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 4, 0);
// }

// // PGA input signal select
// // Each PGA (left and right) has a switch on its non-inverting input.
// // On PGA_LEFT:
// // 	*You can select between VMID, LINPUT2 or LINPUT3
// // 	*Note, the inverting input of PGA_LEFT is perminantly connected to LINPUT1
// // On PGA_RIGHT:
// //	*You can select between VMIN, RINPUT2 or RINPUT3
// // 	*Note, the inverting input of PGA_RIGHT is perminantly connected to RINPUT1

// // 3 options: WM8960_PGAL_LINPUT2, WM8960_PGAL_LINPUT3, WM8960_PGAL_VMID
// bool WM8960::pgaLeftNonInvSignalSelect(uint8_t signal)
// {
//   // Clear LMP2 and LMP3
//   // Necessary because the previous setting could have either set,
//   // And we don't want to confuse the 
//   // Only 1 input can be selected.

//   // LMP3
//   bool result1 = WM8960::_writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 7, 0); 

//   // LMP2
//   bool result2 = WM8960::_writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 6, 0); 
//   bool result3 = false;

//   if(signal == WM8960_PGAL_LINPUT2)
//   {
//     // LMP2
//     result3 = WM8960::_writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 6, 1); 
//   }
//   else if(signal == WM8960_PGAL_LINPUT3)
//   {
//     // LMP3
//     result3 = WM8960::_writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 7, 1); 
//   }
//   else if(signal == WM8960_PGAL_VMID)
//   {
//     // Don't set any bits. When both LMP2 and LMP3 are cleared, then the signal 
//     // is set to VMID
//   }
//   return (result1 && result2 && result3);
// }

//  // 3 options: WM8960_PGAR_RINPUT2, WM8960_PGAR_RINPUT3, WM8960_PGAR_VMID
// bool WM8960::pgaRightNonInvSignalSelect(uint8_t signal)
// {
//   // Clear RMP2 and RMP3
//   // Necessary because the previous setting could have either set,
//   // And we don't want to confuse the 
//   // Only 1 input can be selected.

//   // RMP3
//   bool result1 = WM8960::_writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 7, 0); 

//   // RMP2
//   bool result2 = WM8960::_writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 6, 0); 
//   bool result3 = false;

//   if(signal == WM8960_PGAR_RINPUT2)
//   {
//     // RMP2
//     result3 = WM8960::_writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 6, 1); 
//   }
//   else if(signal == WM8960_PGAR_RINPUT3)
//   {
//     // RMP3
//     result3 = WM8960::_writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 7, 1); 
//   }
//   else if(signal == WM8960_PGAR_VMID)
//   {
//     // Don't set any bits. When both RMP2 and RMP3 are cleared, then the signal 
//     // is set to VMID
//   }
//   return (result1 && result2 && result3);
// }

// // Connection from each INPUT1 to the inverting input of its PGA
// bool WM8960::connectLMN1()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 8, 1);
// }

// // Disconnect LINPUT1 to inverting input of Left Input PGA
// bool WM8960::disconnectLMN1()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 8, 0);
// }

// // Connect RINPUT1 from inverting input of Right Input PGA
// bool WM8960::connectRMN1()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 8, 1);
// }

// // Disconnect RINPUT1 to inverting input of Right Input PGA
// bool WM8960::disconnectRMN1()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 8, 0);
// }

// // Connections from output of PGAs to downstream "boost mixers".

// // Connect Left Input PGA to Left Input Boost mixer
// bool WM8960::connectLMIC2B()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 3, 1);
// }

// // Disconnect Left Input PGA to Left Input Boost mixer
// bool WM8960::disconnectLMIC2B()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 3, 0);
// }

// // Connect Right Input PGA to Right Input Boost mixer
// bool WM8960::connectRMIC2B()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 3, 1);
// }

// // Disconnect Right Input PGA to Right Input Boost mixer
// bool WM8960::disconnectRMIC2B()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 3, 0);
// }

// // 0-63, (0 = -17.25dB) <<-- 0.75dB steps -->> (63 = +30dB)
// bool WM8960::setLINVOL(uint8_t volume) 
// {
//   if(volume > 63) volume = 63; // Limit incoming values max
//   bool result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_LEFT_INPUT_VOLUME,5,0,volume);
//   bool result2 = WM8960::pgaLeftIPVUSet();
//   return (result1 && result2);
// }

// // setLINVOLDB
// // Sets the volume of the PGA input buffer amp to a specified dB value 
// // passed in as a float argument.
// // Valid dB settings are -17.25 up to +30.00
// // -17.25 = -17.25dB (MIN)
// // ... 0.75dB steps ...
// // 30.00 = +30.00dB  (MAX)
// bool WM8960::setLINVOLDB(float dB)
// {
//   // Create an unsigned integer volume setting variable we can send to 
//   // setLINVOL()
//   uint8_t volume = WM8960::convertDBtoSetting(dB, WM8960_PGA_GAIN_OFFSET, WM8960_PGA_GAIN_STEPSIZE, WM8960_PGA_GAIN_MIN, WM8960_PGA_GAIN_MAX);

//   return WM8960::setLINVOL(volume);
// }

// // 0-63, (0 = -17.25dB) <<-- 0.75dB steps -->> (63 = +30dB)
// bool WM8960::setRINVOL(uint8_t volume) 
// {
//   if(volume > 63) volume = 63; // Limit incoming values max
//   bool result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_RIGHT_INPUT_VOLUME,5,0,volume);
//   bool result2 = WM8960::pgaRightIPVUSet();
//   return (result1 && result2);
// }

// // setRINVOLDB
// // Sets the volume of the PGA input buffer amp to a specified dB value 
// // passed in as a float argument.
// // Valid dB settings are -17.25 up to +30.00
// // -17.25 = -17.25dB (MIN)
// // ... 0.75dB steps ...
// // 30.00 = +30.00dB  (MAX)
// bool WM8960::setRINVOLDB(float dB)
// {
//   // Create an unsigned integer volume setting variable we can send to 
//   // setRINVOL()
//   uint8_t volume = WM8960::convertDBtoSetting(dB, WM8960_PGA_GAIN_OFFSET, WM8960_PGA_GAIN_STEPSIZE, WM8960_PGA_GAIN_MIN, WM8960_PGA_GAIN_MAX);

//   return WM8960::setRINVOL(volume);
// }

// // Zero Cross prevents zipper sounds on volume changes
// // Sets both left and right PGAs
// bool WM8960::enablePgaZeroCross()
// {
//   if (WM8960::_writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 6, 1) == 0) return false;
//   return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 6, 1);
// }

// bool WM8960::disablePgaZeroCross()
// {
//   if (WM8960::_writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 6, 0) == 0) return false;
//   return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 6, 0);
// }

// bool WM8960::enableLINMUTE()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 7, 1);
// }

// bool WM8960::disableLINMUTE()
// {
//   WM8960::_writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 7, 0);
//   return WM8960::_writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 8, 1);
// }

// bool WM8960::enableRINMUTE()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 7, 1);
// }

// bool WM8960::disableRINMUTE()
// {
//   WM8960::_writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 7, 0);
//   return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 8, 1);
// }

// // Causes left and right input PGA volumes to be updated (LINVOL and RINVOL)
// bool WM8960::pgaLeftIPVUSet()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 8, 1);
// }

//  // Causes left and right input PGA volumes to be updated (LINVOL and RINVOL)
// bool WM8960::pgaRightIPVUSet()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 8, 1);
// }


// // Input Boosts

// // 0-3, 0 = +0dB, 1 = +13dB, 2 = +20dB, 3 = +29dB
// bool WM8960::setLMICBOOST(uint8_t boost_gain) 
// {
//   if(boost_gain > 3) boost_gain = 3; // Limit incoming values max
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_ADCL_SIGNAL_PATH,5,4,boost_gain);
// }

// // 0-3, 0 = +0dB, 1 = +13dB, 2 = +20dB, 3 = +29dB
// bool WM8960::setRMICBOOST(uint8_t boost_gain) 
// {
//   if(boost_gain > 3) boost_gain = 3; // Limit incoming values max
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_ADCR_SIGNAL_PATH,5,4,boost_gain);
// }

// // 0-7, 0 = Mute, 1 = -12dB ... 3dB steps ... 7 = +6dB
// bool WM8960::setLIN3BOOST(uint8_t boost_gain) 
// {
//   if(boost_gain > 7) boost_gain = 7; // Limit incoming values max
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_1,6,4,boost_gain);
// }

// // 0-7, 0 = Mute, 1 = -12dB ... 3dB steps ... 7 = +6dB
// bool WM8960::setLIN2BOOST(uint8_t boost_gain) 
// {
//   if(boost_gain > 7) boost_gain = 7; // Limit incoming values max
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_1,3,1,boost_gain);
// }

// // 0-7, 0 = Mute, 1 = -12dB ... 3dB steps ... 7 = +6dB
// bool WM8960::setRIN3BOOST(uint8_t boost_gain) 
// {
//   if(boost_gain > 7) boost_gain = 7; // Limit incoming values max
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_2,6,4,boost_gain);
// }

// // 0-7, 0 = Mute, 1 = -12dB ... 3dB steps ... 7 = +6dB	
// bool WM8960::setRIN2BOOST(uint8_t boost_gain) 
// {
//   if(boost_gain > 7) boost_gain = 7; // Limit incoming values max
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_2,3,1,boost_gain);
// }

// // Mic Bias control
// bool WM8960::enableMicBias()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 1, 1);
// }

// bool WM8960::disableMicBias()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 1, 0);
// }

// // WM8960_MIC_BIAS_VOLTAGE_0_9_AVDD (0.9*AVDD) 
// // or WM8960_MIC_BIAS_VOLTAGE_0_65_AVDD (0.65*AVDD)
// bool WM8960::setMicBiasVoltage(bool voltage)
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_ADDITIONAL_CONTROL_4, 0, voltage);
// }

// /////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////// ADC
// /////////////////////////////////////////////////////////

// bool WM8960::enableAdcLeft()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 3, 1);
// }

// bool WM8960::disableAdcLeft()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 3, 0);
// }

// bool WM8960::enableAdcRight()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 2, 1);
// }

// bool WM8960::disableAdcRight()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_1, 2, 0);
// }

// // ADC digital volume
// // Note, also needs to handle control of the ADCVU bits (volume update).
// // Valid inputs are 0-255
// // 0 = mute
// // 1 = -97dB
// // ... 0.5dB steps up to
// // 195 = +0dB
// // 255 = +30dB

// bool WM8960::setAdcLeftDigitalVolume(uint8_t volume)
// {
//   bool result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_LEFT_ADC_VOLUME,7,0,volume);
//   bool result2 = WM8960::adcLeftADCVUSet();
//   return (result1 && result2);
// }
// bool WM8960::setAdcRightDigitalVolume(uint8_t volume)
// {
//   bool result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_RIGHT_ADC_VOLUME,7,0,volume);
//   bool result2 = WM8960::adcRightADCVUSet();
//   return (result1 && result2);
// }

// // Causes left and right input adc digital volumes to be updated
// bool WM8960::adcLeftADCVUSet()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_LEFT_ADC_VOLUME, 8, 1);
// }

// // Causes left and right input adc digital volumes to be updated
// bool WM8960::adcRightADCVUSet()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_ADC_VOLUME, 8, 1);
// }

// // ADC digital volume DB
// // Sets the volume of the ADC to a specified dB value passed in as a float 
// // argument.
// // Valid dB settings are -97.00 up to +30.0 (0.5dB steps)
// // -97.50 (or lower) = MUTE
// // -97.00 = -97.00dB (MIN)
// // ... 0.5dB steps ...
// // 30.00 = +30.00dB  (MAX)

// bool WM8960::setAdcLeftDigitalVolumeDB(float dB)
// {
//   // Create an unsigned integer volume setting variable we can send to 
//   // setAdcLeftDigitalVolume()
//   uint8_t volume = WM8960::convertDBtoSetting(dB, WM8960_ADC_GAIN_OFFSET, WM8960_ADC_GAIN_STEPSIZE, WM8960_ADC_GAIN_MIN, WM8960_ADC_GAIN_MAX);

//   return WM8960::setAdcLeftDigitalVolume(volume);
// }
// bool WM8960::setAdcRightDigitalVolumeDB(float dB)
// {
//   // Create an unsigned integer volume setting variable we can send to 
//   // setAdcRightDigitalVolume()
//   uint8_t volume = WM8960::convertDBtoSetting(dB, WM8960_ADC_GAIN_OFFSET, WM8960_ADC_GAIN_STEPSIZE, WM8960_ADC_GAIN_MIN, WM8960_ADC_GAIN_MAX);

//   return WM8960::setAdcRightDigitalVolume(volume);
// }

// /////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////// ALC
// /////////////////////////////////////////////////////////

// // Automatic Level Control
// // Note that when the ALC function is enabled, the settings of registers 0 and 
// // 1 (LINVOL, IPVU, LIZC, LINMUTE, RINVOL, RIZC and RINMUTE) are ignored.
// bool WM8960::enableAlc(uint8_t mode)
// {
//   bool bit8 = (mode>>1);
//   bool bit7 = (mode & 0b00000001);
//   if (WM8960::_writeRegisterBit(WM8960_REG_ALC1, 8, bit8) == 0) return false;
//   return WM8960::_writeRegisterBit(WM8960_REG_ALC1, 7, bit7);
// }

//  // Also sets alc sample rate to match global sample rate.
// bool WM8960::disableAlc()
// {
//   if (WM8960::_writeRegisterBit(WM8960_REG_ALC1, 8, 0) == 0) return false;
//   return WM8960::_writeRegisterBit(WM8960_REG_ALC1, 7, 0);
// }

// // Valid inputs are 0-15, 0 = -22.5dB FS, ... 1.5dB steps ... , 15 = -1.5dB FS
// bool WM8960::setAlcTarget(uint8_t target) 
// {
//   if(target > 15) target = 15; // Limit incoming values max
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_ALC1,3,0,target);
// }

// // Valid inputs are 0-10, 0 = 24ms, 1 = 48ms, ... 10 = 24.58seconds
// bool WM8960::setAlcDecay(uint8_t decay) 
// {
//   if(decay > 10) decay = 10; // Limit incoming values max
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_ALC3,7,4,decay);
// }

// // Valid inputs are 0-10, 0 = 6ms, 1 = 12ms, 2 = 24ms, ... 10 = 6.14seconds
// bool WM8960::setAlcAttack(uint8_t attack) 
// {
//   if(attack > 10) attack = 10; // Limit incoming values max
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_ALC3,3,0,attack);
// }

// // Valid inputs are 0-7, 0 = -12dB, ... 7 = +30dB
// bool WM8960::setAlcMaxGain(uint8_t maxGain) 
// {
//   if(maxGain > 7) maxGain = 7; // Limit incoming values max
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_ALC1,6,4,maxGain);
// }

// // Valid inputs are 0-7, 0 = -17.25dB, ... 7 = +24.75dB
// bool WM8960::setAlcMinGain(uint8_t minGain) 
// {
//   if(minGain > 7) minGain = 7; // Limit incoming values max
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_ALC2,6,4,minGain);
// }

// // Valid inputs are 0-15, 0 = 0ms, ... 15 = 43.691s
// bool WM8960::setAlcHold(uint8_t hold) 
// {
//   if(hold > 15) hold = 15; // Limit incoming values max
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_ALC2,3,0,hold);
// }

// // Peak Limiter
// bool WM8960::enablePeakLimiter()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_ALC3, 8, 1);
// }

// bool WM8960::disablePeakLimiter()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_ALC3, 8, 0);
// }

// // Noise Gate
// bool WM8960::enableNoiseGate()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_NOISE_GATE, 0, 1);
// }

// bool WM8960::disableNoiseGate()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_NOISE_GATE, 0, 0);
// }

// // 0-31, 0 = -76.5dBfs, 31 = -30dBfs
// bool WM8960::setNoiseGateThreshold(uint8_t threshold) 
// {
//   return true;
// }

// /////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////// DAC
// /////////////////////////////////////////////////////////

// // Enable/disble each channel
// bool WM8960::enableDacLeft()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 8, 1);
// }

// bool WM8960::disableDacLeft()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 8, 0);
// }

// bool WM8960::enableDacRight()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 7, 1);
// }

// bool WM8960::disableDacRight()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 7, 0);
// }

// // DAC digital volume
// // Valid inputs are 0-255
// // 0 = mute
// // 1 = -127dB
// // ... 0.5dB steps up to
// // 255 = 0dB
// bool WM8960::setDacLeftDigitalVolume(uint8_t volume)
// {
//   bool result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_LEFT_DAC_VOLUME,7,0,volume);
//   bool result2 = WM8960::dacLeftDACVUSet();
//   return (result1 && result2);
// }

// bool WM8960::setDacRightDigitalVolume(uint8_t volume)
// {
//   bool result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_RIGHT_DAC_VOLUME,7,0,volume);
//   bool result2 = WM8960::dacRightDACVUSet();
//   return (result1 && result2);
// }

// // Causes left and right input dac digital volumes to be updated
// bool WM8960::dacLeftDACVUSet()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_LEFT_DAC_VOLUME, 8, 1);
// }

//  // Causes left and right input dac digital volumes to be updated
// bool WM8960::dacRightDACVUSet()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_DAC_VOLUME, 8, 1);
// }	

// // DAC digital volume DB
// // Sets the volume of the DAC to a specified dB value passed in as a float 
// // argument.
// // Valid dB settings are -97.00 up to +30.0 (0.5dB steps)
// // -97.50 (or lower) = MUTE
// // -97.00 = -97.00dB (MIN)
// // ... 0.5dB steps ...
// // 30.00 = +30.00dB  (MAX)

// bool WM8960::setDacLeftDigitalVolumeDB(float dB)
// {
//   // Create an unsigned integer volume setting variable we can send to 
//   // setDacLeftDigitalVolume()
//   uint8_t volume = WM8960::convertDBtoSetting(dB, WM8960_DAC_GAIN_OFFSET, WM8960_DAC_GAIN_STEPSIZE, WM8960_DAC_GAIN_MIN, WM8960_DAC_GAIN_MAX);

//   return WM8960::setDacLeftDigitalVolume(volume);
// }

// bool WM8960::setDacRightDigitalVolumeDB(float dB)
// {
//   // Create an unsigned integer volume setting variable we can send to 
//   // setDacRightDigitalVolume()
//   uint8_t volume = WM8960::convertDBtoSetting(dB, WM8960_DAC_GAIN_OFFSET, WM8960_DAC_GAIN_STEPSIZE, WM8960_DAC_GAIN_MIN, WM8960_DAC_GAIN_MAX);

//   return WM8960::setDacRightDigitalVolume(volume);
// }

// // DAC mute
// bool WM8960::enableDacMute()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_ADC_DAC_CTRL_1, 3, 1);
// }

// bool WM8960::disableDacMute()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_ADC_DAC_CTRL_1, 3, 0);
// }

// // 3D Stereo Enhancement
// // 3D enable/disable
// bool WM8960::enable3d()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_3D_CONTROL, 0, 1);
// }

// bool WM8960::disable3d()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_3D_CONTROL, 0, 0);
// }

// bool WM8960::set3dDepth(uint8_t depth) // 0 = 0%, 15 = 100%
// {
//   if(depth > 15) depth = 15; // Limit incoming values max
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_3D_CONTROL,4,1,depth);
// }

// // DAC output -6dB attentuation enable/disable
// bool WM8960::enableDac6dbAttenuation()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_ADC_DAC_CTRL_1, 7, 1);
// }

// bool WM8960::disableDac6dbAttentuation()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_ADC_DAC_CTRL_1, 7, 0);
// }

// /////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////// OUTPUT mixers
// /////////////////////////////////////////////////////////

// // What's connected to what? Oh so many options...
// // LOMIX	Left Output Mixer
// // ROMIX	Right Output Mixer
// // OUT3MIX		Mono Output Mixer

// // Enable/disable left and right output mixers
// bool WM8960::enableLOMIX()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 3, 1);
// }

// bool WM8960::disableLOMIX()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 3, 0);
// }

// bool WM8960::enableROMIX()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 2, 1);
// }

// bool WM8960::disableROMIX()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_3, 2, 0);
// }

// bool WM8960::enableOUT3MIX()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 1, 1);
// }

// bool WM8960::disableOUT3MIX()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 1, 0);
// }

// // Enable/disable audio path connections/vols to/from output mixers
// // See datasheet page 35 for a nice image of all the connections.
// bool WM8960::enableLI2LO()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_LEFT_OUT_MIX_1, 7, 1);
// }

// bool WM8960::disableLI2LO()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_LEFT_OUT_MIX_1, 7, 0);
// }

// // Valid inputs are 0-7. 0 = 0dB ...3dB steps... 7 = -21dB
// bool WM8960::setLI2LOVOL(uint8_t volume) 
// {
//   if(volume > 7) volume = 7; // Limit incoming values max
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_LEFT_OUT_MIX_1,6,4,volume);
// }

// bool WM8960::enableLB2LO()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_BYPASS_1, 7, 1);
// }

// bool WM8960::disableLB2LO()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_BYPASS_1, 7, 0);
// }

// // Valid inputs are 0-7. 0 = 0dB ...3dB steps... 7 = -21dB
// bool WM8960::setLB2LOVOL(uint8_t volume) 
// {
//   if(volume > 7) volume = 7; // Limit incoming values max
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_BYPASS_1,6,4,volume);
// }

// bool WM8960::enableLD2LO()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_LEFT_OUT_MIX_1, 8, 1);
// }

// bool WM8960::disableLD2LO()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_LEFT_OUT_MIX_1, 8, 0);
// }

// bool WM8960::enableRI2RO()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_OUT_MIX_2, 7, 1);
// }

// bool WM8960::disableRI2RO()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_OUT_MIX_2, 7, 0);
// }

// // Valid inputs are 0-7. 0 = 0dB ...3dB steps... 7 = -21dB
// bool WM8960::setRI2ROVOL(uint8_t volume) 
// {
//   if(volume > 7) volume = 7; // Limit incoming values max
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_RIGHT_OUT_MIX_2,6,4,volume);
// }

// bool WM8960::enableRB2RO()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_BYPASS_2, 7, 1);
// }

// bool WM8960::disableRB2RO()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_BYPASS_2, 7, 0);
// }

// // Valid inputs are 0-7. 0 = 0dB ...3dB steps... 7 = -21dB
// bool WM8960::setRB2ROVOL(uint8_t volume) 
// {
//   if(volume > 7) volume = 7; // Limit incoming values max
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_BYPASS_2,6,4,volume);
// }

// bool WM8960::enableRD2RO()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_OUT_MIX_2, 8, 1);
// }

// bool WM8960::disableRD2RO()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_RIGHT_OUT_MIX_2, 8, 0);
// }

// // Mono Output mixer. 
// // Note, for capless HPs, we'll want this to output a buffered VMID.
// // To do this, we need to disable both of these connections.
// bool WM8960::enableLI2MO()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_MONO_OUT_MIX_1, 7, 1);
// }

// bool WM8960::disableLI2MO()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_MONO_OUT_MIX_1, 7, 0);
// }

// bool WM8960::enableRI2MO()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_MONO_OUT_MIX_2, 7, 1);
// }

// bool WM8960::disableRI2MO()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_MONO_OUT_MIX_2, 7, 0);
// }

// // Enables VMID in the WM8960_REG_PWR_MGMT_2 register, and set's it to 
// // playback/record settings of 2*50Kohm.
// // Note, this function is only hear for backwards compatibility with the
// // original releases of this library. It is recommended to use the
// // setVMID() function instead.
// bool WM8960::enableVMID()
// {
//   return WM8960::setVMID(WM8960_VMIDSEL_2X50KOHM);
// }

// bool WM8960::disableVMID()
// {
//   return WM8960::setVMID(WM8960_VMIDSEL_DISABLED);
// }

// // setVMID
// // Sets the VMID signal to one of three possible settings.
// // 4 options:
// // WM8960_VMIDSEL_DISABLED
// // WM8960_VMIDSEL_2X50KOHM (playback / record)
// // WM8960_VMIDSEL_2X250KOHM (for low power / standby)
// // WM8960_VMIDSEL_2X5KOHM (for fast start-up)
// bool WM8960::setVMID(uint8_t setting)
// {
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_PWR_MGMT_1, 8, 7, setting);
// }

// /////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////// Headphones
// /////////////////////////////////////////////////////////

// // Enable and disable headphones (mute)
// bool WM8960::enableHeadphones()
// {
//   return (WM8960::enableRightHeadphone() & WM8960::enableLeftHeadphone());
// }

// bool WM8960::disableHeadphones()
// {
//   return (WM8960::disableRightHeadphone() & WM8960::disableLeftHeadphone());
// }

// bool WM8960::enableRightHeadphone()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 5, 1);
// }

// bool WM8960::disableRightHeadphone()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 5, 0);
// }

// bool WM8960::enableLeftHeadphone()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 6, 1);
// }

// bool WM8960::disableLeftHeadphone()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 6, 0);
// }

// bool WM8960::enableHeadphoneStandby()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_ANTI_POP_1, 0, 1);
// }

// bool WM8960::disableHeadphoneStandby()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_ANTI_POP_1, 0, 0);
// }

// // SetHeadphoneVolume
// // Sets the volume for both left and right headphone outputs
// // 
// // Although you can control each headphone output independently, here we are
// // Going to assume you want both left and right to do the same thing.
// // 
// // Valid inputs: 47-127. 0-47 = mute, 48 = -73dB ... 1dB steps ... 127 = +6dB
// bool WM8960::setHeadphoneVolume(uint8_t volume) 
// {		
//   // Updates both left and right channels
// 	// Handles the OUT1VU (volume update) bit control, so that it happens at the 
//   // same time on both channels. Note, we must also make sure that the outputs 
//   // are enabled in the WM8960_REG_PWR_MGMT_2 [6:5]
//   // Grab local copy of register
//   // Modify the bits we need to
//   // Write register in device, including the volume update bit write
//   // If successful, save locally.

//   // Limit inputs
//   if (volume > 127) volume = 127;

//   // LEFT
//     bool result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_LOUT1_VOLUME,6,0,volume);
//   // RIGHT
//     bool result2 = WM8960::_writeRegisterMultiBits(WM8960_REG_ROUT1_VOLUME,6,0,volume);
//   // UPDATES

//   // Updated left channel
//     bool result3 = WM8960::_writeRegisterBit(WM8960_REG_LOUT1_VOLUME, 8, 1); 

//   // Updated right channel
//     bool result4 = WM8960::_writeRegisterBit(WM8960_REG_ROUT1_VOLUME, 8, 1); 

//     if (result1 && result2 && result3 && result4) // If all writes ACK'd
//     {
//         return true;
//     }
//   return false; 
// }

// bool WM8960::enableHeadphoneJackDetect() {
//   bool result1 = WM8960::_writeRegisterBit(WM8960_REG_ADDITIONAL_CONTROL_2, 6, 1);
//   bool result2 = WM8960::_writeRegisterBit(WM8960_REG_ADDITIONAL_CONTROL_1, 0, 1); // TOEN - slow clock enable
//   return result1 && result2;
// }

// bool WM8960::disableHeadphoneJackDetect() {
//   bool result1 = WM8960::_writeRegisterBit(WM8960_REG_ADDITIONAL_CONTROL_2, 6, 0);
//   bool result2 = WM8960::_writeRegisterBit(WM8960_REG_ADDITIONAL_CONTROL_1, 0, 0); // TOEN - slow clock disable
//   return result1 && result2;
// }

// // SetHeadphoneJackDetectInput
// // Sets the input pin for jack insertion detection
// //
// // Valid inputs: WM8960_JACKDETECT_LINPUT3, WM8960_JACKDETECT_RINPUT3, WM8960_JACKDETECT_GPIO1
// bool WM8960::setHeadphoneJackDetectInput(uint8_t setting) {
//   bool result1 = false;
//   bool result2 = false;
//   switch (setting) {
//     case WM8960_JACKDETECT_LINPUT3:
//       result1 = WM8960::_writeRegisterBit(WM8960_REG_ADDITIONAL_CONTROL_4, 3, 1);
//       result2 = WM8960::_writeRegisterBit(WM8960_REG_ADDITIONAL_CONTROL_4, 2, 0); // JD2
//       break;
//     case WM8960_JACKDETECT_RINPUT3:
//       result1 = WM8960::_writeRegisterBit(WM8960_REG_ADDITIONAL_CONTROL_4, 3, 1);
//       result2 = WM8960::_writeRegisterBit(WM8960_REG_ADDITIONAL_CONTROL_4, 2, 1); // JD3
//     default:
//       result1 = WM8960::_writeRegisterBit(WM8960_REG_ADDITIONAL_CONTROL_4, 3, 0); // GPIO1
//       result2 = WM8960::_writeRegisterBit(WM8960_REG_ADDITIONAL_CONTROL_4, 2, 0); // defaults to 0
//       break;
//   }
//   return result1 && result2;
// }

// // Set headphone volume dB
// // Sets the volume of the headphone output buffer amp to a specified dB value 
// // passed in as a float argument.
// // Valid dB settings are -74.0 up to +6.0
// // Note, we are accepting float arguments here, in order to keep it consistent
// // with other volume setting functions in this library that can do partial dB
// // values (such as the PGA, ADC and DAC gains).
// // -74 (or lower) = MUTE
// // -73 = -73dB (MIN)
// // ... 1dB steps ...
// // 0 = 0dB
// // ... 1dB steps ...
// // 6 = +6dB  (MAX)
// bool WM8960::setHeadphoneVolumeDB(float dB)
// {
//   // Create an unsigned integer volume setting variable we can send to 
//   // setHeadphoneVolume()
//   uint8_t volume = WM8960::convertDBtoSetting(dB, WM8960_HP_GAIN_OFFSET, WM8960_HP_GAIN_STEPSIZE, WM8960_HP_GAIN_MIN, WM8960_HP_GAIN_MAX);

//   return WM8960::setHeadphoneVolume(volume);
// }

// // Zero Cross prevents zipper sounds on volume changes
// // Sets both left and right Headphone outputs
// bool WM8960::enableHeadphoneZeroCross()
// {
//   // Left
//   bool result1 = WM8960::_writeRegisterBit(WM8960_REG_LOUT1_VOLUME, 7, 1); 

//   // Right
//   bool result2 = WM8960::_writeRegisterBit(WM8960_REG_ROUT1_VOLUME, 7, 1); 
//   return (result1 & result2);
// }

// bool WM8960::disableHeadphoneZeroCross()
// {
//   // Left
//   bool result1 = WM8960::_writeRegisterBit(WM8960_REG_LOUT1_VOLUME, 7, 0); 

//   // Right
//   bool result2 = WM8960::_writeRegisterBit(WM8960_REG_ROUT1_VOLUME, 7, 0); 
//   return (result1 & result2);
// }

// /////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////// Speakers
// /////////////////////////////////////////////////////////

// // Enable and disable speakers (mute)
// bool WM8960::enableSpeakers()
// {
//   return (WM8960::enableRightSpeaker() & WM8960::enableLeftSpeaker());
// }

// bool WM8960::disableSpeakers()
// {
//   return (WM8960::disableRightHeadphone() & WM8960::disableLeftHeadphone());
// }

// bool WM8960::enableRightSpeaker()
// {
//   // SPK_OP_EN
//   bool result1 = WM8960::_writeRegisterBit(WM8960_REG_CLASS_D_CONTROL_1, 7, 1); 

//   // SPKR
//   bool result2 = WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 3, 1); 
//   return (result1 & result2);
// }

// bool WM8960::disableRightSpeaker()
// {
//   // SPK_OP_EN
//   bool result1 = WM8960::_writeRegisterBit(WM8960_REG_CLASS_D_CONTROL_1, 7, 0); 

//   // SPKR
//   bool result2 = WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 3, 0); 
//   return (result1 & result2);
// }

// bool WM8960::enableLeftSpeaker()
// {
//   // SPK_OP_EN
//   bool result1 = WM8960::_writeRegisterBit(WM8960_REG_CLASS_D_CONTROL_1, 6, 1); 

//   // SPKL
//   bool result2 = WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 4, 1); 
//   return (result1 & result2);
// }

// bool WM8960::disableLeftSpeaker()
// {
//   // SPK_OP_EN
//   bool result1 = WM8960::_writeRegisterBit(WM8960_REG_CLASS_D_CONTROL_1, 6, 0); 

//   // SPKL
//   bool result2 = WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 4, 0); 
//   return (result1 & result2);
// }

// // SetSpeakerVolume
// // Sets to volume for both left and right speaker outputs
// // 
// // Although you can control each Speaker output independently, here we are
// // Going to assume you want both left and right to do the same thing.
// // 
// // Valid inputs are 47-127. 0-47 = mute, 48 = -73dB, ... 1dB steps ... , 127 = +6dB
// bool WM8960::setSpeakerVolume(uint8_t volume) 
// {		
//   // Updates both left and right channels
// 	// Handles the SPKVU (volume update) bit control, so that it happens at the 
//   // same time on both channels. Note, we must also make sure that the outputs 
//   // are enabled in the WM8960_REG_PWR_MGMT_2 [4:3], and the class D control 
//   // reg WM8960_REG_CLASS_D_CONTROL_1 [7:6]

//   // Limit inputs
//   if (volume > 127) volume = 127;

//   // LEFT
//   bool result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_LOUT2_VOLUME,6,0,volume);

//   // RIGHT
//   bool result2 = WM8960::_writeRegisterMultiBits(WM8960_REG_ROUT2_VOLUME,6,0,volume);

//   // SPKVU

//   // Updated left channel
//   bool result3 = WM8960::_writeRegisterBit(WM8960_REG_LOUT2_VOLUME, 8, 1); 

//   // Updated right channel
//   bool result4 = WM8960::_writeRegisterBit(WM8960_REG_ROUT2_VOLUME, 8, 1); 

//   if (result1 && result2 && result3 && result4) // If all writes ACK'd
//     {
//         return true;
//     }
//   return false;
// }

// // Set speaker volume dB
// // Sets the volume of the class-d speaker output amp to a specified dB value 
// // passed in as a float argument.
// // Valid dB settings are -74.0 up to +6.0
// // Note, we are accepting float arguments here, in order to keep it consistent
// // with other volume setting functions in this library that can do partial dB
// // values (such as the PGA, ADC and DAC gains).
// // -74 (or lower) = MUTE
// // -73 = -73dB (MIN)
// // ... 1dB steps ...
// // 0 = 0dB
// // ... 1dB steps ...
// // 6 = +6dB  (MAX)
// bool WM8960::setSpeakerVolumeDB(float dB)
// {
//   // Create an unsigned integer volume setting variable we can send to 
//   // setSpeakerVolume()
//   uint8_t volume = WM8960::convertDBtoSetting(dB, WM8960_SPEAKER_GAIN_OFFSET, WM8960_SPEAKER_GAIN_STEPSIZE, WM8960_SPEAKER_GAIN_MIN, WM8960_SPEAKER_GAIN_MAX);

//   return WM8960::setSpeakerVolume(volume);
// }

// // Zero Cross prevents zipper sounds on volume changes
// // Sets both left and right Speaker outputs
// bool WM8960::enableSpeakerZeroCross()
// {
//   // Left
//   bool result1 = WM8960::_writeRegisterBit(WM8960_REG_LOUT2_VOLUME, 7, 1); 

//   // Right
//   bool result2 = WM8960::_writeRegisterBit(WM8960_REG_ROUT2_VOLUME, 7, 1); 
//   return (result1 & result2);
// }

// bool WM8960::disableSpeakerZeroCross()
// {
//   // Left
//   bool result1 = WM8960::_writeRegisterBit(WM8960_REG_LOUT2_VOLUME, 7, 0); 

//   // Right
//   bool result2 = WM8960::_writeRegisterBit(WM8960_REG_ROUT2_VOLUME, 7, 0); 
//   return (result1 & result2);
// }

// // SetSpeakerDcGain
// // DC and AC gain - allows signal to be higher than the DACs swing
// // (use only if your SPKVDD is high enough to handle a larger signal)
// // Valid inputs are 0-5
// // 0 = +0dB (1.0x boost) ... up to ... 5 = +5.1dB (1.8x boost)
// bool WM8960::setSpeakerDcGain(uint8_t gain)
// {
//   if(gain > 5) gain = 5; // Limit incoming values max
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_CLASS_D_CONTROL_3,5,3,gain);
// }

// // SetSpeakerAcGain
// // DC and AC gain - allows signal to be higher than the DACs swing
// // (use only if your SPKVDD is high enough to handle a larger signal)
// // Valid inputs are 0-5
// // 0 = +0dB (1.0x boost) ... up to ... 5 = +5.1dB (1.8x boost)
// bool WM8960::setSpeakerAcGain(uint8_t gain)
// {
//   if(gain > 5) gain = 5; // Limit incoming values max
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_CLASS_D_CONTROL_3,2,0,gain);
// }

// //////////////////////////////////////////////
// ////////////////////////////////////////////// Digital audio interface control
// //////////////////////////////////////////////

// // Defaults to I2S, peripheral-mode, 24-bit word length

// // Loopback
// // When enabled, the output data from the ADC audio interface is fed directly 
// // into the DAC data input.
// bool WM8960::enableLoopBack()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_2, 0, 1);
// }

// bool WM8960::disableLoopBack()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_2, 0, 0);
// }

// /////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////// Clock controls
// /////////////////////////////////////////////////////////

// // Getting the Frequency of SampleRate as we wish
// // Our MCLK (an external clock on the SFE breakout board) is 24.0MHz.
// // According to table 40 (DS pg 58), we want SYSCLK to be 11.2896 for a SR of 
// // 44.1KHz. To get that Desired Output (SYSCLK), we need the following settings 
// // on the PLL stuff, as found on table 45 (ds pg 61):
// // PRESCALE DIVIDE (PLLPRESCALE): 2
// // POSTSCALE DVIDE (SYSCLKDIV[1:0]): 2
// // FIXED POST-DIVIDE: 4
// // R: 7.5264 
// // N: 7h
// // K: 86C226h

// // Example at bottom of table 46, shows that we should be in fractional mode 
// // for a 44.1KHz.

// // In terms of registers, this is what we want for 44.1KHz
// // PLLEN=1			(PLL enable)
// // PLLPRESCALE=1	(divide by 2) *This get's us from MCLK (24MHz) down to 12MHZ 
// // for F2.
// // PLLN=7h			(PLL N value) *this is "int R"
// // PLLK=86C226h		(PLL K value) *this is int ( 2^24 * (R- intR)) 
// // SDM=1			(Fractional mode)
// // CLKSEL=1			(PLL select) 
// // MS=0				(Peripheral mode)
// // WL=00			(16 bits)
// // SYSCLKDIV=2		(Divide by 2)
// // ADCDIV=000		(Divide by 1) = 44.1kHz
// // DACDIV=000		(Divide by 1) = 44.1kHz
// // BCLKDIV=0100		(Divide by 4) = 64fs
// // DCLKDIV=111		(Divide by 16) = 705.6kHz

// // And now for the functions that will set these registers...
// bool WM8960::enablePLL()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 0, 1);
// }

// bool WM8960::disablePLL()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PWR_MGMT_2, 0, 0);
// }

// bool WM8960::setPLLPRESCALE(bool div)
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PLL_N, 4, div);
// }

// bool WM8960::setPLLN(uint8_t n)
// {
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_PLL_N,3,0,n); 
// }

// // Send each nibble of 24-bit value for value K
// bool WM8960::setPLLK(uint8_t one, uint8_t two, uint8_t three) 
// {
//   bool result1 = WM8960::_writeRegisterMultiBits(WM8960_REG_PLL_K_1,5,0,one); 
//   bool result2 = WM8960::_writeRegisterMultiBits(WM8960_REG_PLL_K_2,8,0,two); 
//   bool result3 = WM8960::_writeRegisterMultiBits(WM8960_REG_PLL_K_3,8,0,three); 
//   if (result1 && result2 && result3) // If all I2C sommands Ack'd, then...
//   {
//     return true;
//   }
//   return false;  
// }

// // 0=integer, 1=fractional
// bool WM8960::setSMD(bool mode)
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_PLL_N, 5, mode);
// }

//  // 0=MCLK, 1=PLL_output
// bool WM8960::setCLKSEL(bool sel)
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_CLOCKING_1, 0, sel);
// }

// // (0=divide by 1), (2=div by 2) *1 and 3 are "reserved"
// bool WM8960::setSYSCLKDIV(uint8_t div) 
// {
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_CLOCKING_1,2,1,div);  
// }

// // 000 = SYSCLK / (1.0*256). See ds pg 57 for other options
// bool WM8960::setADCDIV(uint8_t div) 
// {
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_CLOCKING_1,8,6,div);  
// }

// // 000 = SYSCLK / (1.0*256). See ds pg 57 for other options
// bool WM8960::setDACDIV(uint8_t div) 
// {
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_CLOCKING_1,5,3,div);  
// }

// bool WM8960::setBCLKDIV(uint8_t div)
// {
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_CLOCKING_2,3,0,div);  
// }

// // Class D amp, 111= SYSCLK/16, so 11.2896MHz/16 = 705.6KHz
// bool WM8960::setDCLKDIV(uint8_t div) 
// {
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_CLOCKING_2,8,6,div);
// }

// bool WM8960::setALRCGPIO()
// {
//   // This setting should not be changed if ADCs are enabled.
//   return WM8960::_writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_2, 6, 1);
// }

// bool WM8960::enableMasterMode()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_1, 6, 1);
// }

// bool WM8960::enablePeripheralMode()
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_1, 6, 0);
// }

// bool WM8960::setWL(uint8_t word_length)
// {
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_AUDIO_INTERFACE_1,3,2,word_length);  
// }

// bool WM8960::setLRP(bool polarity)
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_1, 4, polarity);
// }

// bool WM8960::setALRSWAP(bool swap)
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_1, 8, swap);
// }

// bool WM8960::setVROI(bool setting)
// {
//   return WM8960::_writeRegisterBit(WM8960_REG_ADDITIONAL_CONTROL_3, 6, setting);
// }

// bool WM8960::setVSEL(uint8_t setting)
// {
//   return WM8960::_writeRegisterMultiBits(WM8960_REG_ADDITIONAL_CONTROL_1,7,6,setting); 
// }

// // convertDBtoSetting
// // This function will take in a dB value (as a float), and return the 
// // corresponding volume setting necessary.
// // For example, Headphone volume control goes from 47-120.
// // While PGA gain control is from 0-63.
// // The offset values allow for proper conversion.
// //
// // dB - float value of dB
// //
// // offset - the differnce from lowest dB value to lowest setting value
// //
// // stepSize - the dB step for each setting (aka the "resolution" of the setting)
// // This is 0.75dB for the PGAs, 0.5 for ADC/DAC, and 1dB for most other amps.
// //
// // minDB - float of minimum dB setting allowed, note this is not mute on the 
// // amp. "True mute" is always one stepSize lower.
// //
// // maxDB - float of maximum dB setting allowed. If you send anything higher, it
// // will be limited to this max value.
// uint8_t WM8960::convertDBtoSetting(float dB, float offset, float stepSize, float minDB, float maxDB)
// {
//   // Limit incoming dB values to acceptable range. Note, the minimum limit we
//   // want to limit this too is actually one step lower than the minDB, because
//   // that is still an acceptable dB level (it is actually "true mute").
//   // Note, the PGA amp does not have a "true mute" setting available, so we 
//   // must check for its unique minDB of -17.25.

//   // Limit max. This is the same for all amps.
//   if (dB > maxDB) dB = maxDB;

//   // PGA amp doesn't have mute setting, so minDB should be limited to minDB
//   // Let's check for the PGAs unique minDB (-17.25) to know we are currently
//   // converting a PGA setting.
//   if(minDB == WM8960_PGA_GAIN_MIN) 
//   {
//     if (dB < minDB) dB = minDB;
//   }
//   else // Not PGA. All other amps have a mute setting below minDb
//   {
//     if (dB < (minDB - stepSize)) dB = (minDB - stepSize);
//   }

//   // Adjust for offset
//   // Offset is the number that gets us from the minimum dB option of an amp
//   // up to the minimum setting value in the register.
//   dB = dB + offset; 

//   // Find out how many steps we are above the minimum (at this point, our 
//   // minimum is "0". Note, because dB comes in as a float, the result of this 
//   // division (volume) can be a partial number. We will round that next.
//   float volume = dB / stepSize;

//   volume = round(volume); // round to the nearest setting value.

//   // Serial debug (optional)
//   // Serial.print("\t");
//   // Serial.print((uint8_t)volume);

//   return (uint8_t)volume; // cast from float to unsigned 8-bit integer.
// }

// void WM8960::unmuteHeadPhones() {
//     // Read current volume register values if you have a readRegister() method,
//     // or just set with a reasonable default + clear mute bit

//     uint16_t left_vol = 0b000111100;  // 0d30 = ~0dB
//     uint16_t right_vol = 0b000111100; // same for right

//     // Set update bit (bit 8) and clear mute bit (bit 7)
//     left_vol |= (1 << 8);   // update bit
//     left_vol &= ~(1 << 7);  // clear mute bit

//     right_vol |= (1 << 8);   // update bit
//     right_vol &= ~(1 << 7);  // clear mute bit

//     writeRegister(0x02, left_vol);  // LOUT1
//     writeRegister(0x03, right_vol); // ROUT1
// }