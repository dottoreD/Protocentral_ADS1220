
//////////////////////////////////////////////////////////////////////////////////////////
//
//    Arduino library for the ADS1220 24-bit ADC breakout board
//
//    Author: Ashwin Whitchurch
//    Copyright (c) 2018 ProtoCentral
//
//    Based on original code from Texas Instruments
//
//    This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   For information on how to use, visit https://github.com/Protocentral/Protocentral_ADS1220/
//
/////////////////////////////////////////////////////////////////////////////////////////


#include "Arduino.h"

#include "SPI.h"

//ADS1220 SPI commands
#define SPI_MASTER_DUMMY    0xFF
#define RESET               0x06   //Send the RESET command (06h) to make sure the ADS1220 is properly reset after power-up
#define START               0x08    //Send the START/SYNC command (08h) to start converting in continuous conversion mode
#define WREG  0x40
#define RREG  0x20

//Config registers
#define CONFIG_REG0_ADDRESS 0x00
#define CONFIG_REG1_ADDRESS 0x01
#define CONFIG_REG2_ADDRESS 0x02
#define CONFIG_REG3_ADDRESS 0x03

#define REG_CONFIG3_IDAC1routing_MASK    0xE0  // added DR, untested
#define REG_CONFIG3_IDAC2routing_MASK    0x1C  // added DR, untested
#define REG_CONFIG2_VREF_MASK            0xC0  // added DR, untested
#define REG_CONFIG2_FIR_MASK             0x30  // added DR, untested
#define REG_CONFIG2_IDACcurrent_MASK     0x07  // added DR, untested
#define REG_CONFIG1_MODE_MASK            0x18  // added DR, untested
#define REG_CONFIG1_DR_MASK       0xE0
#define REG_CONFIG0_PGA_GAIN_MASK 0x0E
#define REG_CONFIG0_MUX_MASK      0xF0

#define IDAC1_disable     0x00  // added DR, untested
#define IDAC1_AIN0        0x20  // added DR, untested
#define IDAC1_AIN1        0x40  // added DR, untested
#define IDAC1_AIN2        0x60  // added DR, untested
#define IDAC1_AIN3        0x80  // added DR, untested
#define IDAC1_REFP0       0xA0  // added DR, untested
#define IDAC1_REFN0       0xC0  // added DR, untested
#define IDAC1_reserved    0xE0  // added DR, untested

#define IDAC2_disable     0x00  // added DR, untested
#define IDAC2_AIN0        0x04  // added DR, untested
#define IDAC2_AIN1        0x08  // added DR, untested
#define IDAC2_AIN2        0x0C  // added DR, untested
#define IDAC2_AIN3        0x10  // added DR, untested
#define IDAC2_REFP0       0x14  // added DR, untested
#define IDAC2_REFN0       0x18  // added DR, untested
#define IDAC2_reserved    0x1C  // added DR, untested

#define IDAC_OFF     0x00  // added DR, untested
#define IDAC_10      0x01  // added DR, untested
#define IDAC_50      0x02  // added DR, untested
#define IDAC_100     0x03  // added DR, untested
#define IDAC_250     0x04  // added DR, untested
#define IDAC_500     0x05  // added DR, untested
#define IDAC_1000    0x06  // added DR, untested
#define IDAC_1500    0x07  // added DR, untested

#define FIR_OFF      0x00  // added DR, untested
#define FIR_5060     0x10  // added DR, untested
#define FIR_50Hz     0x20  // added DR, untested
#define FIR_60Hz     0x30  // added DR, untested

#define VREF_2048       0x00  // added DR, untested
#define VREF_REFP0      0x40  // added DR, untested
#define VREF_AIN0       0x80  // added DR, untested
#define VREF_ANALOG     0xC0  // added DR, untested

#define MODE_NORMAL     0x00  // added DR, untested
#define MODE_DUTYCYCLE  0x08  // added DR, untested
#define MODE_TURBO      0x10  // added DR, untested
#define MODE_RESERVED   0x18  // added DR, untested

#define DR_20SPS    0x00
#define DR_45SPS    0x20
#define DR_90SPS    0x40
#define DR_175SPS   0x60
#define DR_330SPS   0x80
#define DR_600SPS   0xA0
#define DR_1000SPS  0xC0

#define PGA_GAIN_1   0x00
#define PGA_GAIN_2   0x02
#define PGA_GAIN_4   0x04
#define PGA_GAIN_8   0x06
#define PGA_GAIN_16  0x08
#define PGA_GAIN_32  0x0A
#define PGA_GAIN_64  0x0C
#define PGA_GAIN_128 0x0E

#define MUX_AIN0_AIN1   0x00
#define MUX_AIN0_AIN2   0x10
#define MUX_AIN0_AIN3   0x20
#define MUX_AIN1_AIN2   0x30
#define MUX_AIN1_AIN3   0x40
#define MUX_AIN2_AIN3   0x50
#define MUX_AIN1_AIN0   0x60
#define MUX_AIN3_AIN2   0x70
#define MUX_AIN0_AVSS   0x80
#define MUX_AIN1_AVSS   0x90
#define MUX_AIN2_AVSS   0xA0
#define MUX_AIN3_AVSS   0xB0

#define MUX_SE_CH0      0x80
#define MUX_SE_CH1      0x90
#define MUX_SE_CH2      0xA0
#define MUX_SE_CH3      0xB0

#define _BV(bit) (1<<(bit))

class Protocentral_ADS1220
{
private:
      uint8_t m_config_reg0;
      uint8_t m_config_reg1;
      uint8_t m_config_reg2;
      uint8_t m_config_reg3;

      uint8_t Config_Reg0;
      uint8_t Config_Reg1;
      uint8_t Config_Reg2;
      uint8_t Config_Reg3;

      uint8_t m_drdy_pin=6;
      uint8_t m_cs_pin=7;
  public:
      uint8_t NewDataAvailable;

      Protocentral_ADS1220();
      void begin(uint8_t cs_pin, uint8_t drdy_pin);
      void Start_Conv(void);
      void ads1220_Reset(void);

      void SPI_Command(unsigned char data_in);
      void writeRegister(uint8_t address, uint8_t value);
      uint8_t readRegister(uint8_t address);
      uint8_t * Read_Data(void);
      int32_t Read_WaitForData();

      uint8_t * get_config_reg(void);
      int32_t Read_SingleShot_WaitForData(void);
      int32_t Read_SingleShot_SingleEnded_WaitForData(uint8_t channel_no);
      int32_t Read_Data_Samples();

// control register 0
      void select_mux_channels(int channels_conf);
      void set_pga_gain(int pgagain);
      void PGA_OFF(void);
      void PGA_ON(void);
// control register 1
      void set_data_rate(int datarate);
	  void set_OperationMode(int OPmode);    // added DR
//      void Single_shot_mode_ON(void);     //remarked for deletion, not used
      void set_conv_mode_single_shot(void);
      void set_conv_mode_continuous(void);
      void TemperatureSensorMode_enable(void);    // added DR
      void TemperatureSensorMode_disable(void);   // added DR
      void CurrentSources_ON(void);       // added DR
      void CurrentSources_OFF(void);      // added DR
// control register 2
	  void set_VREF(int vref);             // added DR
	  void set_FIR_Filter(int filter);     // added DR
      void LowSideSwitch_OPEN(void);       // added DR
      void LowSideSwitch_CLOSED(void);        // added DR
	  void set_IDAC_Current(int IDACcurrent);  // added DR
// control register 3
	  void set_IDAC1_Route(int IDAC1routing);   // added DR
	  void set_IDAC2_Route(int IDAC2routing);   // added DR
      void DRDYmode_default(void);       // added DR
      void DRDYmode_DOUT(void);          // added DR
// end control register

};
