#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define conversionReg 0x00
#define configReg 0x01
#define lowThrReg 0x02
#define highThrReg 0x03

enum
{
    COMP_QUE_1 = (0x0000),
    COMP_QUE_2 = (0x0001),
    COMP_QUE_4 = (0x0002),
    COMP_QUE_DISABLED = (0x0003)
};

enum
{
    COMP_LAT = (0x0000),
    COMP_NO_LAT = (0x0004)
};

enum
{
    COMP_POL_LOW = (0x0000),
    COMP_POL_HIGH = (0x0008)
};

enum
{
    COMP_MODE_TRAD = (0x0000),
    COMP_MODE_WINDOW = (0x0010)
};

enum
{
    DATA_RATE_8 = (0x0000),
    DATA_RATE_16 = (0x0020),
    DATA_RATE_32 = (0x0040),
    DATA_RATE_64 = (0x0060),
    DATA_RATE_128 = (0x0080),
    DATA_RATE_250 = (0x00A0),
    DATA_RATE_475 = (0x00C0),
    DATA_RATE_860 = (0x00E0)
};

enum
{
    MODE_CONT = (0x0000),
    MODE_SINGLE_SHOT = (0x0100)
};

enum
{
    PGA_FSR_6114 = (0x0000),
    PGA_FSR_4096 = (0x0200),
    PGA_FSR_2048 = (0x0400),
    PGA_FSR_1024 = (0x0600),
    PGA_FSR_512 = (0x0800),
    PGA_FSR_256 = (0x0A00)
};

enum
{
    MUX_CONFIG_A0_A1 = (0x0000),
    MUX_CONFIG_A0_A3 = (0x1000),
    MUX_CONFIG_A1_A3 = (0x2000),
    MUX_CONFIG_A2_A3 = (0x3000),
    MUX_CONFIG_A0_GND = (0x4000),
    MUX_CONFIG_A1_GND = (0x5000),
    MUX_CONFIG_A2_GND = (0x6000),
    MUX_CONFIG_A3_GND = (0x7000)
};

enum
{
    OS = (0x0000),
    OS_START_CONV = (0x8000)
};

constexpr uint16_t MUX_BY_CHANNEL[] = {
    MUX_CONFIG_A0_GND, ///< Single-ended AIN0
    MUX_CONFIG_A1_GND, ///< Single-ended AIN1
    MUX_CONFIG_A2_GND, ///< Single-ended AIN2
    MUX_CONFIG_A3_GND  ///< Single-ended AIN3
}; 

class ads
{
private:
    i2c_inst_t *_i2c;
    int _sda;
    int _scl;
    int _addr;
    int m_gain;
    int m_dataRate;
    uint16_t config;

public:
    ads(i2c_inst_t *i2c, int sda, int scl, int addr);
    void init(int gain, int dataRate);
    int adcReadSingleEnd(int channel);
    float getVoltageSingleEnd(int channel);
    ~ads();
};

ads::ads(i2c_inst_t *i2c, int sda, int scl, int addr)
{
    _i2c = i2c;
    _sda = sda;
    _scl = scl;
    _addr = addr;
    i2c_init(_i2c, 400 * 1000);
    gpio_set_function(_sda, GPIO_FUNC_I2C);
    gpio_set_function(_scl, GPIO_FUNC_I2C);
    gpio_pull_up(_sda);
    gpio_pull_up(_scl);
}

ads::~ads()
{
}

void ads::init(int gain = PGA_FSR_6114, int dataRate = DATA_RATE_8)
{
    m_gain = gain;
    m_dataRate = dataRate;
    
}

int ads::adcReadSingleEnd(int channel){
    config = 0;
    config = OS_START_CONV | MODE_CONT  | COMP_MODE_TRAD |
             COMP_POL_LOW | COMP_NO_LAT |
             COMP_QUE_1;
    config |= MUX_BY_CHANNEL[channel] | m_gain | m_dataRate;
    uint8_t datatosend1[3];
    uint8_t datatosend2[2];
    uint8_t data[2];
    datatosend1[0] = (uint8_t)configReg;
    datatosend1[1] = ((uint16_t)config) >> 8;
    datatosend1[2] = ((uint16_t)config) & 0x00FF;
    printf("%08b %08b %08b \n",datatosend1[0],datatosend1[1],datatosend1[2]);
    i2c_write_blocking(_i2c,_addr,datatosend1,3,false);
    datatosend2[0] = (uint8_t)lowThrReg;
    datatosend2[1] = 0x0000;
    i2c_write_blocking(_i2c,_addr,datatosend2,2,false);
    datatosend2[0] = (uint8_t)highThrReg;
    datatosend2[1] = (uint8_t)0x8000;
    i2c_write_blocking(_i2c,_addr,datatosend2,2,false);
    uint8_t regRead = (uint8_t)conversionReg;
    i2c_write_blocking(_i2c,_addr,&regRead,1,false);
    i2c_read_blocking(_i2c,_addr,data,2,false);
    int rawAdc = (data[0] << 8)|(data[1]);
    return rawAdc;
}

float ads::getVoltageSingleEnd(int channel){
    int rawAdc = adcReadSingleEnd(channel);
    float fsRange;
    switch (m_gain) {
    case PGA_FSR_6114:
        fsRange = 6.144f;
        break;
    case PGA_FSR_4096:
        fsRange = 4.096f;
        break;
    case PGA_FSR_2048:
        fsRange = 2.048f;
        break;
    case PGA_FSR_1024:
        fsRange = 1.024f;
        break;
    case PGA_FSR_512:
        fsRange = 0.512f;
        break;
    case PGA_FSR_256:
        fsRange = 0.256f;
        break;
    default:
        fsRange = 0.0f;
    }
    return rawAdc * (fsRange / (32768));
}

