/**
 ******************************************************************************
 * File Name          : MCP342x.CPP
 * Description        : MCP3428 ADC DRIVER
 * DATE				  : DEC 15 2019
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2019 MAKERMAX INC.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of MAKERMAX INC. nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#include "mcp342x.h"

#define LEN_DATA_REGISTER  3          // Length of the data registers.
#define LEN_ONE_BYTE       1

MCP342X::MCP342X(I2C *conn, SlaveAddress addr) {
    slaveAddress = addr;
    connection = conn;
    
    init();
}

MCP342X::Status MCP342X::init() {
    Status status;
    int16_t val = 0;
    
    // Reads the current configuration.
    if ((status=readData(&val, &currentConfig)) != SUCCESS) {
        return status;
    }

    // Initialize internal variable    
    currentConfig.measurementTrigger = NONE;
    
    return status;
}

MCP342X::Status MCP342X::readData(int16_t *val, Config *config) {
    char buf[LEN_DATA_REGISTER];
    
    // Reads data registers.
    if (connection->read((slaveAddress << 1), buf, LEN_DATA_REGISTER) != 0) {
        return ERROR_I2C_READ;
    }
    
    // Decodes configuration register data.
    decodeConfigurationRegister(config, buf[2]);   

    // Converts AD value
    *val = (int16_t)((buf[0] << 8) | buf[1]);
    
    return SUCCESS;
}

void MCP342X::decodeConfigurationRegister(Config *config, uint8_t regVal) {
    uint8_t tmp = 0;

    // For the meaning of each bit, see the section 5.2 in the datasheet.
    
    // Decodes Ready Bit (~RDY), Bit 7
    tmp = ((0x80 & regVal) >> 7);
    if (tmp == 0x00) {
        config->dataStatus = DATA_UPDATED;
    } else {
        config->dataStatus = DATA_NOT_UPDATED;
    }
    
    // Decodes Channel Selection Bits, Bit 6-5
    tmp = ((0x60 & regVal) >> 5);
    if (tmp == 0x00) {
        config->adcChannel = ADC_CH1;
    } else if (tmp == 0x01) {
        config->adcChannel = ADC_CH2;
    } else if (tmp == 0x02) {
        config->adcChannel = ADC_CH3;
    } else {  //  ch == 0x03
        config->adcChannel = ADC_CH4;
    }

    // Decodes Conversion Mode Bit, Bit 4
    tmp = ((0x10 & regVal) >> 4);
    if (tmp == 0x01) {
        config->conversionMode = CONTINUOUS;
    } else {
        config->conversionMode = ONE_SHOT;
    }

    // Decodes Sample Rate Selection Bit
    tmp = ((0x0C & regVal) >> 2);
    if (tmp == 0x00) {
        config->sampleSetting = SAMPLE_240HZ_12BIT;
    } else if (tmp == 0x01) {
        config->sampleSetting = SAMPLE_60HZ_14BIT;
    } else {
        config->sampleSetting = SAMPLE_15HZ_16BIT;
    }
    
    // Decodes PGA Gain Selection Bits
    tmp = (0x03 & regVal);
    if (tmp == 0x00) {
        config->pgaSetting = PGA_1X;
    } else if (tmp == 0x01) {
        config->pgaSetting = PGA_2X;
    } else if (tmp == 0x02) {
        config->pgaSetting = PGA_4X;
    } else {
        config->pgaSetting = PGA_8X;
    }
}

MCP342X::Status MCP342X::setConfig(const Config *config) {
    char val = 0;
    
    // Measurement trigger
    if (config->measurementTrigger == TRIGGER) {
        val |= 0x80;
    } else {
        val |= 0x00;
    }

    // Channel Selection
    if (config->adcChannel == ADC_CH1) {
        val |= 0x00;
    } else if (config->adcChannel == ADC_CH2) {
        val |= 0x20;
    } else if (config->adcChannel == ADC_CH3) {
        val |= 0x40;
    } else {  // config->adcChannel == ADC_CH4
        val |= 0x60;
    }
    
    // Conversion Mode
    if (config->conversionMode == CONTINUOUS) {
        val |= 0x10;
    } else if (config->conversionMode == ONE_SHOT) {
        val |= 0x00;
    }
    
    // Sample Rate
    if (config->sampleSetting == SAMPLE_240HZ_12BIT) {
        val |= 0x00;
    } else if (config->sampleSetting == SAMPLE_60HZ_14BIT) {
        val |= 0x04;
    } else { //config->sampleSetting == SAMPLE_15HZ_16BIT
        val |= 0x08;
    }
    
    // PGA Gain Selection
    if (config->pgaSetting == PGA_1X) {
        val |= 0x00;
    } else if (config->pgaSetting == PGA_2X) {
        val |= 0x01;
    } else if (config->pgaSetting == PGA_4X) {
        val |= 0x02;
    } else { // config->pgaSetting == PGA_8X) {
        val |= 0x03;
    }
    
    // Write to the device.
    if (connection->write((slaveAddress << 1), &val, LEN_ONE_BYTE) != 0) {
        return ERROR_I2C_WRITE;
    }
    
    return SUCCESS;
}


MCP342X::Status MCP342X::setChannel(AdcChannel ch) {
    currentConfig.adcChannel = ch;
    return setConfig(&currentConfig);
}

MCP342X::AdcChannel MCP342X::getChannel() {
    return currentConfig.adcChannel;
}

MCP342X::Status MCP342X::setConversionMode(ConversionMode mode) {
    currentConfig.conversionMode = mode;
    return setConfig(&currentConfig);
}

MCP342X::ConversionMode MCP342X::getConversionMode() {
    return currentConfig.conversionMode;
}

MCP342X::Status MCP342X::setSampleSetting(SampleSetting s) {
    currentConfig.sampleSetting = s;
    return setConfig(&currentConfig);
}

MCP342X::SampleSetting MCP342X::getSampleSetting() {
    return currentConfig.sampleSetting;
}

MCP342X::Status MCP342X::setPgaSetting(PgaSetting s) {
    currentConfig.pgaSetting = s;
    return setConfig(&currentConfig);
}

MCP342X::PgaSetting MCP342X::getPgaSetting() {
    return currentConfig.pgaSetting;
}

MCP342X::Status MCP342X::getData(Data *pt) {
    Status status;
    
    int16_t val = 0;
    if ((status=readData(&val, &currentConfig)) != SUCCESS) {
        return status;
    }
    
    pt->st = currentConfig.dataStatus;
    pt->value = val;
    
    return status;
}

MCP342X::Status MCP342X::trigger(){
    Status status;
    
    currentConfig.measurementTrigger = TRIGGER;
    status = setConfig(&currentConfig);
    currentConfig.measurementTrigger = NONE;
    
    return status;
}

