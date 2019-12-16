/**
 ******************************************************************************
 * File Name          : MCP342x.H
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

#ifndef __MCP342X_H__
#define __MCP342X_H__

#include "mbed.h"

/**
 * Device driver for MCP3426, MCP3427, and MCP3428.
 * @note MCP342x is Analog-to-Digital Converter (ADC) IC with I2C interface.
 *
 * Example:
 * @code
 * #include "mbed.h"
 * #include "mcp342x.h"
 *
 * #define I2C_SPEED_100KHZ    100000
 * #define I2C_SPEED_400KHZ    400000
 * #define PIN_SERIAL_TX   P0_4
 * #define PIN_SERIAL_RX   P0_5 
 *
 * int16_t getAdcData(MCP342X *mcp3428, MCP342X::AdcChannel ch, MCP342X::SampleSetting s) {
 *     // Configure channel and trigger.
 *     mcp3428->setChannel(ch);
 *     mcp3428->setSampleSetting(s);
 *     mcp3428->trigger();
 *
 *     // polling data
 *     MCP342X::Data data;
 *     do {
 *         wait_ms(WAIT_ADC_MS);
 *         mcp3428->getData(&data);
 *     } while(data.st == MCP342X::DATA_NOT_UPDATED);
 *
 *     return data.value;
 * }
 *
 * int main(void) {
 *     // Instanciate I2C
 *     I2C i2c(I2C_SDA0, I2C_SCL0);
 *     i2c.frequency(I2C_SPEED_400KHZ);
 *
 *     // Serial output for debug. (optional)
 *     Serial serial(PIN_SERIAL_TX, PIN_SERIAL_RX);
 *
 *     // Instanciate MCP342x
 *     // Suppose that the slave address of MCP342x on your board is .
 *     MCP342X mcp342x(&i2c, MCP342X::SLAVE_ADDRESS_68H);
 *
 *     // Sets MCP342x one-shot measurement mode.
 *     mcp342x.setSampleSetting(MCP342X::ONE_SHOT); 
 *
 *     while(true) {
 *         // Supposes that the device is MCP3428, which has 4 channels.
 *         const uint8_t CHANNEL_NUM = 4;
 *         // Sampling setting. Ch1 is 12-bit, Ch2 is 14-bit, Ch3 is 16-bit, Ch4 is 16-bit.
 *         const MCP342X::SampleSetting sampleSetting[CHANNEL_NUM] = 
 *                 {MCP342X::SAMPLE_240HZ_12BIT, MCP342X::SAMPLE_60HZ_14BIT,
 *                  MCP342X::SAMPLE_15HZ_16BIT, MCP342X::SAMPLE_15HZ_16BIT};
 *         // Data buffer.
 *         int16_t data[CHANNEL_NUM];
 *         // Measures each channel.
 *         for (int i=0; i < CHANNEL_NUM; i++) {
 *             mcp342x.getAdcData(&data[i], (MCP342X::AdcChannel)i, sampleSetting[i]);
 *         }
 *         // Prints out the ADC results.
 *         serial.printf("%d, %d, %d, %d\r\n", data[0], data[1], data[2], data[3]);
 *     }
 * }
 * @endcode
 */
class MCP342X
{
public:
    /**
     * Slave addresses.
     */
    typedef enum {
        SLAVE_ADDRESS_68H = 0x68,     /**< When Adr0 pin = L and Adr1 pin = L, or Adr0 pin = float and Adr1 pin = float. */
        SLAVE_ADDRESS_69H = 0x69,     /**< When Adr0 pin = L and Adr1 pin = float. */
        SLAVE_ADDRESS_6AH = 0x6A,     /**< When Adr0 pin = L and Adr1 pin = H. */
        SLAVE_ADDRESS_6CH = 0x6C,     /**< When Adr0 pin = H and Adr1 pin = L. */
        SLAVE_ADDRESS_6DH = 0x6D,     /**< When Adr0 pin = H and Adr1 pin = float. */
        SLAVE_ADDRESS_6EH = 0x6E,     /**< When Adr0 pin = H and Adr1 pin = H. */
        SLAVE_ADDRESS_6BH = 0x6B,     /**< When Adr0 pin = float and Adr1 pin = L. */
        SLAVE_ADDRESS_6FH = 0x6F,     /**< When Adr0 pin = float and Adr1 pin = H. */
    } SlaveAddress;
    
    /**
     * Status of function. 
     */
    typedef enum {
        SUCCESS,               /**< The function processed successfully. */
        ERROR_I2C_READ,        /**< Error related to I2C read. */
        ERROR_I2C_WRITE,       /**< Error related to I2C write. */
        ERROR,                 /**< General Error */
    } Status;

    /**
     * Conversion mode setting.
     */
    typedef enum {
        CONTINUOUS,            /**< Continuous conversion mode. Default. */
        ONE_SHOT,              /**< One-shot conversion mode. */
    } ConversionMode;

    /**
     * Data ready status.
     */
    typedef enum {
        DATA_NOT_UPDATED,       /**< Output register has not been updated. */
        DATA_UPDATED,           /**< Output register has been updated with the latest conversion result. */
    } DataStatus;

    /**
     * Measurement trigger command.
     */
    typedef enum {
        TRIGGER,              /**< Initiate a new conversion. */
        NONE,                 /**< No effect. */
    } MeasurementTrigger;
    
    /**
     * Sample rate and resolution setting.
     */
    typedef enum {
        SAMPLE_240HZ_12BIT,       /**< 240 sample per second with 12 bit data. Default. */
        SAMPLE_60HZ_14BIT,        /**< 60 sample per second with 14 bit data. */
        SAMPLE_15HZ_16BIT,        /**< 15 sample per second with 16 bit data. */
    } SampleSetting;

    
    /**
     * ADC channel selection.
     */
    typedef enum {
        ADC_CH1 = 0,                 /**< Channel 1, default. */
        ADC_CH2 = 1,                 /**< Channel 2 */
        ADC_CH3 = 2,                 /**< Channel 3, MCP3428 only, treated as channel 1 by the MCP3426/MCP3427. */
        ADC_CH4 = 3,                 /**< Channel 4, MCP3428 only, treated as channel 2 by the MCP3426/MCP3427. */
    } AdcChannel;
    
    /**
     * Programmable Gain Amplifier setting.
     */
    typedef enum {
        PGA_1X,                  /**< Gain 1x, Default. */
        PGA_2X,                  /**< Gain 2x. */
        PGA_4X,                  /**< Gain 4x. */
        PGA_8X,                  /**< Gain 8x. */
    } PgaSetting;
    
    /**
     * ADC result.
     */
    typedef struct {
        DataStatus st;
        int16_t value;  /**< ADC value. The value takes from -2^11 to (2^11 - 1) when 12 bit sample mode, from -2^13 to (2^13 - 1) when 14 bit sample mode, from -2^15 to (2^15 - 1) when 16bit sample mode. */
    } Data;
    
    /**
     * Constructor.
     *
     * @param conn Pointer to an instance of I2C.
     * @param addr Slave address of the device.
     */
    MCP342X(I2C *conn, SlaveAddress addr);

    /**
     * Sets a ADC channel.
     * @param ch ADC channel which to be the input.
     * @return SUCCESS when succeeded. Other value will be returned when error.
     */
    Status setChannel(AdcChannel ch);
    
    /**
     * Gets the current selected ADC channel.
     * @return ADC channel currently set.
     */
    AdcChannel getChannel();
    
    /**
     * Sets a conversion mode.
     * @param mode Conversion mode which to be set.
     * @return SUCCESS when succeeded. Other value will be returned when error.
     */
    Status setConversionMode(ConversionMode mode);

    /**
     * Gets the current conversion mode.
     * @return Current conversion mode.
     */
    ConversionMode getConversionMode();

    /**
     * Sets sample setting, i.e. sampling frequency and resolution bits.
     * @param s Sample setting to be set.
     * @return SUCCESS when succeeded. Other value will be returned when error.
     */
    Status setSampleSetting(SampleSetting s);

    /**
     * Gets the current sample setting.
     * @return Current sample setting.
     */
    SampleSetting getSampleSetting();
    
    /**
     * Sets the gain of Programmable Gain Amplifier (PGA).
     * @param s PGA seeting to be set.
     * @return SUCCESS when succeeded. Other value will be returned when error.
     */
    Status setPgaSetting(PgaSetting s);
    
    /**
     * Gets the current Programmable Gain Amplifier (PGA) setting.
     * @return Current PGA setting.
     */
    PgaSetting getPgaSetting();

    /**
     * Gets the AD value.
     * @return AD value.
     */
    Status getData(Data *pt);
    
    /**
     * Trigger AD conversion. In continuous measurement mode, this function has no effect.
     * @return SUCCESS when succeeded. Other value will be returned when error.
     */
    Status trigger();
    
private:
    typedef struct {
        MeasurementTrigger   measurementTrigger;
        DataStatus           dataStatus;
        ConversionMode       conversionMode;
        SampleSetting        sampleSetting;
        AdcChannel           adcChannel;
        PgaSetting           pgaSetting;
    } Config;


    I2C *connection;        /**< Pointer to an I2C object. */
    uint8_t slaveAddress;   /**< Slave address. */
    Config currentConfig;  /**< Stores the latest configuration. */

    /**
     * Initialize this device.
     * @return SUCCESS when succeeded. Other value will be returned when error.
     */
    Status init();

    /**
     * Reads the data registers including the configuration register.
     * @param val Pointer to the buffer which stores ADC value.
     * @param currentConfig Pointer to the structure which stores the current configuration.
     * @return SUCCESS when succeeded. Other value will be returned when error.
     */
    Status readData(int16_t *val, Config *config);
    
    /**
     * Sets the configuration register.
     * @param Configuration to be set.
     * @return SUCCESS when succeeded. Other value will be returned when error.
     */
    Status setConfig(const Config *config);
    
    /**
     * Decodes a configuration register value and put them into the specified Config structure.
     * @param config Pointer to a Config structure to store the result.
     * @param regVal Register value of the configuration register.
     */
    void decodeConfigurationRegister(Config *config, uint8_t regVal);
};

#endif
