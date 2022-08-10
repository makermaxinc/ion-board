/**
 ******************************************************************************
 * File Name          : MAIN.CPP
 * Description        : DEMO CODE FOR THE ION HARDWARE KIT
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


#include "mbed.h"
#include "mcp342x.h"
#include "ssd1306.h"

Serial device(USBTX, USBRX);  // tx, rx
SSD1306 lcd (I2C_SDA, I2C_SCL);

int16_t getAdcData(MCP342X *mcp3428, MCP342X::AdcChannel ch, MCP342X::SampleSetting s) {
      //Configure channel and trigger.
      mcp3428->setChannel(ch);
      mcp3428->setSampleSetting(s);
      mcp3428->trigger();
 
      // polling data
      MCP342X::Data data;
      do {
          wait_us(100000);
          mcp3428->getData(&data);
      } while(data.st == MCP342X::DATA_NOT_UPDATED);
 
      return data.value;
  }

#define I2C_SPEED_100KHZ 100000
#define PIN_SERIAL_TX   P0_4
#define PIN_SERIAL_RX   P0_5 

DigitalOut myled(LED1);
AnalogOut aout(A2);
AnalogOut aout2(D13);



float voltage=0; //ch3
float chg=0;  //ch1
float dchg=0; //ch2
float temp=0; //ch4
float cshunt=0.2;
float dshunt=0.2;


int main() {
 // Instanciate I2C
  I2C i2c(I2C_SDA, I2C_SCL);
  i2c.frequency(I2C_SPEED_100KHZ);

// Serial output for debug. (optional)

MCP342X mcp342x(&i2c, MCP342X::SLAVE_ADDRESS_68H);
mcp342x.setConversionMode(MCP342X::ONE_SHOT); 

    while(1) {
        aout.write_u16(0);
        aout2.write_u16(30000);  // 0 - 65500
        HAL_Delay(100);

// Supposes that the device is MCP3428, which has 4 channels.
const uint8_t CHANNEL_NUM = 4;
// Sampling setting. Ch1 is 12-bit, Ch2 is 14-bit, Ch3 is 16-bit, Ch4 is 16-bit.
const MCP342X::SampleSetting sampleSetting[CHANNEL_NUM] = 
{MCP342X::SAMPLE_15HZ_16BIT, MCP342X::SAMPLE_15HZ_16BIT,
MCP342X::SAMPLE_15HZ_16BIT, MCP342X::SAMPLE_15HZ_16BIT};
          // Data buffer.
int16_t data[CHANNEL_NUM];
          // Measures each channel.
for (int i=0; i < CHANNEL_NUM; i++) {
    data[i]=getAdcData(&mcp342x, (MCP342X::AdcChannel)i, sampleSetting[i]);
 }
          // Prints out the ADC results.
device.printf("%d, %d, %d, %d\r\n", data[0], data[1], data[2], data[3]);

chg=float(data[0]);    
chg=chg/32767;
chg=chg*2.048;
chg=chg/cshunt;

dchg=float(data[1]);
dchg=dchg/32767;
dchg=dchg*2.048;
dchg=dchg/dshunt;

voltage=float(data[2]);
voltage=voltage/32767;
voltage=voltage*2.048;
voltage=voltage/0.3197;


temp=float(data[3]);
temp=temp/32767;
temp=temp*2.048;

lcd.speed (SSD1306::Slow);  // set working frequency
lcd.init();                   // initialize SSD1306
lcd.cls();                    // clear frame buffer
lcd.locate (2,1);             // set text cursor to line 3, column 1
lcd.printf ("ch1=%f",chg);    // print to frame buffer
lcd.locate (3,1);
lcd.printf ("ch2=%f",dchg);
lcd.locate (4,1);
lcd.printf ("ch3=%f",voltage);
lcd.locate (5,1);
lcd.printf ("ch4=%f",temp);

lcd.line (  6, 12, 114, 12, SSD1306::Normal); //
lcd.line (114, 64, 114, 12, SSD1306::Normal); // Surrounds text with 
lcd.line (114, 62,   6, 62, SSD1306::Normal); // a rectangle
lcd.line (  6, 64,   6, 12, SSD1306::Normal); //
//lcd.fill (255, 255);              // fills screen outside rectangle
lcd.redraw();                 // updates actual display transferring frame buffer over I2C bus

HAL_Delay(5000);
aout2.write_u16(0);  // 0 - 65500
aout.write_u16(30000);


for (int i=0; i < CHANNEL_NUM; i++) {
    data[i]=getAdcData(&mcp342x, (MCP342X::AdcChannel)i, sampleSetting[i]);
 }
          // Prints out the ADC results.
device.printf("%d, %d, %d, %d\r\n", data[0], data[1], data[2], data[3]);

chg=float(data[0]);    
chg=chg/32767;
chg=chg*2.048;
chg=chg/cshunt;

dchg=float(data[1]);
dchg=dchg/32767;
dchg=dchg*2.048;
dchg=dchg/dshunt;

voltage=float(data[2]);
voltage=voltage/32767;
voltage=voltage*2.048;
voltage=voltage/0.3197;


temp=float(data[3]);
temp=temp/32767;
temp=temp*2.048;

lcd.speed (SSD1306::Slow);  // set working frequency
lcd.init();                   // initialize SSD1306
lcd.cls();                    // clear frame buffer
lcd.locate (2,1);             // set text cursor to line 3, column 1
lcd.printf ("ch1=%f",chg);    // print to frame buffer
lcd.locate (3,1);
lcd.printf ("ch2=%f",dchg);
lcd.locate (4,1);
lcd.printf ("ch3=%f",voltage);
lcd.locate (5,1);
lcd.printf ("ch4=%f",temp);

lcd.line (  6, 12, 114, 12, SSD1306::Normal); //
lcd.line (114, 64, 114, 12, SSD1306::Normal); // Surrounds text with 
lcd.line (114, 62,   6, 62, SSD1306::Normal); // a rectangle
lcd.line (  6, 64,   6, 12, SSD1306::Normal); //
//lcd.fill (255, 255);              // fills screen outside rectangle
lcd.redraw();                 // updates actual display transferring frame buffer over I2C bus
HAL_Delay(5000);



    }
}


