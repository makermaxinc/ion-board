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
 
     // Polling data
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


// Instantiate I2C
 I2C i2c(I2C_SDA, I2C_SCL);
 i2c.frequency(I2C_SPEED_100KHZ);
 
// Serial output for debug. (optional)
MCP342X mcp342x(&i2c, MCP342X::SLAVE_ADDRESS_68H);
mcp342x.setConversionMode(MCP342X::ONE_SHOT);
 
   while(1) {
       aout.write_u16(0);
       aout2.write_u16(0);  // 0 - 32767
       HAL_Delay(1000);
 
// MCP3428
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
 
voltage=float(data[2]); // stores raw value in voltage variable
voltage=voltage/32767;  // convert to true voltage
voltage=voltage*2.048;  // multiply with reference
voltage=voltage/0.3197; // divider ratio
 
 
temp=float(data[3]);
temp=temp/32767;
temp=temp*2.048;
 
lcd.speed (SSD1306::Slow);  // set working frequency
lcd.init();                   // initialize SSD1306
lcd.cls();                    // clear frame buffer
lcd.locate (4,1);
lcd.printf ("ch3=%f",voltage);
 
 
lcd.line (  6, 12, 114, 12, SSD1306::Normal); //
lcd.line (114, 64, 114, 12, SSD1306::Normal); // 
lcd.line (114, 62,   6, 62, SSD1306::Normal); // 
lcd.line (  6, 64,   6, 12, SSD1306::Normal); //
//lcd.fill (255, 255);              // fills screen outside rectangle
lcd.redraw();                 // updates actual display transferring frame buffer over I2C bus
   }
}
