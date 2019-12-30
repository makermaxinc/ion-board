#include <mcp47FEB22.h>

#include <MCP342x.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

uint8_t address = 0x68;
MCP342x adc = MCP342x(address);
mcp47FEB22 dac(0x60);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

float cshunt=0.2;
float dshunt=0.2;

void setup() {
dac.begin();

   Serial.begin(9600);
Wire.begin();
dac.reset();
dac.wake();
dac.setVref(0,0);
delay(10);
dac.setGain(0,0);
delay(10);
dac.setPowerDown(1,1);

  // Enable power for MCP342x (needed for FL100 shield only)
  
  // Reset devices
  MCP342x::generalCallReset();
  delay(1); // MC342x needs 300us to settle, wait 1ms
  
  // Check device present
  Wire.requestFrom(address, (uint8_t)1);
  if (!Wire.available()) {
    Serial.print("No device found at address ");
    Serial.println(address, HEX);
    while (1);
  }
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }


  display.display();
  
  // put your setup code here, to run once:

}


float voltage=0;
float chgcurrent=0;
float dchgcurrent=0;
float temp=0;

void loop() {
 dac.wake();
 dac.analogWrite(0,50); //dchg,chg

 voltage=readvoltage();
 chgcurrent=readchgcurrent();
 dchgcurrent=readdchgcurrent();
 temp=readtemp();
  
  display.clearDisplay();
  delay(100);
  display.setTextSize(1);             
  display.setTextColor(WHITE);        
  display.setCursor(0,0);
  display.print("Voltage(V)= ");
  display.println(voltage);
  
  display.setCursor(0,10);             
  display.print("I chg(mA)= ");
  display.println(chgcurrent);
  
  
  display.setCursor(0,20); 
  display.print("I dchg(mA)= ");            
  display.println(dchgcurrent);
  
  display.setCursor(0,30);  
  display.print("NTC(V)= ");           
  display.println(temp);
  

  display.setCursor(0,46);
  display.print("2019 Makermax Inc.");
  display.setCursor(0,56);
  display.print("makermax.ca/boards");
  display.display();
  
  delay(200);
  // put your main code here, to run repeatedly:

}


//Functions

float readvoltage()
{
  long value = 0;
  float voltage=0;
  MCP342x::Config status;
  // Initiate a conversion; convertAndRead() will wait until it can be read
  uint8_t err = adc.convertAndRead(MCP342x::channel3, MCP342x::oneShot,
          MCP342x::resolution16, MCP342x::gain1,
           1000000, value, status);
  delay(200);
  voltage=value;
  voltage=voltage/32767;
  voltage=voltage*2.048;
  voltage=voltage/0.3197;
  return voltage;
}



int readchgcurrent()
{
  long value = 0;
  float chgcurrent=0;
  MCP342x::Config status;
  // Initiate a conversion; convertAndRead() will wait until it can be read
  uint8_t err = adc.convertAndRead(MCP342x::channel1, MCP342x::oneShot,
          MCP342x::resolution16, MCP342x::gain1,
           1000000, value, status);
  delay(200);
  chgcurrent=value;
  chgcurrent=chgcurrent*1000;    
  chgcurrent=chgcurrent/32767;
  chgcurrent=chgcurrent*2.048;
  chgcurrent=chgcurrent/0.2;  //shunt value
  return chgcurrent;
}


int readdchgcurrent()
{
  long value = 0;
  float dchgcurrent=0;
  MCP342x::Config status;
  // Initiate a conversion; convertAndRead() will wait until it can be read
  uint8_t err = adc.convertAndRead(MCP342x::channel2, MCP342x::oneShot,
          MCP342x::resolution16, MCP342x::gain1,
           1000000, value, status);
  delay(200);
  dchgcurrent=value;
  dchgcurrent=dchgcurrent*1000;
  dchgcurrent=dchgcurrent/32767;
  dchgcurrent=dchgcurrent*2.048;
  dchgcurrent=dchgcurrent/dshunt; //shunt value
  return dchgcurrent;
}


int readtemp()
{
  long value = 0;
  float temp=0;
  MCP342x::Config status;
  // Initiate a conversion; convertAndRead() will wait until it can be read
  uint8_t err = adc.convertAndRead(MCP342x::channel4, MCP342x::oneShot,
          MCP342x::resolution16, MCP342x::gain1,
           1000000, value, status);
  delay(200);
  temp=value;
  temp=temp/32767;
  temp=temp*2.048;

  return temp;
}
