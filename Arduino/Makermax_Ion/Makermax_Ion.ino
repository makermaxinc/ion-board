//Libraries
#include <mcp47FEB22.h>
#include <MCP342x.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

// Guidelines:
//
//  MODE 0 - IDLE STATE
//  MODE 1 - CHARGING
//  MODE 2 - DISCHARGING
//
//  1. Press and hold SW1 & SW2 for 1 second for Mode to change.
//  2. Press and hold SW1 for 1 second for Decrement in charging/discharging Current by 50mA after entering Mode1 or Mode2.
//  3. Press and hold SW2 for 1 second for Increment in charging/discharging Current by 50mA after entering Mode1 or Mode2.



//Global declarations

bool controlLock = false;
uint8_t red_flag = 0;  //Guard against voltage dropping below 2.8
uint8_t address = 0x68;

MCP342x adc = MCP342x(address);
mcp47FEB22 dac(0x60);

uint8_t mode = 0; // 0 - idle, 1 charge, 2 dischage
float chg_curr  = 0;
float dchg_curr = 0;

//(discharge current value(mA),charge current value(mA))
 float CHARGE     =    500;                            //Initial charge value
 float DISCHARGE  =    500;                            //Initial Discharge value

#define UNDER_VOLTAGE   2.80
#define OVER_VOLTAGE    4.20     
#define SCREEN_WIDTH  128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

float cshunt=0.2;
float dshunt=0.2;





//Initializations
void setup() {
dac.begin();          //Initializes DAC

Serial.begin(9600);
Wire.begin();       //Initializes I2C communication for OLED display

//DAC initializations
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
  
// Check device present - OLED
Wire.requestFrom(address, (uint8_t)1);
  if (!Wire.available()) {
    Serial.print("No device found at address ");
    Serial.println(address, HEX);
    while (1);
  }
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed")); //Make sure global variables are lesser than 1024bytes to avoid this error.
    for(;;); // Don't proceed, loop forever
  }


  display.display();
  
  // put your setup code here, to run once:

pinMode(7, INPUT_PULLUP); //SW1 is D7
pinMode(8, INPUT_PULLUP); //SW2 is D8
}


float voltage=0;
float chgcurrent=0;
float dchgcurrent=0;
float temp=0;





//Main loop
void loop() {
dac.wake();

buttonWait(7,8);    //This enables mode control using - SW1 connected to D7 and SW2 connected to D8

voltage=readvoltage();
 //If under-voltage, reset to idle state
 if(voltage <= UNDER_VOLTAGE || voltage>= OVER_VOLTAGE)
  {
  idle_state();
  }
 
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

  display.setCursor(0,39);  
  display.print("Mode = ");           
  display.println(mode);

  

  display.setCursor(0,49);
  display.print("2019 Makermax Inc.");
  display.setCursor(0,57);
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





//MODE logic implementation
void buttonWait(int SW1, int SW2)
{
int buttonState1 = 0;
int buttonState2 = 0;

buttonState1 = digitalRead(SW1);
buttonState2 = digitalRead(SW2);

//Polling for Charge  
 if(buttonState1== LOW)
 {
  if(!controlLock)
  {
  
  delay(200);
      if(buttonState2 == LOW)
      {
       delay(100);
       mode++;
       if(mode>=0 && mode <=2)
       {
       mode_change(mode);    
       }
       else
       {
       mode = 0;      
       }
      }else
       {
        if(mode == 1)
        {
        delay(500);
        chg_curr -= 50;
        delay(500);
        charging(chg_curr);  
        }
        else if(mode == 2)
        {
        delay(500);
        dchg_curr -= 50;
        discharging(dchg_curr);
        }
       }
  }
 }else if(buttonState2 == LOW)
  {
   if(!controlLock)
   {
        if(mode == 1)
        {
        delay(500);
        chg_curr += 50;
        delay(500);
        charging(chg_curr);  
        }else if(mode == 2)
           {
           delay(500);
           dchg_curr += 50;
           discharging(dchg_curr);
           }
   }
  }
 
 else 
 {
  controlLock = false;
 }

}
  
//Switches to the appropriate function for the specified mode
void mode_change(uint8_t mode){

  if(mode == 0)
    {
     idle_state(); //idle
    }
  else if(mode == 1)
    {
     charging(CHARGE); //Charging
    }
    else if(mode == 2)
    {
     discharging(DISCHARGE); //Discharging
    }
    controlLock = true;
}



//Function to be called for charging
//@param - currentVal(mA)
void charging(float curr_val)
{
    if(curr_val>=0 && curr_val<=1200)
    {
    Serial.print(" Charge- ");
    Serial.println(curr_val);
    chg_curr = curr_val;
    curr_val = curr_val/8.77;
    dac.analogWrite(0,curr_val);
    
    }else
     {
     Serial.println("Enter a value between 0-120; 0->0mA | 120->1.15A");
     curr_val = 0;
     }
  controlLock = true;
  }

  
 //Neither charging nor discharging
 void idle_state()
 {
  dac.analogWrite(0,0);
 }



//Function for discharging
//@param - currentValue(mA)
void discharging(float curr_val)
{
  if(curr_val>=0 && curr_val<=1200)
  {
    Serial.print("Discharge-");
    Serial.println(curr_val);
    dchg_curr = curr_val;
    curr_val = curr_val/8.77;
    dac.analogWrite(curr_val, 0);
  }
  else
   {
    Serial.println("Enter between 0-120");
   }
  controlLock = true;
}
