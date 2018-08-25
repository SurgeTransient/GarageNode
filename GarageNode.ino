
 /*
 * REVISION HISTORY
 * Version 1.1: Alex Arent
 * 
 * 
 * 
 * v1.1: Display input 0-7 states on the LCD
 */

//Test Outputs with mosquitto_pub using topic mygateway1-in/Node-ID/Child-ID/1/1/2
//Subscribe to inputs with mosquitto_sub using topic mygateway1-in/Node-ID/Child-ID

// Enable debug prints
//#define MY_DEBUG
#define MY_NODE_ID 2
#define MY_RADIO_NRF24
#define MY_RF24_PA_LEVEL RF24_PA_MAX
#define MY_DEFAULT_TX_LED_PIN 8
#define MY_DEFAULT_RX_LED_PIN 5
#define MY_SPLASH_SCREEN_DISABLED
//#define MY_SIGNING_ATSHA204 //!< Hardware signing using ATSHA204A

// SETTINGS FOR MY_SIGNING_ATSHA204

//#define MY_SIGNING_ATSHA204_PIN 17 //!< A3 - pin where ATSHA204 is attached
#include <Wire.h>    
#include <SPI.h>
#include <MySensors.h>  
#include <DHT.h>
#include "PCF8574.h"  //0x27 = LCD, 0x21 = input port, 0x22 = output port
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);
PCF8574 inputExpander;
PCF8574 outputExpander;
DHT dht;

// Set this offset if the sensor has a permanent small offset to the real temperatures
#define SENSOR_TEMP_OFFSET 0

// Sleep time between sensor updates (in milliseconds)
// Must be >1000ms for DHT22 and >2000ms for DHT11
static const uint64_t UPDATE_INTERVAL = 5000;

// Force sending an update of the temperature after n sensor reads, so a controller showing the
// timestamp of the last update doesn't show something like 3 hours in the unlikely case, that
// the value didn't change since;
// i.e. the sensor would force sending an update every UPDATE_INTERVAL*FORCE_UPDATE_N_READS [ms]
static const uint8_t FORCE_UPDATE_N_READS = 10;

#define ON 1                  // GPIO value to write to turn on attached relay
#define OFF 0                 // GPIO value to write to turn off attached relay

//uP I/O pin def's
#define GENERIC_OUTPUT_O A3
#define GENERIC_OUTPUT_1 7
#define DHT_DATA_PIN 3

//Input definitions for Input PCF8574 Port
#define GARAGE_DOOR_LEFT_CLOSED 0
#define GARAGE_DOOR_LEFT_OPEN 1
#define GARAGE_DOOR_RIGHT_CLOSED 2
#define GARAGE_DOOR_RIGHT_OPEN 3
#define SIDE_GARAGE_DOOR 4
#define GARAGE_WINDOW 5

//Temp & humidity sensor stuff
#define CHILD_ID_HUM 0          //'mygateway1-out/2/0/#'
#define CHILD_ID_TEMP 1         //'mygateway1-out/2/1/#'

//PCF8574 #1 Outputs            //mosquitto_pub test topics
#define CHILD_ID_OUTPUT_0 2     //'mygateway1-in/2/2/1/1/2' -m '1' = OFF, //'mygateway1-in/2/2/1/1/2' -m '0' = ON
#define CHILD_ID_OUTPUT_1 3     //'mygateway1-in/2/3/1/1/2' -m '1' = OFF, //'mygateway1-in/2/3/1/1/2' -m '0' = ON
#define CHILD_ID_OUTPUT_2 4     //'mygateway1-in/2/4/1/1/2' -m '1' = OFF, //'mygateway1-in/2/4/1/1/2' -m '0' = ON
#define CHILD_ID_OUTPUT_3 5     //'mygateway1-in/2/5/1/1/2' -m '1' = OFF, //'mygateway1-in/2/5/1/1/2' -m '0' = ON
#define CHILD_ID_OUTPUT_4 6     //'mygateway1-in/2/6/1/1/2' -m '1' = OFF, //'mygateway1-in/2/6/1/1/2' -m '0' = ON
#define CHILD_ID_OUTPUT_5 7     //'mygateway1-in/2/7/1/1/2' -m '1' = OFF, //'mygateway1-in/2/7/1/1/2' -m '0' = ON
#define CHILD_ID_OUTPUT_6 8     //'mygateway1-in/2/8/1/1/2' -m '1' = OFF, //'mygateway1-in/2/8/1/1/2' -m '0' = ON
#define CHILD_ID_OUTPUT_7 9     //'mygateway1-in/2/9/1/1/2' -m '1' = OFF, //'mygateway1-in/2/9/1/1/2' -m '0' = ON

//PCF8574 #2 Inputs             //mosquitto_sub test topics
#define CHILD_ID_INPUT_0 10     //'mygateway1-out/2/10/#'
#define CHILD_ID_INPUT_1 11     //'mygateway1-out/2/11/#'
#define CHILD_ID_INPUT_2 12     //'mygateway1-out/2/12/#'
#define CHILD_ID_INPUT_3 13     //'mygateway1-out/2/13/#'
#define CHILD_ID_INPUT_4 14     //'mygateway1-out/2/14/#'
#define CHILD_ID_INPUT_5 15     //'mygateway1-out/2/15/#'
#define CHILD_ID_INPUT_6 16     //'mygateway1-out/2/16/#'
#define CHILD_ID_INPUT_7 17     //'mygateway1-out/2/17/#'

//Internal Outputs
#define CHILD_ID_Internal_Output_0  18  //'mygateway1-in/2/18/1/1/2' -m '1' = OFF, //'mygateway1-in/2/2/1/1/2' -m '0' = ON    
#define CHILD_ID_Internal_Output_1  19  //'mygateway1-in/2/19/1/1/2' -m '1' = OFF, //'mygateway1-in/2/2/1/1/2' -m '0' = ON   

#define PCF8574_INPUT_CHANGED 2

float lastTemp;
float lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
bool metric = false;
bool readPCF8574 = false;
byte PCF8574inputs = 0;           //Set to zero for initial value

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);

MyMessage msgLeftGarageDoorClosed(CHILD_ID_INPUT_0, V_STATUS);
MyMessage msgLeftGarageDoorOpen(CHILD_ID_INPUT_1, V_STATUS);
MyMessage msgRightGarageDoorClosed(CHILD_ID_INPUT_2, V_STATUS);
MyMessage msgRightGarageDoorOpen(CHILD_ID_INPUT_3, V_STATUS);
MyMessage msgSideGarageDoorOpen(CHILD_ID_INPUT_4, V_STATUS);
MyMessage msgGarageWindowOpen(CHILD_ID_INPUT_5, V_STATUS);

void presentation()  
{ 
  sendSketchInfo("Garage Controller Node", "1.1");

  //Humidity and Temp Sensor
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);

  //Output PCF8574 
  present(CHILD_ID_OUTPUT_0, S_BINARY);
  present(CHILD_ID_OUTPUT_1, S_BINARY);
  present(CHILD_ID_OUTPUT_2, S_BINARY);
  present(CHILD_ID_OUTPUT_3, S_BINARY);
  present(CHILD_ID_OUTPUT_4, S_BINARY);
  present(CHILD_ID_OUTPUT_5, S_BINARY);
  present(CHILD_ID_OUTPUT_6, S_BINARY);
  present(CHILD_ID_OUTPUT_7, S_BINARY);

  //Input PCF8574
  present(CHILD_ID_INPUT_0, S_BINARY);
  present(CHILD_ID_INPUT_1, S_BINARY);
  present(CHILD_ID_INPUT_2, S_BINARY);
  present(CHILD_ID_INPUT_3, S_BINARY);
  present(CHILD_ID_INPUT_4, S_BINARY);
  present(CHILD_ID_INPUT_5, S_BINARY);
  present(CHILD_ID_INPUT_6, S_BINARY);
  present(CHILD_ID_INPUT_7, S_BINARY);

  //Internal uP Outputs
  present(CHILD_ID_Internal_Output_0, S_BINARY);
  present(CHILD_ID_Internal_Output_1, S_BINARY);
}

void setup()
{
  //Init PCF8574 IC's on I2C bus
  inputExpander.begin(0x21);
  outputExpander.begin(0x22);

  lcd.init();                      // initialize the I2C lcd 
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(1,0);
  lcd.print("Initilizing I/O");
  
  //Set uP I/O
  pinMode(MY_DEFAULT_TX_LED_PIN, OUTPUT);
  pinMode(MY_DEFAULT_RX_LED_PIN, OUTPUT);
  pinMode(GENERIC_OUTPUT_O,LOW);
  pinMode(GENERIC_OUTPUT_O, OUTPUT);
  pinMode(GENERIC_OUTPUT_1, OUTPUT);

  //Set all I/O on the input PCF8574 to input with pullups active
  inputExpander.pinMode(0, INPUT_PULLUP);
  inputExpander.pinMode(1, INPUT_PULLUP);
  inputExpander.pinMode(2, INPUT_PULLUP);
  inputExpander.pinMode(3, INPUT_PULLUP);
  inputExpander.pinMode(4, INPUT_PULLUP);
  inputExpander.pinMode(5, INPUT_PULLUP);
  inputExpander.pinMode(6, INPUT_PULLUP);
  inputExpander.pinMode(7,INPUT_PULLUP);

  //Set all I/O on the output PCF8574 to outputs
  outputExpander.pinMode(0,OUTPUT);
  outputExpander.pinMode(1,OUTPUT);
  outputExpander.pinMode(2,OUTPUT);
  outputExpander.pinMode(3,OUTPUT);
  outputExpander.pinMode(4,OUTPUT);
  outputExpander.pinMode(5,OUTPUT);
  outputExpander.pinMode(6,OUTPUT);
  outputExpander.pinMode(7,OUTPUT);

  //Until I add 2N3904's, set all outputs to High, so they won't sink current
  outputExpander.digitalWrite(0,LOW);
  outputExpander.digitalWrite(1,LOW);
  outputExpander.digitalWrite(2,LOW);
  outputExpander.digitalWrite(3,LOW);
  outputExpander.digitalWrite(4,LOW);
  outputExpander.digitalWrite(5,LOW);
  outputExpander.digitalWrite(6,LOW);
  outputExpander.digitalWrite(7,LOW);

  //PCF8574 input interrupt output pin
  pinMode(PCF8574_INPUT_CHANGED,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PCF8574_INPUT_CHANGED), handle_PCF8574_input_change, FALLING);
  
  dht.setup(DHT_DATA_PIN); // set data pin of DHT sensor
  lcd.clear();
  lcd.setCursor(1,0);
  lcd.print("Garage Node Active");
  lcd.setCursor(1,1);
  lcd.print("Input States");
  lcd.setCursor(1,2); 
  lcd.print("0 1 2 3 4 5 6 7"); 
}


void loop()      
{   
  // Force reading sensor, so it works also after wait()
  dht.readSensor(true);
  
  // Get temperature from DHT library
  float temperature = dht.getTemperature();
  if (isnan(temperature)) {
    Serial.println("Failed reading temperature from DHT!");
  } else if (temperature != lastTemp || nNoUpdatesTemp == FORCE_UPDATE_N_READS) {
    // Only send temperature if it changed since the last measurement or if we didn't send an update for n times
    lastTemp = temperature;
    if (!metric) {
      temperature = dht.toFahrenheit(temperature);
    }
    // Reset no updates counter
    nNoUpdatesTemp = 0;
    temperature += SENSOR_TEMP_OFFSET;
    send(msgTemp.set(temperature, 1));
  } else {
    // Increase no update counter if the temperature stayed the same
    nNoUpdatesTemp++;
  }

  // Get humidity from DHT library
  float humidity = dht.getHumidity();
  if (isnan(humidity)) {
    Serial.println("Failed reading humidity from DHT");
  } else if (humidity != lastHum || nNoUpdatesHum == FORCE_UPDATE_N_READS) {
    // Only send humidity if it changed since the last measurement or if we didn't send an update for n times
    lastHum = humidity;
    // Reset no updates counter
    nNoUpdatesHum = 0;
    send(msgHum.set(humidity, 1));
    
  } else {
    // Increase no update counter if the humidity stayed the same
    nNoUpdatesHum++;
  }

//if(readPCF8574 == true)
  //{
   //readPCF8574 = false;
   updateInputStates(inputExpander.read());
  //}
wait(UPDATE_INTERVAL); 
}

void handle_PCF8574_input_change(void)
{
  readPCF8574 = true;
}

void receive(const MyMessage &message)
{
  // We only expect one type of message from controller. But we better check anyway.
  //if (message.type==V_STATUS) {
    // Change PCF8574 output state
    if(message.sensor >= 2 && message.sensor <= 9)  //I2C I/O expander sensor ID's
    {
     outputExpander.digitalWrite(message.sensor - 2, message.getBool()?ON:OFF);
    }
    else
    {
      switch(message.sensor)
      {
        case CHILD_ID_Internal_Output_0:
          digitalWrite(GENERIC_OUTPUT_O, message.getBool()?ON:OFF);
          break;
        case CHILD_ID_Internal_Output_1:
          digitalWrite(GENERIC_OUTPUT_1, message.getBool()?ON:OFF);
          break;
      }
    }
    //Store state in eeprom
    saveState(message.sensor, message.getBool());
    //Write some debug info
    Serial.print("Incoming change for output:");
    Serial.print(message.sensor);
    Serial.print(", New status: ");
    Serial.println(message.getBool());
 // }
}

void updateInputStates(byte state)
{
Serial.print("PCF8574 Input State Change Detected. Value = ");
Serial.println(state);
//byte changedBits = state ^ PCF8574inputs;                   //What bits changed?
PCF8574inputs = state;                                      //Set old state to current state
byte changedBits = state;
for(int i = 0; i<8; i++)
{
  byte bitChanged = changedBits & 0b000000001;              //Mask other bits
  Serial.print("Input ");
  Serial.print(i);
  Serial.print(" Changed State = ");
  Serial.println(bitChanged);
  //if(bitChanged)
  //{
    Serial.print("State = ");
    byte inputValue = (state & (0b00000001 << i)) >> i;     //Extract the value of the input
    Serial.println(inputValue);

    //Send updates as needed
    switch(i)
    {
      case GARAGE_DOOR_LEFT_CLOSED: 
        lcd.setCursor(1,3);
        lcd.print(inputValue?"1":"0");
        repeat(msgLeftGarageDoorClosed.set(inputValue?"1":"0",1),3);
        break;
      case GARAGE_DOOR_LEFT_OPEN:
        lcd.setCursor(2,3);
        lcd.print(inputValue?"1":"0");
        repeat(msgLeftGarageDoorOpen.set(inputValue?"1":"0"),3);
        break;
      case GARAGE_DOOR_RIGHT_CLOSED:
        lcd.setCursor(3,3);
        lcd.print(inputValue?"1":"0");
        repeat(msgRightGarageDoorClosed.set(inputValue?"1":"0"),3);
        break;
      case GARAGE_DOOR_RIGHT_OPEN: 
        lcd.setCursor(4,3);
        lcd.print(inputValue?"1":"0");
        repeat(msgRightGarageDoorOpen.set(inputValue?"1":"0"),3);
        break;
      case SIDE_GARAGE_DOOR:
        lcd.setCursor(5,3);
        lcd.print(inputValue?"1":"0");
        repeat(msgSideGarageDoorOpen.set(inputValue?"1":"0"),3);
        break;
      case GARAGE_WINDOW:
        lcd.setCursor(6,3);
        lcd.print(inputValue?"1":"0");
        repeat(msgGarageWindowOpen.set(inputValue?"1":"0"),3);
        break;
      //case 6:     
      //case 7:
    }
  
  //}
  changedBits = changedBits >> 1;                           //Get next current bit
}

}

void repeat(MyMessage &msg, int repeats)
{
  int repeat = 1;
  int repeatdelay = 0;
  boolean sendOK = false;
  
  while ((sendOK == false) and (repeat < repeats)) {
    if (send(msg)) {
      sendOK = true;
    } else {
      sendOK = false;
      repeatdelay += 250;
    } repeat++; delay(repeatdelay);
  }
}




