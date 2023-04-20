/*
 * Project Capstone
 * Description: 
 * Author: Isaac De leon
 * Date:4/10/2023
 */
#include <JsonParserGeneratorRK.h> 
#include <Particle.h>             
#include <Wire.h>           
#include <math.h>
#include <neopixel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MQTT.h>

#include "cred.h"
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_BME280.h"
#include "colors.h"
#include "IoTClassroom_CNM.h"
#include "HX711.h"
#include "DS18B20.h"
#define OLED_RESET D4
#define Addr 0x2A
IoTTimer timer;
TCPClient TheClient;

// Setup the MQTT client class by passing in the WiFi client, MQTT server, and login details
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);

/****************************** Feeds ***************************************/ 
// Setup a feed called <object> for publishing. 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>

//Adafruit_MQTT_Publish theTempuratureObject = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Feed2_Tempurature");
Adafruit_MQTT_Publish theCurrentSensorOneStateObject = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/CurrentSensorOneState");
Adafruit_MQTT_Publish theCurrentSensorOneActualValue = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/CurrentSensorOneActualValue");
Adafruit_MQTT_Publish theCurrentSensorTwoStateObject = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/CurrentSensorTwoState");
Adafruit_MQTT_Publish theCurrentSensorTwoActualValue = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/CurrentSensorTwoActualValue");
Adafruit_MQTT_Publish pubTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temp");
Adafruit_MQTT_Publish pubcurrent = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/current");
Adafruit_MQTT_Publish pubBtemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/btemp");
Adafruit_MQTT_Publish pubother = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/other");
Adafruit_MQTT_Publish pubpsi = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/other");


Adafruit_MQTT_Publish pubtype = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/type");
Adafruit_MQTT_Publish pubmax = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/max");
Adafruit_MQTT_Publish pubchannel = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/channel");



float var_SensorValueToPublish;
float var_SensorActualValue;

// Adafruit.io Set Up END
// For the neopixels: Pixel 0 is for sensor one, pixel 1 is for sensor two
const int PIN = 2; // Pin 2 on Argon for output signal to neopixels
const int NUMPIXELS = 2; // the actual number of neopixels
//Adafruit_NeoPixel thePixelObject(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
//supports WS2811/WS2812/WS2813
//neopixel thePixelObject(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
// Constructor: number of LEDs, pin number, LED type
const int pixelDelay=250; // just a variable I created to delay
const int startPixel=0; // the first pixel is 0 (zero based)
int var_PixelBrightness = 200; // the brightness value can go from 1 to 255
// thePixelObject.Color() takes RGB values from 0,0,0, to 255,255,255 - or a defined color from an included COLORS file
//int var_PixelColor = 0x008000;
// The broker's server may sever the connection if a publish event is not done prior to the default of 5 minutes.
// This variable will capture the elapsed time since last publish and 
// if that elapsed time >= 2 minutes, the program will ping the server.

char strOut[1024];

int var_LastServerPing = 0;
int var_LastCurrentSensor1Read = 0;
int var_LastCurrentSensor2Read = 0;
float var_SensorMisreadValue = 16777.21;
bool onOrOff;
byte data[36];
int typeOfSensor, maxCurrent, noOfChannel, i;
//====================================================================================
//SYSTEM_THREAD(ENABLED);
const char n = 0xA4 ; // Decimal 248 = 0 xF8
const int lightpin=D7;
 const int PIXELPIN = D8 ; // Pin the NeoPixels are connected to
 const int PIXELCOUNT = 2; // Total number of NeoPixels
Adafruit_BME280 bme ; // this is for I2C device
String DataTime, TimeOnly ;
Adafruit_SSD1306 display (OLED_RESET);
int temp;
Adafruit_NeoPixel pixel ( PIXELCOUNT , PIXELPIN , WS2812B ); 
const int pressuretranspin=A0;  
//pressure pins for reading fluid preesure and tem""tempertaure"
float PressureReadings;
const int temprod = D7;
bool status;
const int  THERMISTORPIN =A2;         
// resistance at 25 degrees C
const int  THERMISTORNOMINAL = 10000;      
// temp. for nominal resistance (almost always 25 C)
const int  TEMPERATURENOMINAL= 25; 
// how many samples to take and average, more takes longer
// but is more 'smooth'
const int  NUMSAMPLES =125;
// The beta coefficient of the thermistor (usually 3000-4000)
const int  BCOEFFICIENT =3950;
// the value of the 'other' resistor
const int   SERIESRESISTOR =10000;
float steinhartF;
int samples[NUMSAMPLES];
SYSTEM_MODE(SEMI_AUTOMATIC);
void MQTT_connect();
bool MQTT_ping();
void setup()   {  
      pinMode(THERMISTORPIN,INPUT);
 pinMode (lightpin, OUTPUT);
pinMode(D7, OUTPUT);
  timer.startTimer(5000);
Serial.begin(9600);
Wire.begin ();     
  pinMode ( pressuretranspin , INPUT) ;
pixel . begin () ;
pixel . show () ;
pixel.setBrightness (255);
  waitFor(Serial.isConnected, 15000);
 WiFi.connect();
  while(WiFi.connecting()) 
     {
      Serial.printf(".");
       delay(250);
     }
  status= bme.begin (0x76) ;
  waitFor(Serial.isConnected,10000);
  WiFi.on();
  WiFi.connect();
 while(WiFi.connecting()) {
    Serial.printf(".");
  }
 Serial.printf("\n\n");
 Particle . syncTime () ; // Sync time with Particle Cloud
 Serial.begin(9600);
 Wire.begin();
 display.begin(SSD1306_SWITCHCAPVCC, 0X3C);
 Wire.begin();
 bme.begin(0x76);
  // Put initialization like pinMode and begin functions here.
  Serial.begin(9600);
  //while(!Serial);
  waitFor(Serial.isConnected, 15000);
  delay(3000);
  Serial.println("The CurrentMonitor program has activated the Serial port.");
  pinMode(2,OUTPUT); // used for signal for neopixels
  // Initiate the Wire library and join the I2C bus as a master or slave. This should normally be called only once.
  // address: the 7-bit slave address (optional); if not specified, join the bus as a master. 
  Wire.begin();
  //Connect to WiFi but not Particle Cloud
  WiFi.connect();
  while(WiFi.connecting()) 
     {
      Serial.printf(".");
       delay(250);
     }
  Serial.printf("\n");
  currentInit();  // call to initialize NCD board hosting current sensors
  // Request 6 bytes of data
  Wire.requestFrom(Addr, 6);

  // Read 6 bytes of data
  if (Wire.available() == 6) 
    {
      for(i=0;i<6;i++) 
        {
          data[i] = Wire.read();
        }
    }
  typeOfSensor = data[0];
  maxCurrent = data[1];
  noOfChannel = data[2];
  // Output data to Serial Monitor
  Serial.printf("Type of Sensor %i \n",typeOfSensor);
  Serial.printf("Max Current: %i \n", maxCurrent);
  Serial.printf("No. of Channels: %i \n", noOfChannel);
  delay(5000);
}
void loop(){
 
  MQTT_connect();
  MQTT_ping();

pixel.setPixelColor (2,blue);
pixel.show ();

 
  
 sleepULP();
}
void sleepULP(){
  SystemSleepConfiguration config;
  config.mode(SystemSleepMode::ULTRA_LOW_POWER).duration(60000);
  SystemSleepResult result = System.sleep(config);
  delay(2000);
  if(result.wakeupReason() == SystemSleepWakeupReason::BY_RTC){
    Serial.printf("Awakened by RTC\n");
    timer.startTimer(90000);
  }
  while(!timer.isTimerReady()){

  PressureReadings = analogRead(pressuretranspin)/606.00-1;  // read the input pin
  Serial.println(PressureReadings);          // debug value
   display.clearDisplay();
   display.display();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
   display.printf("Pressure. %f\n",PressureReadings );
 temp = bme.readTemperature () ; // deg C
   display.printf("Board Temp. C^%i\n",temp );
    Serial.printf("Board Temp. C^%i\n",temp );
       display.printf("PSI mot temp F^%f \n",steinhartF );
            Serial.printf("Current Value of Sensor One: %0.5f \n", var_SensorActualValue);
            display.printf("Current. %f\n",var_SensorActualValue);
   display.display();
   uint8_t i;
  float average;

  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(THERMISTORPIN);
   delay(10);
  }
  
  // average all the samples out
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;

  Serial.print("Average analog reading "); 
  Serial.println(average);
  
  // convert the value to resistance
  average = 4095 / average - 1;
  average = SERIESRESISTOR / average;
  Serial.print("Thermistor resistance "); 
  Serial.println(average);
  
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert absolute temp to C
  steinhartF = steinhart * 1.8 + 32;
  Serial.print("Temperature "); 
    pubTemp.publish(steinhartF);
  pubcurrent.publish(var_SensorActualValue);
  pubBtemp.publish(PressureReadings);
  pubpsi.publish(PressureReadings);
    pubother.publish(temp);
    
    pubtype.publish(typeOfSensor);
    pubmax.publish(maxCurrent);
    pubchannel.publish(noOfChannel);



  Serial.print(steinhartF);
  Serial.println(" *F");
   display.printf("fluid/psi temp %f \n",steinhartF );
  delay(1000);



  // if ((millis()-var_LastServerPing) > 120000) // (120 seconds)
  //   {
  //     Serial.println("Pinging the MQTT server");
  //     // ping the server to keep the mqtt connection alive
  //     if(! mqtt.ping()) // if I cannot ping the server, force a disconnect
  //        {
  //          Serial.println("Forcing an MQTT disconnect");
  //          mqtt.disconnect(); // forcing a disconnect will force a reconnection
  //          Serial.println("Attempting to reestablish MQTT connection");
  //        }
  //     var_LastServerPing = millis();
  //   }

  ReadSensorOne();
  
  delay(1); // just enough of a delay to allow completion of other tasks
}


  if((millis()-var_LastCurrentSensor1Read) > 15000); // wait 15 seconds before trying to read the sensor again
  {
   var_LastCurrentSensor1Read = millis();

   var_SensorActualValue = getCurrent(Addr,1);
   Serial.printf("Current Value of Sensor One: %0.5f \n", var_SensorActualValue);
            
   // This next code segment is for a sensor misread - the value would be greater than 16,000
   // so I set the Sensor value to 0 to stop the exhaust fan from coming on unnecessarily
   if (var_SensorActualValue == var_SensorMisreadValue) // sensor reads an ON state
   {
     var_SensorValueToPublish = 0;
   }
  }
}

void MQTT_connect() {
  int8_t ret;
   if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
   Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
   Serial.printf("Retrying MQTT connection in 5 seconds...\n");
   mqtt.disconnect();
   delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}
bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;
  if ((millis()-last)>120000) {
    Serial.printf("Pinging MQTT \n");
    pingStatus = mqtt.ping();
    if(!pingStatus) {
      Serial.printf("Disconnecting \n");
      mqtt.disconnect();
    }
   last = millis();
  }
  return pingStatus;
}

// Function to get Current reading from sensor
float getCurrent(int address, int i) 
{
  //int var_SensorIndex; // index for the arr_current array
  //float arr_current[2];
  // Start I2C Transmission
 Wire.beginTransmission(Addr);
 // Command header byte-1
 Wire.write(0x92);
 // Command header byte-2
 Wire.write(0x6A);
 // Command 1
 Wire.write(0x01);
 // Start Channel No.
 Wire.write(i);
  // End Channel No.
 Wire.write(i);
 // Reserved
 Wire.write(0x00);
 // Reserved
 Wire.write(0x00);
 // CheckSum
 Wire.write((0x92 + 0x6A + 0x01 + i + i + 0x00 + 0x00) & 0xFF);
 // Stop I2C Transmission
  Wire.endTransmission();
 delay(500);
 // Request 3 bytes of data
 Wire.requestFrom(Addr, 3);
 // Read 3 bytes of data
 // msb1, msb, lsb
 byte msb1 = Wire.read();
 byte msb = Wire.read();
 byte lsb = Wire.read();
 var_SensorActualValue = msb1<<16 | msb<<8 | lsb;
 // Convert the data to ampere
 var_SensorActualValue = var_SensorActualValue / 1000;
 return var_SensorActualValue;
}
//===================================================================================
// Initialize current sensor
void currentInit() 
{
  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Command header byte-1
  Wire.write(0x92);
  // Command header byte-2
  Wire.write(0x6A);
  // Command 2 is used to read no of sensor type, Max current, No. of channel
  Wire.write(0x02);
  // Reserved
  Wire.write(0x00);
  // Reserved
  Wire.write(0x00);
  // Reserved
  Wire.write(0x00);
  // Reserved
  Wire.write(0x00);
  // CheckSum
  Wire.write(0xFE);
  // Stop I2C transmission
  Wire.endTransmission();
}
void ReadSensorOne()
{
  if((millis()-var_LastCurrentSensor1Read) > 15000) // wait 15 seconds before trying to read the sensor again
  {
   var_LastCurrentSensor1Read = millis();

   var_SensorActualValue = getCurrent(Addr,1);
   Serial.printf("Current Value of Sensor One: %0.5f \n", var_SensorActualValue);
            
   // This next code segment is for a sensor misread - the value would be greater than 16,000
   // so I set the Sensor value to 0 to stop the exhaust fan from coming on unnecessarily
   if (var_SensorActualValue == var_SensorMisreadValue) // sensor reads an ON state
   {
     var_SensorValueToPublish = 0;
   }
  } 
}
void timerTest(){
 timer.startTimer(1000);
 digitalWrite(lightpin, HIGH);
 if (timer.isTimerReady()) {
    onOrOff = TRUE;
    if(onOrOff){
   digitalWrite(lightpin,LOW);
   delay(1000);
 }
 }
}