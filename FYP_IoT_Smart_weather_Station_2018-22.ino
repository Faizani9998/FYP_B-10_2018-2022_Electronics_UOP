#include <math.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include "ThingSpeak.h"
#include <Arduino.h>
#include <HardwareSerial.h>


#define RXD2 16
#define TXD2 17

/////////////////////wifi setup end//////////////////////////////
int lcdColumns = 20; // set the LCD number of columns and rows
int lcdRows = 4; // set the LCD number of columns and rows
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  // set LCD address, number of columns and rows

///////////////////////////////Wifi setup ////////////////////////
const char* ssid = "UOP";   // your network SSID (name)
const char* password = "u1234567";   // your network password
WiFiClient  client;
///////////////////////////////////////////////////////////////
unsigned long myChannelNumber1 = 1554312 ;
const char * myWriteAPIKey1 = "AEYCB9Z5ADL2HEU5";
///////////////////thingspeak Channek 2////////////////////////////////
unsigned long myChannelNumber2 = 1558532 ;
const char * myWriteAPIKey2 = "3O0NP5UV0JJDRI4O";
//////////////////////////////////////////////////////////////////////
// Timer variables
unsigned long lastTime = 0;
//unsigned long timerDelay = 30000;
unsigned long timerDelay = 20000;


//pin assigmet/////
//////////////PIN Assigment //////////////////
#define LuxValuePIN   (36)  // 1. Lux Value LDR (Analogue)
#define RainDetecPIN  (39)  // 2. Rain Detector (Analogue)
#define AirQltyPIN    (34)  // 3. AIr Qualty    (Analogue)
#define WindVanePIN   (35) //  4. WindDirecion  (Analogue)
#define WindSensorPIN (32) //  5. WindSpeed     (Digital)
#define SoilMoistPIN  (33) //  6. Soil Moisture (Analogue)
#define DHTPIN        (25) //  7. Humidity      (Digital)
#define AirTempPIN    (27)  // 8.Air Temprature (Digital)
#define SoilTempPIN   (26)  // 9.Soil Temp      (Digital)
#define SnowTrigPIN  (9)   // 10. Snow Depth   (Digital)
#define SnowEchoPIN  (13) //  10.Snow Depth    (Digital)
// #define i2c (PIN 21,22)
////////////////////////////////////////////////////////

////////////////////Values Assigment///////////////////
#define VaneOffset 0; // define the anemometer offset from magnetic north
#define BdRate     115200  //Baud Rate
#define dt_comn    1000  // Delay
#define dt_ADCRead  250   // Delay
#define dt_digital 1000  // Delay



///-----SOIL MOISTURE MODULE(DS18B20,Soil Mositure(Capactive Nature),Ultrasonic Sensor-------------///

///////////////Snow Deph///////////////////////////////
const int trigPin = SnowTrigPIN ;
const int echoPin = SnowEchoPIN ;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;

/////////////////////////Moisture Sensor ///////////////////////////////
int msensor = SoilMoistPIN; // moisture sensor is connected with the analog pin 36 of the esp32
int msvalue = 0; // moisture sensor value
int SoilMoistr ;
//int led = 13;
boolean flag = false;
float SoilTmp;
/////////////////////////////////////
////--------END---SOIL MOISTURE MODULE-----------////

///----------------DDMB MODULE-(DS18B20,DHT11,MQ135,BMP180)------------------///
const int oneWireBus = AirTempPIN;     // GPIO where the DS18B20 is connected to

OneWire oneWire(oneWireBus); // Setup a oneWire instance to communicate with any OneWire devices

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);
/////////////////////////////////////////////////////////
#define MQ135_THRESHOLD_1 1000 // Fresh Air threshold
Adafruit_BMP085 bmp;
// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor for normal 16mhz Arduino
////////////////////////////////////////////////////////

///---------------END -DDMB MODULE------------------///

///---------------RAIN Detection-------------------///
int rainPin = RainDetecPIN;
int thresholdValue = 500; // you can adjust the threshold value
///-------------END--RAIN Detection-------------------///

///-----------------WIND SPEED------------------------///
int VaneValue; // raw analog value from wind vane
int Direction; // translated 0 - 360 direction
int CalDirection; // converted value with offset applied
int LastValue; // last direction value

volatile bool IsSampleRequired; // this is set true every 2.5s. Get wind speed
volatile unsigned int TimerCount; // used to determine 2.5sec timer count
volatile unsigned long Rotations; // cup rotation counter used in interrupt routine
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in isr

float WindSpeed; // speed miles per hour

///--------------END---WIND SPEED------------------------///

///--------------Timer Interrupt-------------------------///
volatile int interruptCounter;
int totalInterruptCounter;
hw_timer_t * timer = NULL;

///------------END--Timer Interrupt ----------------------///

/////////////////////////////////////////////////////////////////

void IRAM_ATTR onTimer() {
 
  TimerCount++;
  if(TimerCount == 6)
{
IsSampleRequired = true;
TimerCount = 0;
}}
///////////////////////////////i2c scan for LCD and BMP180 ////////////////////
///////////////////////////////////////////////////
void i2c_scan()
{
  byte error, address;
  int nDevices;
  lcd.clear();
   lcd.setCursor(0, 0);
      // print message
       lcd.print("I2C Scanner.....");
        delay(1000);
    Serial.println("Scanning...");
   lcd.setCursor(0, 1);
  // print message
  lcd.print("Scanning........!");
  delay(1000);
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
       lcd.setCursor(0, 2);
      // print message
       lcd.print("I2C Address: 0x");
        delay(1000);
      if (address<16) {
        Serial.print("0");
        lcd.setCursor(0, 2);
      // print message
       lcd.print("I2C device 0");
        delay(1000);
      }
      Serial.println(address,HEX);
      lcd.setCursor(16, 2);
       lcd.print(address,HEX);
        delay(1000);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      lcd.setCursor(0, 3);
       lcd.print("Unknow error at address 0x");
        delay(1000);
      if (address<16) {
        Serial.print("0");
        lcd.setCursor(0, 2);
      // print message
       lcd.print("I2C device 0..");
        delay(1000);
      }
      Serial.println(address,HEX);
       lcd.setCursor(0, 3);
      // print message
       lcd.print(address,HEX);
        delay(1000);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
       lcd.setCursor(0, 1);
      // print message
       lcd.print("No I2C devices found.....");
        delay(1000);
       
  }
  else {
    Serial.println("done\n");
    lcd.setCursor(0, 3);
      // print message
       lcd.print("done...............");
        delay(1000);
  delay(5000);
  lcd.clear();
           
 
   } }
///////////////////////////////////////////////////////////////////////////
void student_info()
{
    lcd.clear();  
    Serial.print(".........Smart Weather Station............");
    lcd.setCursor(0, 0);
  // print message
    lcd.print("Smart Weather Station");
    delay(3000);
    lcd.setCursor(0, 1);
  // print message
    lcd.print("..Faizan Ullah..");
   Serial.println("Faizan Ullah Khan (BS Electronics)");
   delay(3000);
 
  lcd.setCursor(0,1);
  // print message
    lcd.print("..Sajid Shah..");
  Serial.println("..Sajid Shah.... (BS Electronics)");
  delay(3000);

   lcd.setCursor(0, 2);
  // print message
    lcd.print("..Zainab..");
  Serial.println("..Zainab.. (BS Electronics)");
  delay(3000);
  lcd.clear();
    }
////////////////////////////////////////////////////////////
void setup()
{
Wire.begin(); // i2c begin
////////////////////Serial Port //////////////////
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
//////////////////////////////////////////////////
pinMode(RainDetecPIN, INPUT);
pinMode(WindSensorPIN, INPUT);
pinMode(AirQltyPIN, INPUT);
pinMode(WindSensorPIN, INPUT);
pinMode(SoilMoistPIN, INPUT);
pinMode(DHTPIN, INPUT);
pinMode(AirTempPIN, INPUT);
pinMode(SoilTempPIN, INPUT);
//////////////////////////////////////////////////////////////////
LastValue = 1;
LastValue = 0;

IsSampleRequired = false;

TimerCount = 0;
Rotations = 0; // Set Rotations to 0 ready for calculations
attachInterrupt(digitalPinToInterrupt(WindSensorPIN), isr_rotation, FALLING);
////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////
  Serial.println("\nI2C Scanner.....");  // initialize LCD
  lcd.init();                           // turn on LCD backlight                      
  lcd.backlight();
 
   i2c_scan();
   lcd.clear();  
   Serial.print(".........Smart Weather Station............");
   lcd.setCursor(0, 0); // print message
   lcd.print("SmartWeather Station");
   Serial.println(".....Smart Weather Station......");
   delay(1000);
   ////////////////////////////////////////////
   lcd.setCursor(0, 1); // print message
   lcd.print("Solar Powered IoT Based");
  Serial.println("..........Solar Powered IoT Based.......");
  delay(3000);
  Serial.print("\n");
  lcd.setCursor(0, 2); // print message
  lcd.print("Suprv:Dr.M Kamran");
  Serial.println("SuperViser: Dr.Muhmmad Kamran (Electronics)");
  delay(3000);
  Serial.print("\n");
  lcd.setCursor(0, 3);
  lcd.print("PhD Electronics"); // print message
  delay(3000);
  //////////////Group Information////////////////
  void student_info();
  delay(3000);
//////////////////////////////////////////////////////////
 lcd.clear();
 delay(1000);
 lcd.setCursor(0, 0); // print message
 lcd.print("SmartWeather Station");

/////////////////////////DHT11 TEST////////////////////////
if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
  lcd.setCursor(0, 1); // print message
  lcd.print("BMP180 not connect");
  while (1) {
    }
  }
Serial.println("DHTxx test!");
lcd.setCursor(0, 2); // print message
lcd.print("DHTxx test...");
dht.begin();
sensors.begin(); // Start the DS18B20 sensor
Serial.println(".......Solar Powered Smart Weather Station......");
Serial.println("AirTemp rainValue RainStat\tSpeed (MPH)\tKnots\tDirection\tStrength");

/////////////////////////////////////////////////////
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
 // timerAlarmWrite(timer, 1000000, true);
  timerAlarmWrite(timer, 500000, true);
  timerAlarmEnable(timer);
////////////////////Thingspeak/////////////////////////////////
 lcd.clear();
 lcd.setCursor(0, 0); // print message
 lcd.print("SmartWeather Station");
 Serial.println("\n ...WIFI setting.......");
 lcd.setCursor(0, 1); // print message
 lcd.print("...WIFI setting...");
 WiFi.mode(WIFI_STA);  
 ThingSpeak.begin(client);  // Initialize ThingSpeak
 Serial.println("\n ..THingsPeak Initalized.......");
 lcd.setCursor(0, 2); // print message
 lcd.print("THingsPeak Initalized.....");
 lcd.clear();
 lcd.setCursor(0, 0); // print message
 lcd.print("SmartWeather Station");
}

void loop()
{
//////////////////////wifi connected//////////////////////

  Serial.println("\n ...WIFI Intialing.......");
  lcd.setCursor(0, 1); // print message
  lcd.print("WIFI.....");
  if ((millis() - lastTime) > timerDelay) {
   
    // Connect or reconnect to WiFi
    if(WiFi.status() != WL_CONNECTED){
      Serial.print("Attempting to connect");
      lcd.setCursor(0, 2); // print message
      lcd.print("trying to connect");
      while(WiFi.status() != WL_CONNECTED){
        WiFi.begin(ssid, password);
        delay(5000);    
      }
      Serial.println("\nConnected.");
      lcd.setCursor(0, 2); // print message
      lcd.print("Connected....");
    }
  }

///---------------------------LUX Values-----------------------------///
//-------------------------- PIN Status Setup ---------------////
  int analogValue = analogRead(LuxValuePIN); // reads the input on analog pin (value between 0 and 4095)
  Serial.print("Analog Value = ");
  Serial.print(analogValue);   // the raw analog reading
  lcd.setCursor(0, 1); // print message
  lcd.print("........Lux Values..........");
  lcd.setCursor(0, 2 ); // print message
  lcd.print(analogValue);

  // We'll have a few threshholds, qualitatively determined
  if (analogValue < 40) {
    Serial.println(" => Dark");
    lcd.setCursor(0, 3 ); // print message
    lcd.print(" => Dark");
  } else if (analogValue < 800) {
    Serial.println(" => Dim");
    lcd.setCursor(0, 3 ); // print message
    lcd.print(" => Dim");
  } else if (analogValue < 2000) {
    Serial.println(" => Light");
    lcd.setCursor(0, 3 ); // print message
    lcd.print("  => Light");
  } else if (analogValue < 3200) {
    Serial.println(" => Bright");
    lcd.setCursor(0, 3 ); // print message
    lcd.print("=> Bright");
  } else {
    Serial.println(" => Very bright");
    lcd.setCursor(0, 3 ); // print message
    lcd.print(" =>Very bright");
  }
  delay(1000);
///------------  Rain detection------------------------------////
  lcd.clear();
  lcd.setCursor(0, 0); // print message
  lcd.print("SmartWeather Station");
  lcd.setCursor(0, 1); // print message
  lcd.print("..Rain Detection..");
  int sensorValue = analogRead(RainDetecPIN);// read the input on analog pin 39:
  Serial.print(sensorValue);
  Serial.print("\t");
  lcd.setCursor(0, 2); // print message
  lcd.print("RainValues:");
  lcd.setCursor(0, 3); // print message
  lcd.print(sensorValue);
 
    if(sensorValue < thresholdValue){
    Serial.print("-It's wet");
     Serial.print("\t");
     lcd.setCursor(7, 3); // print message
     lcd.print("Start...");
      }
  else {
    Serial.print("-It's dry");
    Serial.print("\t");
    lcd.setCursor(7, 3); // print message
    lcd.print("No Rain...");
  }
    delay(1000);
/////--------------------------------------------------------///

///------------------WIND SPEED & DIRECTION------------------------////
getWindDirection();
if(abs(CalDirection - LastValue) > 5) // Only update the display if change greater than 5 degrees.
{
LastValue = CalDirection;
}

if(IsSampleRequired)
{
// convert to mp/h using the formula V=P(2.25/T)
// V = P(2.25/2.5) = P * 0.9
WindSpeed = Rotations * 0.9;
Rotations = 0; // Reset count for next sample
//////////////////////////////////////////////////////////////////////////
IsSampleRequired = false;
//////////////////////////////////////////////////////////////////////////
if(IsSampleRequired)
{
// convert to mp/h using the formula V=P(2.25/T)
// V = P(2.25/2.5) = P * 0.9
WindSpeed = Rotations * 0.9;
Rotations = 0; // Reset count for next sample
//////////////////////////////////////////////////////////////////////////
IsSampleRequired = false;
///////////////////////////////////////////////////////////////////////////////
Serial.print(WindSpeed); Serial.print("\t\t");
Serial.print(getKnots(WindSpeed)); Serial.print("\t");
Serial.print(CalDirection);
getHeading(CalDirection); Serial.print("\t\t");
getWindStrength(WindSpeed);
}
 

  lcd.setCursor(0, 1); // print message
  lcd.print("..WindSpeed..");
  Serial.print(WindSpeed);
  lcd.setCursor(0, 2); // print message
  lcd.print("WindSpeed: ");
  lcd.setCursor(10, 2); // print message
  lcd.print(WindSpeed);
  Serial.print("\t\t");
  Serial.print(getKnots(WindSpeed));
  Serial.print("\t");
  lcd.setCursor(0, 3); // print message
  lcd.print("getKnots:");
  lcd.setCursor(10,3); // print message
  lcd.print(getKnots(WindSpeed));
  Serial.print(CalDirection); //??
  getHeading(CalDirection); ///??
  Serial.print("\t\t");
  getWindStrength(WindSpeed);
}
///----------------DDMB MODULE-(DS18B20,DHT11,MQ135,BMP180)------------------///
///----------------Air Temperature (DS18B20_01)---------------------------- ///
  lcd.clear(); // clear LCD
  lcd.setCursor(0, 0); // print message
  lcd.print("SmartWeather Station");
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);
  float temperatureF = sensors.getTempFByIndex(0);
  Serial.print(temperatureC);
  Serial.print(" ºC");
  Serial.print("\t");
  lcd.setCursor(0, 1); // print message
  lcd.print("AirTempC");
  lcd.setCursor(0, 2); // print message
  lcd.print(temperatureC);
  delay(1000);

///-------------------------------DHT11---------------------------///
// Wait a few seconds between measurements.
   delay(2000);
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit
  float f = dht.readTemperature(true);
 
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index
  // Must send in temp in Fahrenheit!
  float hi = dht.computeHeatIndex(f, h);

  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  lcd.setCursor(6, 1); // print message
  lcd.print("Humidty");
  lcd.setCursor(6, 2); // print message
  lcd.print(h);
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print(f);
  Serial.print(" *F\t");
  Serial.print("Heat index: ");
  Serial.print(hi);
  Serial.println(" *F");
  Serial.println(" \n");
  delay(1000);
 
  ///-----------------AIR Quality (MQ135)-------------////
  int MQ135_data = analogRead(AirQltyPIN);
  lcd.setCursor(14, 1); // print message
  lcd.print("AirQlty");
  lcd.setCursor(14, 2); // print message
  lcd.print(MQ135_data);
 delay(1000);
if(MQ135_data < MQ135_THRESHOLD_1)
{
Serial.print("Fresh Air: ");
} else {
Serial.print("Poor Air: ");
}
Serial.print(MQ135_data); // analog data
Serial.println(" PPM"); // Unit = part per million
  delay(1000);
 
 ///--------------BMP180-------------------------------------------------////
  Serial.print("Temperature = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");
   
  Serial.print("Pressure = ");
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");
  lcd.setCursor(10, 2); // print message
  lcd.print("AirPr");
  lcd.setCursor(10, 3); // print message
  lcd.print(bmp.readPressure());
  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  Serial.print("Altitude = ");
  Serial.print(bmp.readAltitude());
  Serial.println(" meters");

  Serial.print("Pressure at sealevel (calculated) = ");
  Serial.print(bmp.readSealevelPressure());
  Serial.println(" Pa");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
  Serial.print("Real altitude = ");
  Serial.print(bmp.readAltitude(102000));
  Serial.println(" meters");
  Serial.println();
  delay(1000);

  ///////////////////////////----SOIL MOISTURE MASTER---///////////////////////////
  lcd.clear(); // clear LCD
  lcd.setCursor(0, 0); // print message
  lcd.print("SmartWeather Station");
  int soilmoist();
  SoilMoistr = msvalue ;
  Serial.print("  ");
  delay(250);
  float soilTemp();
  float SoilTmp1 = SoilTmp;
  Serial.print("  ");
  delay(250);
  float SnowDepth();
  Serial.print("  ");
  float distanceCm1 = distanceCm;
  delay(250);
  Serial.print("\n");
 /////////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////////TFT_UNO_SERIAL_DISPLAY///////////////////////////////////////
 Serial.println("------TFT_DISPLAY_SPI_UNO_SERIAL-----");
 Serial.println("  Serial Data_PACKET Sending......");
 delay(1000);
 Serial2.print(",");
 Serial2.print(temperatureC);               //01- AIR TEMP
 Serial2.print(",");
 Serial2.print(h);                           //02- AIR Humidty
 Serial2.print(",");
 Serial2.print(bmp.readPressure());         //03- Barometric pressure
 Serial2.print(",");                
 Serial2.print(WindSpeed);                  //04- WIND SPEED             
 Serial2.print(",");
 Serial2.print(CalDirection);                 //05- WIND DIRECTION      
 delay(1000);
Serial.println("  Serial Data_PACKET DONE......");

////////////////////thingspeak Values write ////////////////////////////////////////////////
//                      Channel 1 Updates :                                             ////
// 1. Air Temperature 2. Air Humidity     3. Barometric Pressure, 4.WindSpeed          ////
// 5. Wind Direction  6. Heat Index Value 7. Air Quality          8. Lux Values       ////                
/////////////////////////////////////////////////////////////////////////////////////////
/*                   // LCD 20x4 Display
    lcd.clear();
    lcd.setCursor(0, 0); // print message 1
    lcd.print("SmartWeather Station");
    lcd.setCursor(0, 1); // print message
    lcd.print("ThingSpeak Channel 1.");
    lcd.setCursor(0, 2); // print message 1
    lcd.print("AT ");
    lcd.setCursor(0, 3); // print message 1
    lcd.print(temperatureC);
    lcd.setCursor(4, 2); // print message 2
    lcd.print("AH ");
    lcd.setCursor(6, 3); // print message 1
    lcd.print(h);
    lcd.setCursor(7, 2); // print message 3
    lcd.print("BP ");
    lcd.setCursor(11, 3); // print message 1
    lcd.print(MQ135_data);
    lcd.setCursor(11, 2); // print message
    lcd.print("WS ");
   
    lcd.setCursor(15, 2); // print message
    lcd.print("WD ");
   
    lcd.setCursor(18, 2); // print message
    lcd.print("AQ ");
   
    lcd.setCursor(20, 2); // print message
    lcd.print("LU");
   

*/

// set the fields with the values
    ThingSpeak.setField(1, temperatureC);         // DS18B20_Air Temperature
    ThingSpeak.setField(2, h);                    //DHT11 Humidity:
    ThingSpeak.setField(3,bmp.readPressure());     //BMP180 Barometric Pressure
    ThingSpeak.setField(4,MQ135_data);            // MQ135 Air Quality
    ThingSpeak.setField(5,WindSpeed);            //WindSpeed
    ThingSpeak.setField(6,CalDirection );       //Wind Direction
    ThingSpeak.setField(7,SoilMoistr);         // Soil Mositure values
    ThingSpeak.setField(8, analogValue);      // LDR as Lux Meter
   
    // Write to ThingSpeak. There are up to 8 fields in a channel, allowing you to store up to 8 different
    // pieces of information in a channel.  Here, we write to field 1.
    int x = ThingSpeak.writeFields(myChannelNumber1, myWriteAPIKey1);

    if(x == 200){
      Serial.println("Channel 1 update successful.");
    }
    else{
      Serial.println("Problem updating channel. HTTP error code " + String(x));
    }
    lastTime = millis();
 
  delay(10000);
  //////////////////////////////////////////////////////////////////////////////////////////
  //                             Channel 2 Updated Values                              ////
  //      1. Rain Detection Values 2. Soil Moisture Values                                 ////    
  //      3.Soil Temprature 4. SnowDepth Values:                                          ////
 ///////////////////////////////////////////////////////////////////////////////////////
    Serial.println("Channel 2 data upload.......");
  // set the fields with the values
    ThingSpeak.setField(1,sensorValue);        //  Rain Detection Values
    ThingSpeak.setField(2,SoilMoistr );       // Soil Mositure Values
    ThingSpeak.setField(3,SoilTmp1);       // Soil Temperature Values
    ThingSpeak.setField(4, distanceCm1);      // Snow Depth Values
  //  ThingSpeak.setField(5, WindDirect);   // Wind Direction
  //  ThingSpeak.setField(6, RainGauge);    // Rain Gauage
  //  ThingSpeak.setField(7, );   //Microphone based
  //  ThingSpeak.setField(8, );   //MQ135
   
    // Write to ThingSpeak. There are up to 8 fields in a channel, allowing you to store up to 8 different
    // pieces of information in a channel.  Here, we write to field 1.
    int x1 = ThingSpeak.writeFields(myChannelNumber2, myWriteAPIKey2);

    if(x1 == 200){
      Serial.println("Channel 2 update successful.");
    }
    else{
      Serial.println("Problem updating channel 2. HTTP error code " + String(x1));
    }
    lastTime = millis();
    delay(10000);
 
 
  }
/////////////////////////---VOID LOOP END--- ///////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
// This is the function that the interrupt calls to increment the rotation count
void isr_rotation()
{

if((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact.
Rotations++;
ContactBounceTime = millis();
}
}
float getKnots(float speed) // Convert MPH to Knots
{
return speed * 0.868976;
}

void getWindDirection() // Get Wind Direction
{
VaneValue = analogRead(WindVanePIN);
Direction = map(VaneValue, 0, 1023, 0, 359);
CalDirection = Direction + VaneOffset;
lcd.setCursor(0, 1); // print message
lcd.print("WindDir..");
lcd.setCursor(10, 1); // print message
lcd.print(CalDirection);
if(CalDirection > 360)
CalDirection = CalDirection - 360;

if(CalDirection < 0)
CalDirection = CalDirection + 360;
}

void getHeading(int direction) // Converts compass direction to heading
{

if(direction < 22){
Serial.print(" N");
lcd.setCursor(0, 3); // print message
lcd.print("Direction: ");
lcd.setCursor(7, 3); // print message
lcd.print(" : North");}
else if (direction < 67){
Serial.print(" NE");
lcd.setCursor(0, 3); // print message
lcd.print("Direction:");
lcd.setCursor(7, 3); // print message
lcd.print("NorthEast");}
else if (direction < 112){
Serial.print(" E");
lcd.setCursor(0, 3); // print message
lcd.print("Direction:");
lcd.setCursor(7, 3); // print message
lcd.print(" East");}
else if (direction < 157){
Serial.print(" SE");
lcd.setCursor(0, 3); // print message
lcd.print("Direction:");
lcd.setCursor(7, 3); // print message
lcd.print("SouthEast");}
else if (direction < 212){
Serial.print(" S");
lcd.setCursor(0, 3); // print message
lcd.print("Direction:");
lcd.setCursor(7, 3); // print message
lcd.print("South");}
else if (direction < 247){
Serial.print(" SW");
lcd.setCursor(0, 3); // print message
lcd.print("Direction:");
lcd.setCursor(7, 3); // print message
lcd.print("SouthWest");}
else if (direction < 292){
Serial.print(" W");
lcd.setCursor(0, 3); // print message
lcd.print("Direction:");
lcd.setCursor(7, 3); // print message
lcd.print("West");}
else if (direction < 337){
Serial.print(" NW");
lcd.setCursor(0, 3); // print message
lcd.print("Direction:");
lcd.setCursor(7, 3); // print message
lcd.print("NorthWest");}
else{
Serial.print(" N");
lcd.setCursor(0, 3); // print message
lcd.print("Direction:");
lcd.setCursor(7, 3); // print message
lcd.print(" North");
}}
//////////////////////////////////////////////////
// converts wind speed to wind strength
void getWindStrength(float speed) {

if(speed < 2)
Serial.println("Calm");
else if(speed >= 2 && speed < 4)
Serial.println("Light Air");
else if(speed >= 4 && speed < 8)
Serial.println("Light Breeze");
else if(speed >= 8 && speed < 13)
Serial.println("Gentle Breeze");
else if(speed >= 13 && speed < 18)
Serial.println("Moderate Breeze");
else if(speed >= 18 && speed < 25)
Serial.println("Fresh Breeze");
else if(speed >= 25 && speed < 31)
Serial.println("Strong Breeze");
else if(speed >= 31 && speed < 39)
Serial.println("Near Gale");
else
Serial.println("RUN");
}

////////////////////////////////////////////
////////////////SoilMositure///////////////////
int soilmoist(int msvlaue)
{
 
 // value= analogRead(sensor_pin);
//  value = map(value,550,0,0,100);
  msvalue = analogRead(msensor);
 // Serial.println(msvalue);
   Serial.print(msvalue);
   lcd.setCursor(1, 3);
  // print message
   lcd.print(msvalue);
   
  return (msvlaue);
  //return msvalue
  delay(250);
  }

////////////////////////////////////SoilTemperature/////////////////////
 float soilTemp()
 {
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);
  SoilTmp = temperatureC;
  Serial.print(temperatureC);
  Serial.println("ºC");
  lcd.setCursor(8,3);
  // print message
  lcd.print(SoilTmp);
   return (SoilTmp);
   delay(250);
 }

//////////////////SnowDepth////////////////////////////////

float SnowDepth(float distanceCm)
 {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
 
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;
 
  // Convert to inches
  distanceInch = distanceCm * CM_TO_INCH;
 
  // Prints the distance in the Serial Monitor
   Serial.print("SnowDepth(cm): ");
   Serial.println(distanceCm);
   lcd.setCursor(15, 3); // print message
   lcd.print(distanceCm);
   Serial.print("Distance (inch): ");
   Serial.println(distanceInch);
   return(distanceCm);
   delay(250);
}
/////////////////////////////////Project END///////////////////////////////////////////////////////////////
