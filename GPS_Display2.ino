#include <ArduinoJson.h>
#include <Arduino_JSON.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// what's the name of the hardware serial port?
#define GPSSerial Serial1
#define SEALEVELPRESSURE_HPA (1013.25)

const int buttonPin = 33;
int buttonState = 0;

Adafruit_BME680 bme;

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();
void setup()
{
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
  pinMode(buttonPin, INPUT);
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

}

void loop() {
  float temperature;
  float humidity;
  float pressure;
  
  int latte;
  int lotte;
  String la;
  String lla;
  String lati;
  String llo;
  String lo;
  String logi;
  String Alt;
  String Ti;
  String Da;
  String Day;
  String Month;
  String Year;
  String ho;
  String mi;
  String sec;

  String F;
  String q;
 
  String Speed;
  String Sats;
  JSONVar sensorArray;
  
  display.clearDisplay();

  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  char c = GPS.read();

  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    //Serial.print("\nTime: ");
    ho = GPS.hour;
    mi = GPS.minute;
    sec = GPS.seconds;
    if (GPS.hour < 10) { ho = "0" + ho; }
    //Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { mi = "0" + mi; }//Serial.print('0');
    //Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { sec = "0" + sec; }
    //Serial.print(GPS.seconds, DEC); Serial.print('.');
    ///if (GPS.milliseconds < 10) {
      //Serial.print("00");
    //} else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      //Serial.print("0");
    //}
    //Serial.println(GPS.milliseconds);
    Day = GPS.day;
    Month = GPS.month;
    Year = GPS.year;
    //Serial.print(GPS.day, DEC); Serial.print('/');
    //Serial.print(GPS.month, DEC); Serial.print("/20");
    //Serial.println(GPS.year, DEC);
    display.print(Day + "/");
    display.print(Month + "/20");
    display.println(Year);
    //Serial.print("Fix: "); Serial.print((int)GPS.fix);
    //Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    F = GPS.fix;
    q = GPS.fixquality;
    display.print("Fix: " + F);
    display.println(" Qual: " + q);
    if (GPS.fix) {
      
      display.clearDisplay();
      display.setCursor(0,0);
      display.print(Day + "/");
      display.print(Month + "/20");
      display.println(Year);
      //Serial.print("Location: ");
      //Serial.print(GPS.latitude, 5); Serial.print(GPS.lat);
      //Serial.print(", ");
      //Serial.print(GPS.longitude, 5); Serial.println(GPS.lon);
      lati = GPS.latitude;
      logi = GPS.longitude;
      la = GPS.lat;
      lo = GPS.lon;
      Sats = GPS.satellites;
      Alt = GPS.altitude;

      display.print(lati); display.print(la + ", ");
      display.print(logi); display.println(lo);
      
      //Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      //Serial.print("Angle: "); Serial.println(GPS.angle);
      //Serial.print("Altitude: "); Serial.println(GPS.altitude);
      //Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      
      //Serial.println("Temp: " + String(temperature));
      //Serial.println("Hum: " + String(humidity));

      lla = lati + la;
      llo = logi + lo;
      Ti = ho + ":" + mi + ":" + sec;
      Da = Day + "/" + Month + "/20" + Year;

      if (! bme.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
      }
      temperature = bme.temperature;
      humidity = bme.humidity;
      pressure = bme.pressure/1000.0;

      display.print("T:" + String(int(temperature)) + " H:" + String(int(humidity)));
      display.println(" P:" + String(int(pressure)));
      
      
      sensorArray[0] = String(temperature);
      sensorArray[1] = String(humidity);
      sensorArray[2] = String(pressure);
      sensorArray[3] = lla;
      sensorArray[4] = llo;
      sensorArray[5] = Ti;
      sensorArray[6] = Da;
      
      Serial.println(sensorArray);
      
    }
    
    display.display();
    buttonState = digitalRead(buttonPin);
      Serial.println(buttonState);
      if (buttonState == HIGH) 
      {
        Serial.println("Lora sent");
        display.clearDisplay();
        display.println("LoRa Sent!");
        display.display();
      }
    delay(1000);
  }
  

 }
