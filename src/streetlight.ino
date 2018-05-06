#include <U8g2lib.h>
#include <avr/pgmspace.h>
#include <AltSoftSerial.h>
#include "RunningAverage.h"
// #include <HM12.h>

AltSoftSerial BlueBee; // permanently pins 8,9

/* Constructor */
U8G2_SSD1327_SEEED_96X96_1_HW_I2C u8g2(U8G2_R0);
// U8G2_SSD1327_SEEED_96X96_1_SW_I2C u8g2(U8G2_R0, A3, A2);
// U8G2_SSD1327_SEEED_96X96_F_HW_I2C u8g2(U8G2_R0);

// unsigned int delayTime = 1000;
const int numReadings = 1;
const long interval = 5000;            // timeout for dust sensor
const long automatic_interval = 6000; // timeout for automatic mode
RunningAverage myRA(100);

// SoftwareSerial Dusty(8,9); // RX, TX
byte data[6];   // for incoming serial data
byte readbuffer[2];
int rb = 0;
int heartRate = 0;
int heartRateIndex[numReadings];          // the readings from the analog input
int readIndex = 0;                        // the index of the current reading
int total = 0;                            // the running total
int averageHeartRate = 0;                 // the average
unsigned long previousMillis = 0;         // will store last time LED was updated
unsigned long currentMillis = 0;
int j = 0, k = 0;
int step = 50;

// dust sensing
int measurePin = A1;
int ledPower = A0;
int voltage_threshold = 2;
unsigned int samplingTime = 280;
unsigned int deltaTime = 40;
unsigned int sleepTime = 9680;
unsigned int delayTime = 5000;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;
bool dust_indicator = false;
bool status = false;
bool automaticMode = false;
int mistState = 0;
unsigned long previousMillis2 = automatic_interval;
float previous_brightness = 0;
float brightness = 0;

// everything else
int ALSPin = A4;
int fanPin = 4;
int button1 = 12;
int button2 = 7;
int button3 = 11;
int mist = 3;
int warmLED = 6, coolLED = 5;
int btnAutomatic = 0, btnFan = 0, btnMist = 0;
int brightness_multiplier = 5; // base brightness is level 5. Min = 1, max = 10

void setup() {
  u8g2.begin();
  do {
      u8g2.setFont(u8g2_font_logisoso78_tn);
      u8g2.setCursor(0, 87);
      u8g2.print(averageHeartRate);
    } while ( u8g2.nextPage() );
  BlueBee.begin(9600); // set baudrate according to BlueBee 4.0 baudrate
  BlueBee.println("BlueBee ready");
  pinMode(ledPower, OUTPUT); // Initialize infrared LED in dust sensor
  pinMode(fanPin, OUTPUT);
  pinMode(warmLED, OUTPUT);
  pinMode(coolLED, OUTPUT);
  pinMode(mist, OUTPUT);
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(ALSPin, INPUT);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    heartRateIndex[thisReading] = 0; // initialize all the readings to 0:
  }
  Serial.begin(9600);
  Brighten(warmLED, 0, 25 * brightness_multiplier, 5);
  status = false;
  myRA.clear(); // explicitly start clean
  previous_brightness = 25 * brightness_multiplier;
}

void loop(){
  // ReceiveHeartData();
  // DisplayInfo_Screen();
  // analogWrite(coolLED, 255);
  // analogWrite(warmLED, 255);
  // Mist_On();
  CalculateDust();
  DisplayInfo_Dust();
  Read_Buttons();
  // Brightness_Automatic();
  // Streetlight();
}

void ReceiveHeartData(){
  while (BlueBee.available() > 0) {
    previousMillis = currentMillis;
    heartRate = BlueBee.read(); // read the incoming byte:
    BlueBee.print("Heart-rate: ");
    BlueBee.println(heartRate);
    total = total - heartRateIndex[readIndex]; // subtract the last reading:
    heartRateIndex[readIndex] = heartRate; // read from ReceiveBluetoothData()
    total = total + heartRateIndex[readIndex]; // add the reading to the total:
    readIndex = readIndex + 1; // advance to the next position in the array:
    if (readIndex >= numReadings) {   // if we're at the end of the array...
    averageHeartRate = total / numReadings;   // calculate the average:
    BlueBee.print("Average heart-rate: ");
    BlueBee.println(averageHeartRate);
    BlueBee.println("*****************************");
    readIndex = 0;     // ...wrap around to the beginning:
    }
  delay(1);        // delay in between reads for stability
  break;
  }
  // heart rate range from 50 ---> 135
  if(averageHeartRate < 60){averageHeartRate = 60;}
  if(averageHeartRate > 100){averageHeartRate = 100;}
  j = (averageHeartRate - 60) * 6;
  k = (100 - averageHeartRate) * 6;
  analogWrite(warmLED, j);
  analogWrite(coolLED, k);
  delay(1);
}

void CalculateDust(){
  digitalWrite(ledPower, LOW);
  delayMicroseconds(samplingTime); // pulse infrared LED
  voMeasured = analogRead(measurePin);
  delayMicroseconds(deltaTime); // slight delay to preserve data integrity
  digitalWrite(ledPower, HIGH);
  delayMicroseconds(sleepTime);
  calcVoltage = voMeasured * (5.0 / 1024);
  dustDensity = 0.17 * calcVoltage - 0.1;
  if (dustDensity < 0)
  {
    dustDensity = 0.00;
  }
}

void DisplayInfo_Dust(){
  if(calcVoltage > voltage_threshold){
    currentMillis = millis();
    dust_indicator = true;
    Serial.println("WARNING! FOG DETECTED");
  }
  if (millis() - currentMillis >= interval){ // if no dust sensed for Tinterval, switch off
    dust_indicator = false;
  }
  // Serial.print("Raw Signal (0-1023): ");
  // Serial.print(voMeasured);
  // Serial.print(" - Voltage: ");
  // Serial.print(calcVoltage);
  // Serial.print(" - Dust Density: ");
  // Serial.println(dustDensity);
}

void DisplayInfo_Screen(){
  u8g2.firstPage(); // display on OLED
  do {
    if(averageHeartRate < 100){ // 3 numbers or more
      u8g2.setFont(u8g2_font_logisoso78_tn);
      u8g2.setCursor(0, 87);
    }
    else{ // 2 numbers or less
      u8g2.setFont(u8g2_font_logisoso50_tn);
      u8g2.setCursor(0, 73);
    }
    u8g2.setFontDirection(0); // top to bottom
    u8g2.print(averageHeartRate);
  } while ( u8g2.nextPage() );
  delay(1);
}

// void Draw_Screensaver(){
//   // u8g2.clearDisplay();
//   u8g2.firstPage(); // display on OLED
//   do{
//     u8g2.drawXBM(0,0,u8g_logo_width, u8g_logo_height, u8g_logo_bits);
//     delay(1);
//   } while( u8g2.nextPage());
// }

void Fan_On(){
  digitalWrite(fanPin, HIGH);
}

void Fan_Off(){
  digitalWrite(fanPin, LOW);
}

void Mist_On(){
  digitalWrite(mist, HIGH);
}

void Mist_Off(){
  digitalWrite(mist, LOW);
}

void Read_Buttons(){
  btnAutomatic = digitalRead(button1);
  btnMist = digitalRead(button2);
  btnFan = digitalRead(button3);

  // if (btnAutomatic == LOW && automaticMode == false){ // automatic mode
  if (btnMist == HIGH && btnFan == HIGH && automaticMode == false) { //
    Fan_Off();
    Mist_Off();
    mistState = 0;
    previousMillis2 = millis() - automatic_interval;
    automaticMode = true;
  }

  if (automaticMode == true && btnAutomatic == HIGH){ //  autobrightness
    Brightness_Automatic();
  }
  if (automaticMode == true && btnAutomatic == LOW){ //  auto mist
    Streetlight_Automatic();
  }

  if (btnMist == LOW || btnFan == LOW && automaticMode == true) { // manual mode
    Fan_Off();
    Mist_Off();
    automaticMode = false;
  }
  if(automaticMode == false){
    Streetlight_Manual();
    if (btnMist == LOW){
      Mist_On();
    } else { Mist_Off(); }
    if (btnFan == LOW){
      Fan_On();
    } else { Fan_Off(); }
  }
}

void Streetlight_Manual(){
  if (dust_indicator == true && status == false){
    Brighten(coolLED, 0, 255, 5);
    // analogWrite(warmLED, 0);
    // Dim(warmLED, 150, 50, 5);
  }
  if (dust_indicator == false && status == true){
    Dim(coolLED, 255, 0, 5);
    // Brighten(warmLED, 0, 100, 5);
    // Brighten(warmLED, 50, 150, 5);
  }
}

void Streetlight_Automatic(){
  if (dust_indicator == true && status == false){
    Brighten(coolLED, 0, 255, 5);
    // analogWrite(warmLED, 0);
    // Dim(warmLED, 150, 50, 5);
  }
  if (dust_indicator == false && status == true){
    Dim(coolLED, 255, 0, 5);
    // Brighten(warmLED, 0, 100, 5);
    // Brighten(warmLED, 50, 150, 5);
  }
  unsigned long currentMillis2 = millis();
  if (currentMillis2 - previousMillis2 >= automatic_interval) {
    previousMillis2 = currentMillis2;
    if (mistState == 0) {
      mistState = 1;
      Mist_On();
      Fan_Off();
    }
    else if (mistState == 1) {
      mistState = 2;
      Mist_Off();
    }
    else  {
      mistState = 0;
      Fan_On();
    }
  }
}

bool Brighten(int _color, int _z, int _max, int _speed){ // LED string, start, end, speed
  for(_z; _z <= _max; _z++){
    if(_z < 0){_z = 0;}
    if(_z > 255){_z = 255;}
    analogWrite(_color, _z);
    delay(_speed);
  }
  return status = true;
}

bool Dim(int _color, int _z, int _min, int _speed){ // LED string, start, end, speed
  for(_z; _z >= _min; _z--){
    if(_z < 0){_z = 0;}
    if(_z > 255){_z = 255;}
    analogWrite(_color, _z);
    delay(_speed);
  }
  return status = false;
}

void Brightness_Automatic(){
  mistState = 0;
  previousMillis2 = millis() - automatic_interval;
  Mist_Off();
  Fan_Off();
  float readBrightness = analogRead(ALSPin);
  myRA.addValue(readBrightness);
  delay(10);

  if (myRA.getAverage() < 100) {
    brightness_multiplier = 10;
    brightness = 25 * brightness_multiplier;
  }
  else if (myRA.getAverage() >= 150 && myRA.getAverage() < 300) {
    brightness_multiplier = 8;
    brightness = 25 * brightness_multiplier;

  }
  else if (myRA.getAverage() >= 350 && myRA.getAverage() < 500) {
    brightness_multiplier = 6;
    brightness = 25 * brightness_multiplier;

  }
  else if (myRA.getAverage() >= 550 && myRA.getAverage() < 700) {
    brightness_multiplier = 4;
    brightness = 25 * brightness_multiplier;

  }
  else if (myRA.getAverage() >= 750 && myRA.getAverage() < 900) {
    brightness_multiplier = 2;
    brightness = 25 * brightness_multiplier;

  }
  else if (myRA.getAverage() >= 950) {
    brightness_multiplier = 0;
    // brightness = 25 * brightness_multiplier;
    brightness = 0;

  }
  while (brightness < previous_brightness){
    Dim (warmLED, previous_brightness, brightness, 15);
    if(status == false){ // loop finished
      break;
    }
  }
  while (brightness > previous_brightness){
    Brighten(warmLED, previous_brightness, brightness, 15);
    if(status == true){ // loop finished
      break;
    }
  }
  previous_brightness = brightness;
  // analogWrite(warmLED, 25 * brightness_multiplier);
}
