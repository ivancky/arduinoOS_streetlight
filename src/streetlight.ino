#include <avr/pgmspace.h>
#include "RunningAverage.h"

const int numReadings = 1;
const long interval = 5000;            // timeout for dust sensor
const long automatic_interval = 6000; // timeout for automatic mode
RunningAverage myRA(100);

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
bool dust_indicator_flag = false;
bool brightened_flag = false;
bool faded_flag = false;
bool automaticMistMode = false;
int mistState = 0;
unsigned long previousMillis2 = automatic_interval;
unsigned long currentMillis = 0;
int previous_brightness = 0;
int brightness = 0;

// everything else
int ALSPin = A4;
int fanPin = 4;
int button1 = 12;
int button2 = 7;
int button3 = 11;
int mist = 3;
int warmLED_1 = 6, coolLED_1 = 5;
int btnAutomatic = 0, btnFan = 0, btnMist = 0;
int brightness_multiplier = 5; // base brightness is level 5. Min = 1, max = 10
int warmLED = 0, coolLED = 0;
int btnLED = 0;

void setup() {
  // BlueBee.begin(9600); // set baudrate according to BlueBee 4.0 baudrate
  // BlueBee.println("BlueBee ready");
  pinMode(ledPower, OUTPUT); // Initialize infrared LED in dust sensor
  pinMode(fanPin, OUTPUT);
  pinMode(warmLED_1, OUTPUT);
  pinMode(coolLED_1, OUTPUT);
  pinMode(mist, OUTPUT);
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(ALSPin, INPUT);
  // Serial.begin(9600);
  previous_brightness = 25 * brightness_multiplier;
  selectLEDs();
  Brighten(warmLED, 0, previous_brightness, 5);
  // brightened_flag = false;
  myRA.clear(); // explicitly start clean
}

void loop(){
  selectLEDs();
  calculateDust();
  displayInfo_Dust();
  readAmbientBrightness();
  adjustBrightness_Ambient();
  selectMode();
}

void selectLEDs(){
  btnLED = digitalRead(button3);
  if(btnLED == LOW){
    analogWrite(warmLED_1, 0);
    delay(10);
    warmLED = coolLED_1;
  }else{
    analogWrite(coolLED_1, 0);
    delay(10);
    warmLED = warmLED_1;
  }
}
void selectMode(){
  btnAutomatic = digitalRead(button1);
  if(btnAutomatic == LOW && automaticMistMode == false){
    automaticMistMode = true;
    Fan_Off();
    Mist_Off();
    previousMillis2 = millis() - automatic_interval;
    mistState = 0;
  }
  if(btnAutomatic == HIGH && automaticMistMode == true){
    automaticMistMode = false;
    Fan_Off();
    Mist_Off();
    previousMillis2 = millis() - automatic_interval;
    mistState = 0;
  }
  if (automaticMistMode == false){ //  autobrightness
    Streetlight_Manual();
  }
  if (automaticMistMode == true){ //  auto mist
    Streetlight_Automatic();
  }
}
void Streetlight_Manual(){
  btnMist = digitalRead(button2);
  btnFan = HIGH;
  adjustLight_Fog();
  if (btnMist == LOW){
    Mist_On();
  } else { Mist_Off(); }
  if (btnFan == LOW){
    Fan_On();
  } else { Fan_Off(); }
}
void Streetlight_Automatic(){
  adjustLight_Fog();
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
int readAmbientBrightness(){
  float readBrightness = analogRead(ALSPin);
  myRA.addValue(readBrightness);
  delay(10);
  if (myRA.getAverage() < 100) {
    brightness_multiplier = 10;
  }
  else if (myRA.getAverage() >= 150 && myRA.getAverage() < 300) {
    brightness_multiplier = 8;
  }
  else if (myRA.getAverage() >= 350 && myRA.getAverage() < 500) {
    brightness_multiplier = 6;
  }
  else if (myRA.getAverage() >= 550 && myRA.getAverage() < 700) {
    brightness_multiplier = 4;
  }
  else if (myRA.getAverage() >= 750 && myRA.getAverage() < 900) {
    brightness_multiplier = 2;
  }
  else if (myRA.getAverage() >= 950) {
    brightness_multiplier = 0;
  }
  brightness = 25 * brightness_multiplier;
  return brightness;
}
void adjustBrightness_Ambient(){
  if(dust_indicator_flag == true){
    while (brightness < previous_brightness){
      Dim (warmLED, previous_brightness, brightness, 10);
      if(brightened_flag == false){ // loop finished
        break;
      }
    }
    while (brightness > previous_brightness){
      Brighten(warmLED, previous_brightness, brightness, 10);
      if(brightened_flag == true){ // loop finished
        break;
      }
    }
    previous_brightness = brightness;
  }
  else if(dust_indicator_flag == false){
    while (brightness < previous_brightness){
      Dim (warmLED, previous_brightness, brightness, 10);
      if(brightened_flag == false){ // dimmed flag
        break;
      }
    }
    while (brightness > previous_brightness){
      Brighten(warmLED, previous_brightness, brightness, 10);
      if(brightened_flag == true){ // brightened flag
        break;
      }
    }
    previous_brightness = brightness;
  }
}
void adjustLight_Fog(){
  if (dust_indicator_flag == true && faded_flag == false){
    // FadeInOut(coolLED_1, warmLED_1, 0, brightness, 10);
    Brighten(warmLED, 0, 255, 10);
    faded_flag = true;
  }
  if (dust_indicator_flag == false && faded_flag == true){
    // FadeInOut(warmLED_1, coolLED_1, 0, brightness, 10);
    Dim(warmLED, 255, 0, 10);
    faded_flag = false;
  }
}
void calculateDust(){
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
void displayInfo_Dust(){
  if(calcVoltage > voltage_threshold){
    currentMillis = millis();
    dust_indicator_flag = true;
    // Serial.println("WARNING! FOG DETECTED");
  }
  if (millis() - currentMillis >= interval){ // if no dust sensed for Tinterval, switch off
    dust_indicator_flag = false;
  }
  // Serial.print("Raw Signal (0-1023): ");
  // Serial.print(voMeasured);
  // Serial.print(" - Voltage: ");
  // Serial.print(calcVoltage);
  // Serial.print(" - Dust Density: ");
  // Serial.println(dustDensity);
}

void FadeInOut(int _color1, int _color2, int _z, int _max, int _speed){
  for(_z; _z <= _max; _z++){
    if(_z < 0){_z = 0;}
    if(_z > 255){_z = 255;}
    analogWrite(_color1, _z);
    analogWrite(_color2, _max - _z);
    delay(_speed);
  }
}
bool Brighten(int _color, int _z, int _max, int _speed){ // LED string, start, end, speed
  for(_z; _z <= _max; _z++){
    if(_z < 0){_z = 0;}
    if(_z > 255){_z = 255;}
    analogWrite(_color, _z);
    delay(_speed);
  }
  return brightened_flag = true;
}
bool Dim(int _color, int _z, int _min, int _speed){ // LED string, start, end, speed
  for(_z; _z >= _min; _z--){
    if(_z < 0){_z = 0;}
    if(_z > 255){_z = 255;}
    analogWrite(_color, _z);
    delay(_speed);
  }
  return brightened_flag = false;
}
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
