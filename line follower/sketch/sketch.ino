#include <LiquidCrystal_I2C.h>
#include "Adafruit_TCS34725.h"
#define MOTOR_LEFT 7
#define MOTOR_RIGHT 2
#define ena_a 9
#define ena_b 10
#define SENSOR_1 A0
#define SENSOR_2 A1
#define SENSOR_3 A2
#define SENSOR_4 A3
#define SENSOR_5 A6
#define echoPin    11 //pwm pin! 
#define trigPin     12//digital/pwm pin!
//LiquidCrystal_I2C lcd(0x27,16,2);

bool ON;bool forward=false;
uint8_t motorleft;
uint8_t motorright;
 int sensor1;
  int sensor2;
  int sensor3;
  int sensor4;
  int sensor5;
char color='N';
char prev_color='N';
uint16_t prev_sensor3,prev_sensor1,prev_sensor5;
int RGB[3]={};
int skip;
bool stopB=false;
int stopStartTime;
int distinctRGB[15][3] = {{27,14,8}, {75, 51, 19}, {24, 19, 9}, {27,18 ,16}, {38, 36, 27},{21,14,7},{85,69,42},{54,43,26},{47,37,22}};
//char distinctColors[5] = {'r' , 'y',   'g',    'p',   'b'};
//int distinctRGB[7][3] = {{138,40,18}, {244, 218, 144}, {52, 94, 66}, {127,0 ,190}, {36, 32, 69},{255,255,255},{0,0,0}};
char distinctColors[15] = {'r' , 'y',   'g',    'p',   'b' ,'k','w','M','M'};
                        //red  yellow  green purple  blue
                                    //include the sensor library
 
 #define commonAnode false 

 
byte gammatable[256];                                             // our RGB -> eye-recognized gamma color
 
                                                                  //Create an instance of the TCS34725 Sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);


void setup() {
  // put your setup code here, to run once:
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;
      
    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;      
    }
    }
  pinMode(MOTOR_LEFT, OUTPUT);
  pinMode(MOTOR_RIGHT, OUTPUT);
  pinMode(ena_a,OUTPUT);
  pinMode(ena_b, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);
  pinMode(SENSOR_3, INPUT);
  pinMode(SENSOR_4, INPUT);
  pinMode(SENSOR_5, INPUT);
  pinMode(4, INPUT_PULLUP);
  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  Serial.begin(9600);
  //lcd.init();
   //lcd.backlight();
}

void loop() {
  if(digitalRead(4)==1){
    ON=true;
  }else {
    ON=false;
  }
  // put your main code here, to run repeatedly:
  while (!ON){
  //setting S0 and S1 to high for 100% power
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float distance = (pulseIn(echoPin, HIGH)/2) / 29.1;
  if(distance<=10){
    analogWrite(ena_a, 0);
    analogWrite(ena_b, 0);
    delay(10000);
    analogWrite(ena_a, motorright);
    analogWrite(ena_b, motorleft);
  }
  //read color in RGB
  readcolor();
  // seeing the colsest color 
  color=what_color(distinctRGB, distinctColors, RGB);
  Serial.print(color);
  Serial.print(",");
  switch (color) {
    case 'y':analogWrite(3,239);analogWrite(5,216);analogWrite(6,7);if(prev_color=='y'||prev_color=='N'){prev_color=color;}/*lcd.clear();lcd.print("yellow");
  lcd.setCursor(0, 1)*/;break;
    case 'g':analogWrite(3,0);analogWrite(5,86);analogWrite(6,27);if(prev_color=='g'||prev_color=='N'){prev_color=color;}/*lcd.clear();lcd.print("green");
  lcd.setCursor(0, 1);*/break;
    case 'b': analogWrite(ena_a,0);
              analogWrite(ena_b,0);
              skip=0;
    // If we're in the stop period
    if (millis() - stopStartTime >= 6000) { // If the delay period has elapsed
      stopB = false; // Stop the stop period
    } else { 
              
      stopB = true; // Start the stop period // Record the start time of the stop period
  }
  
  if (!stopB) { 
  delay(5000);
  stopStartTime = millis();}else{stopStartTime = millis();}
  analogWrite(ena_a,motorright);
              analogWrite(ena_b, motorleft);
              Serial.print("going in blue");/*lcd.clear();lcd.print("blue");
              lcd.setCursor(0, 1);*/break;
    case 'p':/*lcd.clear();lcd.print("purple");
  lcd.setCursor(0, 1);*/skip=0;break;
    case 'r': analogWrite(ena_a,0);
              analogWrite(ena_b,0);
              ON=true;
              /*lcd.clear();
              lcd.print("red");
              lcd.setCursor(0, 1);
              */break;
              Serial.print("going in red");
  }
  Serial.print(prev_color);
  Serial.print(",");
  /*if(prev_color!=color&&color!='N'){
    skip=1;
  }else {
    skip=0;
  }*/
  /*Serial.print(color);
  Serial.print(",");
  Serial.print(prev_color);
  Serial.print(",");
  Serial.print(sensor1);
  Serial.print(",");
  Serial.print(sensor2);
  Serial.print(",");
  Serial.print(sensor3);
  Serial.print(",");
  Serial.print(sensor4);
  Serial.print(",");
  Serial.print(sensor5);
  Serial.print(",");
  Serial.print(motorleft);
  Serial.print(",");
  Serial.print(motorright);*/
  Serial.println("");
  read_sensors();
  }
}
//////////////////////////////////////////////////////////////////////
void read_sensors(){
  sensor1 = analogRead(SENSOR_1);
  sensor2= analogRead(SENSOR_2);
  sensor3 = analogRead(SENSOR_3);
  sensor4= analogRead(SENSOR_4);
  sensor5 = analogRead(SENSOR_5);
  digitalWrite(MOTOR_RIGHT,HIGH);
  digitalWrite(MOTOR_LEFT,HIGH);
  if(sensor3<50){
    if (sensor1<50&&sensor2<50&&sensor4<50&&sensor5<50) {
      forwardM(125,125);
    }else {
      if(skip==1){
        while(1){
          forwardM(125,125); 
          if(sensor5>50&&prev_sensor5<50){
            break;
          }
        }
        analogWrite(ena_a, 0);
        analogWrite(ena_b, 0);
        skip=0;
      }else{
    
    if(sensor1<50) {
      forwardM(0,190);
    }else if (sensor2<50) {
      forwardM(0,190);
    }else if (sensor5<50) {
      forwardM(190,0);
    }else if (sensor4<50) {
      forwardM(190,0);
    }else {
      forwardM(125,125);
    }}
  }}else{
     if (sensor1<50) {
      forwardM(0,190);
    }else if (sensor2<50) {
      forwardM(0,190);
    }else if (sensor5<50) {
      forwardM(190,0);
    }else if (sensor4<50) {
      
      *motorleft=190;
      *motorright=0;
      forwardM(190,0);
    }else if(prev_sensor1<50){
      while (sensor3>50) {
        *motorleft=0;
        *motorright=190;
        forwardM(motorleft,motorright);
        read_sensors(motorleft,motorright);
        Serial.print("prev_sensor3");
        Serial.print(",");
      }
    }else if(prev_sensor5<50){
      while(sensor3>50){
        forwardM(190,0);
        read_sensors();
        Serial.print("prev_sensor5");
        Serial.print(",");
      }}else {
        forwardM(125,125);
      }
    }
    prev_sensor3=sensor3;
  prev_sensor1=sensor1;
  prev_sensor5=sensor5;
  }
  
void readcolor(){
  //read RED
 uint16_t clear, red, green, blue;                                 //declare variables for the colors                                                      // takes 50ms to read 
  tcs.getRawData(&red, &green, &blue, &clear);                      //read the sensor
                                            // turn off LED
  
  Serial.print("C:\t"); Serial.print(clear,DEC);                        //print color values
  Serial.print("\tR:\t"); Serial.print(red,DEC);
  Serial.print("\tG:\t"); Serial.print(green,DEC);
  Serial.print("\tB:\t"); Serial.print(blue,DEC);
 
                                                                    // Figure out some basic hex code for visualization
  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;
  RGB[0]= red;
  RGB[1]= green;
  RGB[2]= blue;
  Serial.print("\t");
  Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
  Serial.println();
  Serial.print(RGB[0]);
  Serial.print(",");
  Serial.print(RGB[1]);
  Serial.print(",");
  Serial.print(RGB[2]);
  Serial.print(",");
}
char what_color(int distinctRGB[][3],char distinctColors[],int RGB[]){
  char color;
  int biggestDifference =15 ;
	for (int i = 0; i < 15; i++)
	{
		int difference = sqrt(pow(RGB[0] - distinctRGB[i][0], 2) + pow(RGB[1] - distinctRGB[i][1], 2) + pow(RGB[2] - distinctRGB[i][2], 2));
		if (difference < biggestDifference)
		{
			color=distinctColors[i];
			biggestDifference = difference;
		}
	}
  if(biggestDifference==15){color='N';}
  return color;
}
void forwardM(uint8_t motorleft,uint8_t motorright){
  analogWrite(ena_a,motorright);
  analogWrite(ena_b, motorleft);
}
void calibration(){
  if(sensor2<50&&sensor1>50&&sensor3<50){
    while (sensor2<50){
      analogWrite(ena_a, 100);
      analogWrite(ena_b, 0);
    }
  }else if (sensor4<50&&sensor5>50&&sensor3<50) {
    while (sensor4<50){
      analogWrite(ena_a, 0);
      analogWrite(ena_b, 100);
    }
  }
  analogWrite(ena_a, 0);
  analogWrite(ena_b, 0);
}