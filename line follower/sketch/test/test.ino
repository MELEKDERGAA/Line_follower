#define MOTOR_LEFT 7
#define reverse_LEFT 12
#define MOTOR_RIGHT 6
#define reverse_RIGHT 11
#define ena_a 9
#define ena_b 10
#define SENSOR_1 A0
#define SENSOR_2 A1
#define SENSOR_3 A2
#define SENSOR_4 A3
#define SENSOR_5 A6
#define S0 2
#define S1 3
#define S2 4
#define S3 5
#define out 8
bool stop=false;bool forward=true;
uint8_t motorleft;
uint8_t motorright;
int P, D, I, previousError, PIDvalue, error;
int lfspeed = 200;
float Kp = 0;
float Kd = 0;
float Ki = 0 ;
int minValues[6], maxValues[6], threshold[6];
 int sensor1;
  int sensor2;
  int sensor3;
  int sensor4;
  int sensor5;
char color;
char prev_color='N';
uint16_t prev_sensor3,prev_sensor1,prev_sensor5;
int RGB[3]={};
//int distinctRGB[7][3] = {{109,7,26}, {239, 216, 7}, {0, 86, 27}, {127,0 ,190}, {15, 5, 107},{255,255,255},{0,0,0}};
//char distinctColors[5] = {'r' , 'y',   'g',    'p',   'b'};
int distinctRGB[7][3] = {{131,32,46}, {246, 230, 162}, {90, 131, 57}, {148,156 ,104}, {37, 33, 100},{255,255,255},{0,0,0}};
char distinctColors[7] = {'r' , 'y',   'g',    'p',   'b' ,'w','k'};
                        //red  yellow  green purple  blue
void setup() {
  // put your setup code here, to run once:

  pinMode(MOTOR_LEFT, OUTPUT);
  pinMode(MOTOR_RIGHT, OUTPUT);
  pinMode(reverse_LEFT, OUTPUT);
  pinMode(reverse_RIGHT, OUTPUT);
  pinMode(ena_a,OUTPUT);
  pinMode(ena_b, OUTPUT);
  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);
  pinMode(SENSOR_3, INPUT);
  pinMode(SENSOR_4, INPUT);
  pinMode(SENSOR_5, INPUT);
  pinMode(S0, OUTPUT);
	pinMode(S1, OUTPUT);
	pinMode(S2, OUTPUT);
	pinMode(S3, OUTPUT);
	pinMode(out, INPUT);
  Serial.begin(9600);
  calibrate();
}

void loop() {
  // put your main code here, to run repeatedly:
  read_sensors(&motorleft,&motorright);
  if (sensor1 > threshold[1] && sensor5 < threshold[5] )
    {
      motorleft = 0; motorright = lfspeed;
      analogWrite(ena_a,0);
      analogWrite(ena_b,lfspeed);
    }

    else if (sensor5 > threshold[5] && sensor1 < threshold[1])
    { motorleft = lfspeed; motorright = 0;
      analogWrite(ena_a,lfspeed);
      analogWrite(ena_b,0);
    }
    else if (sensor3 > threshold[3])
    {
      Kp = 1 * (1000 - sensor3);
      Kd = 5 * Kp;
      //Ki = 0.0001;
      linefollow();
    }
  //read color in RGB
  readcolor(S2,S3,out);
  // seeing the colsest color 
  color=what_color(distinctRGB, distinctColors, RGB);
  Serial.print(color);
  Serial.print(",");
  if(prev_color!=color&&color!='N'){
  switch (color) {
    case 'y':if(prev_color=='y'||prev_color=='N'){prev_color=color;}break;
    case 'g':if(prev_color=='g'||prev_color=='N'){prev_color=color;}break;
    case 'b': analogWrite(ena_a,0);
              analogWrite(ena_b,0);delay(5000);
              analogWrite(ena_a,motorright);
              analogWrite(ena_b, motorleft);break;
    case 'p':break;
    case 'r': analogWrite(ena_a,0);
              analogWrite(ena_b,0);
              stop=true;
              break;
  }
  forward=true;
  }else{
  forward=false;
  }
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
  
}
//////////////////////////////////////////////////////////////////////
void read_sensors(uint8_t *motorleft,uint8_t *motorright){
  sensor1 = analogRead(SENSOR_1);
  sensor2= analogRead(SENSOR_2);
  sensor3 = analogRead(SENSOR_3);
  sensor4= analogRead(SENSOR_4);
  sensor5 = analogRead(SENSOR_5);
  int sensor[5]={sensor1,sensor2,sensor3,sensor4,sensor5};
}
void readcolor(int S_2 , int S_3 , int Out){
  //read RED
  digitalWrite(S_2, LOW);
  digitalWrite(S_3, LOW);
  RGB[0]=map(pulseIn(Out, HIGH),150,23,0,255);
  //RGB[0]=pulseIn(Out, HIGH);
  //read GREEN
  digitalWrite(S_2, HIGH);
  digitalWrite(S_3, HIGH);
  RGB[1]=map(pulseIn(Out, HIGH),150,23,0,255);
  //RGB[1]=pulseIn(Out, HIGH);
  //read BLUE
  digitalWrite(S_2, LOW);
  digitalWrite(S_3, HIGH);
  RGB[2]=map(pulseIn(Out, HIGH),116,17,0,255);
  //RGB[2]=pulseIn(Out, HIGH);
  Serial.print(RGB[0]);
  Serial.print(",");
  Serial.print(RGB[1]);
  Serial.print(",");
  Serial.print(RGB[2]);
  Serial.print(",");
}
char what_color(int distinctRGB[][3],char distinctColors[],int RGB[]){
  char color="N";
  int biggestDifference =20 ;

	for (int i = 0; i < 7; i++)
	{
		int difference = sqrt(pow(RGB[0] - distinctRGB[i][0], 2) + pow(RGB[1] - distinctRGB[i][1], 2) + pow(RGB[2] - distinctRGB[i][2], 2));
		if (difference < biggestDifference)
		{
			color=distinctColors[i];
			biggestDifference = difference;
		}
	}
  return color;
}
void calibrate()
{
   sensor1 = analogRead(SENSOR_1);
  sensor2= analogRead(SENSOR_2);
  sensor3 = analogRead(SENSOR_3);
  sensor4= analogRead(SENSOR_4);
  sensor5 = analogRead(SENSOR_5);
  int sensor[5]={sensor1,sensor2,sensor3,sensor4,sensor5};
  for ( int i = 0; i < 6; i++)
  {
    minValues[i] = sensor[i];
    maxValues[i] = sensor[i];
  }
  
  for (int i = 0; i < 500; i++)
  {
    digitalWrite(MOTOR_RIGHT, HIGH);
    digitalWrite(reverse_RIGHT,LOW);
    digitalWrite(MOTOR_LEFT,LOW);
    digitalWrite(reverse_LEFT,HIGH);
    analogWrite(ena_a, 100);
    analogWrite(ena_b,100);

    for ( int i = 1; i < 6; i++)
    {
      if (sensor[i] < minValues[i])
      {
        minValues[i] = sensor[i];
      }
      if (sensor[i] > maxValues[i])
      {
        maxValues[i] = sensor[i];
      }
    }
  }

  for ( int i = 0; i < 6; i++)
  {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
  }
  Serial.println();
  
  digitalWrite(MOTOR_RIGHT, HIGH);
    digitalWrite(reverse_RIGHT,LOW);
    digitalWrite(MOTOR_LEFT,HIGH);
    digitalWrite(reverse_LEFT,LOW);
    analogWrite(ena_a,0);
    analogWrite(ena_b,0);
}
void linefollow()
{
  int error = (sensor2 - sensor4);

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  motorleft = lfspeed - PIDvalue;
  motorright = lfspeed + PIDvalue;

  if (motorleft > 255) {
    motorleft = 255;
  }
  if (motorleft < 0) {
    motorleft = 0;
  }
  if (motorright > 255) {
    motorright = 255;
  }
  if (motorright < 0) {
    motorright = 0;
  }
  analogWrite(ena_a,motorleft);
  analogWrite(ena_b,motorright);

}