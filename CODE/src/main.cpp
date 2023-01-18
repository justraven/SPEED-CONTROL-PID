#include <Arduino.h>

#include <LiquidCrystal.h>
#include <Servo.h>

#include <parsing.h>

#define TACHO_GENERATOR A0

//----- POT PINs -----//
#define KP_PIN A1
#define KI_PIN A2
#define KD_PIN A3
#define SET_POINT_PIN     A4
#define TIME_SAMPLING_PIN A6

//----- INDICATOR -----//
#define BUZZER 10
#define LED 11

//----- LCD PINs -----//
#define RS 3
#define EN 2
#define D4 4
#define D5 5
#define D6 6
#define D7 7

LiquidCrystal lcd(RS,EN,D4,D5,D6,D7);

//----- filter variable -----//
float sum = 0, average = 0;
float averageFilter(uint8_t analogPin,int N);

//----- ESC -----//
Servo esc;
int escValue = 0;

//----- loop time -----//
unsigned long t = 0;
uint8_t timeSampling = 50; //period of loop : 50ms

//----- PID Variable -----//
float KP, KI, KD, KP_, KI_, KD_;
float setPoint, feedback, controlSignal; //velocity is feedback sensor
float error = 0.00, totalError, lastError, deltaError;
float maxControl = 1410;
float minControl = 900;

//----- parsing variable -----//
int mode;
float PWM;

//----- map float -----//
float mapFloat(float x, float inMin, float inMax, float outMin, float outMax){
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void setup() {

  Serial.begin(9600);

  lcd.begin(16,2);
  esc.attach(9);

  pinMode(BUZZER,OUTPUT);
  pinMode(LED,OUTPUT);

  digitalWrite(BUZZER,LOW);
  digitalWrite(LED,HIGH);

}

void loop() {

  if(Serial.available() > 0){

    READ_DATA_UNTIL('\n');
    data.replace(',','.');
    parseString();

    mode           = DATA_STR(0).toInt();
    controlSignal  = DATA_STR(1).toFloat(); 

    Serial.flush();

  }

  if(millis() - t > timeSampling){

    t += timeSampling;

    feedback = averageFilter(TACHO_GENERATOR,50);

    if(mode == 0){

      KP = mapFloat(analogRead(KP_PIN),0,883,0,10);
      KI = mapFloat(analogRead(KI_PIN),0,883,0,100);
      KD = mapFloat(analogRead(KD_PIN),0,883,0,10);

      // KP = 1.529;
      // KI = 0.01619;
      // KD = 0.1829;

      // KP = 1.313;
      // KI = 0.01371;
      // KD = 0.1396;

      // KP = 1.414;
      // KI = 0.0264;
      // KD = 0.08539;


      setPoint = map(analogRead(SET_POINT_PIN),0,883,0,400);

      error = setPoint - feedback;
      totalError += error;

      if (totalError >= maxControl) totalError = maxControl;
      else if (totalError <= minControl) totalError = minControl;

      deltaError = error - lastError;

      KP_ = KP * error;
      KI_ = (KI/1000) * totalError * timeSampling;
      // KI_ = KI * totalError * timeSampling;
      KD_ = (KD / timeSampling) * deltaError;

      controlSignal = KP_ + KI_ + KD_;

      if(controlSignal >= maxControl) controlSignal = maxControl;
      else if(controlSignal <= minControl) controlSignal = minControl;

      esc.writeMicroseconds(controlSignal);

      lastError = error;

    }

    else if (mode == 1){

      if(controlSignal >= maxControl)
        controlSignal = maxControl;
      else if(controlSignal <= minControl)
        controlSignal = minControl;

      esc.writeMicroseconds(controlSignal);      

    }

    String S = String(setPoint);
    String F = String(feedback);
    String P = String(KP);
    String I = String(KI);
    String D = String(KD);
    String T = String(timeSampling);
    String O = String(controlSignal);

    S.replace('.',',');
    F.replace('.',',');
    P.replace('.',',');
    I.replace('.',',');
    D.replace('.',',');
    T.replace('.',',');
    O.replace('.',',');

    Serial.print('S');Serial.print(S);
    Serial.print('F');Serial.print(F);
    Serial.print('P');Serial.print(P);
    Serial.print('I');Serial.print(I);
    Serial.print('D');Serial.print(D);
    Serial.print('T');Serial.print(T);
    Serial.print('O');Serial.println(O);

    lcd.setCursor(0,0);
    lcd.print("SP:");lcd.print(S);
    lcd.setCursor(0,1);
    lcd.print("PV:");lcd.print(F);
    lcd.flush();
    

  }

}

float averageFilter(uint8_t analogPin, int N){

  sum = 0;

  for(int i = 0; i < N; i++)
    sum += analogRead(analogPin);
  
  average = sum / N;

  return average;

}