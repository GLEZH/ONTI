#include <Servo.h>

#define SPEED_1      5
#define DIR_1        4

#define R_PIN_TRIG 4  //R
#define R_PIN_ECHO 5

#define RU_PIN_TRIG 2  //RU 
#define RU_PIN_ECHO 3

#define U_PIN_TRIG 7  //U 
#define U_PIN_ECHO 6

#define LU_PIN_TRIG 11  //LU 
#define LU_PIN_ECHO 10

#define L_PIN_TRIG 13  //L
#define L_PIN_ECHO 12

#define D_PIN_TRIG 1  //D 
#define D_PIN_ECHO 0

#define SERVO_PIN 9

unsigned int R_duration;
unsigned int RU_duration;
unsigned int U_duration;
unsigned int L_duration;
unsigned int LU_duration;
unsigned int D_duration;

unsigned int R_cm;
unsigned int RU_cm;
unsigned int U_cm;
unsigned int L_cm;
unsigned int LU_cm;
unsigned int D_cm;

unsigned int range_min = 100 ;

unsigned int range_max = 200;

unsigned int pos = 0;

unsigned int pos_const = 117;

const int NUM_READ = 30;

int side_range_max = 58;

Servo myservo;

int k = 0;

int c = 0;


int durat_R() {
  digitalWrite(R_PIN_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(R_PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(R_PIN_TRIG, LOW);
  R_duration = pulseIn(midArifm(R_PIN_ECHO), HIGH);
  R_cm = R_duration / 58;
  return R_cm;

}

int durat_RU() {
  digitalWrite(RU_PIN_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(RU_PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(RU_PIN_TRIG, LOW);
  RU_duration = pulseIn(midArifm(RU_PIN_ECHO), HIGH);
  RU_cm = RU_duration / 58;
  return RU_cm;

}

int durat_U() {
  digitalWrite(U_PIN_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(U_PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(U_PIN_TRIG, LOW);
  U_duration = pulseIn(midArifm(U_PIN_ECHO), HIGH);
  U_cm = U_duration / 58;
  return U_cm;

}

int durat_L() {
  digitalWrite(L_PIN_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(L_PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(L_PIN_TRIG, LOW);
  L_duration = pulseIn(midArifm(L_PIN_ECHO), HIGH);
  L_cm = L_duration / 58;
  return L_cm;

}

int durat_LU() {
  digitalWrite(LU_PIN_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(LU_PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(LU_PIN_TRIG, LOW);
  LU_duration = pulseIn(midArifm(LU_PIN_ECHO), HIGH);
  LU_cm = LU_duration / 58;
  return LU_cm;

}

int durat_D() {
  digitalWrite(D_PIN_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(D_PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(D_PIN_TRIG, LOW);
  D_duration = pulseIn(midArifm(D_PIN_ECHO), HIGH);
  D_cm = D_duration / 58;
  return D_cm;

}

int midArifm(int x ) {
  long sum = 0;                       // локальная переменная sum
  for (int i = 0; i < NUM_READ; i++)  // согласно количеству усреднений
    sum += x;                  // суммируем значения с любого датчика в переменную sum
  return ((float)sum / NUM_READ);
}

void servoturnleft() {
  myservo.write(147);



}

void servoturnright() {
  myservo.write(87);

}

void forward() {
  digitalWrite(DIR_1, LOW);
  analogWrite(SPEED_1, 150);

}

void backwards() {
  digitalWrite(DIR_1, HIGH);
  analogWrite(SPEED_1, 150);
}

void stop() {
  analogWrite(SPEED_1, 0);
}

void straight() {
  myservo.write(pos_const);
}


void setup() {



  Serial.begin (9600);

  pinMode(R_PIN_TRIG, OUTPUT);
  pinMode(R_PIN_ECHO, INPUT);

  pinMode(RU_PIN_TRIG, OUTPUT);
  pinMode(RU_PIN_ECHO, INPUT);

  pinMode(U_PIN_TRIG, OUTPUT);
  pinMode(U_PIN_ECHO, INPUT);

  pinMode(LU_PIN_TRIG, OUTPUT);
  pinMode(LU_PIN_ECHO, INPUT);

  pinMode(L_PIN_TRIG, OUTPUT);
  pinMode(L_PIN_ECHO, INPUT);

  pinMode(D_PIN_TRIG, OUTPUT);
  pinMode(D_PIN_ECHO, INPUT);

  myservo.attach(9);
  myservo.write(pos_const);

  delay(100);
}

void loop() {
  forward();
  delay(18500);
  stop();
  
  servoturnleft();
  forward();
  delay(8800);

  servoturnright();
  forward();
  delay(2900);
  
  straight();
  forward();
  delay(16600);
  stop();
  delay(500000);

}
