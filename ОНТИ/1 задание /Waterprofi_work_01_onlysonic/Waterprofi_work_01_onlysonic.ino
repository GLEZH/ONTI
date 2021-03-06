#include <Servo.h>

#define SPEED_1      5
#define DIR_1        4

#define R_PIN_TRIG 28 //R
#define R_PIN_ECHO 29

#define RU_PIN_TRIG 30  //RU 
#define RU_PIN_ECHO 31

#define U_PIN_TRIG 32  //U 
#define U_PIN_ECHO 33

#define LU_PIN_TRIG 34  //LU 
#define LU_PIN_ECHO 35

#define L_PIN_TRIG 36  //L
#define L_PIN_ECHO 37

#define D_PIN_TRIG 38  //D 
#define D_PIN_ECHO 38

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
  myservo.write(137);



}

void servoturnright() {
  myservo.write(97);

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

 while (k != 1)
    {

  if (durat_R() == range_min and durat_RU() == side_range_max)
  {
   
      if (durat_U() <= range_min)
      {
        stop();
        delay(1000);
        Serial.println("Im stop");
        Serial.println("Turning...");
        servoturnleft();
        forward();
        if (durat_D() == range_min)
        {
          stop();
          k++;

        }

      }
      Serial.println("Go");
      pos_const = 117;
      myservo.write(pos_const);
      forward();
    
  }
  
  else if (durat_R() < range_min and durat_RU() < side_range_max) {
    do {
      myservo.write(pos_const-15);
      forward();
    } while (durat_R() < range_min and durat_RU() < side_range_max);
  }

  else if (durat_R() > range_min and durat_RU() > side_range_max) {
    do {
      myservo.write(pos_const+15);
      forward();
    } while (durat_R() > range_min and durat_RU() > side_range_max);
  }
  }
    

  //-----------------------------------------
while (c != 1)
    {
  if (durat_L() == range_min and durat_LU() == side_range_max )
  {

      if (durat_U() <= range_min)
      {
        stop();
        Serial.println("Im stop");
      }


      Serial.println("Go");
      myservo.write(117);
      forward();

    
  }
  else if (durat_L() < range_min and durat_LU() < side_range_max) {
    do {
      myservo.write(pos_const-15);
      forward();
    } while (durat_L() < range_min and durat_LU() < side_range_max);
  }

  else if (durat_L() > range_min and durat_LU() > side_range_max) {
    do {
      myservo.write(pos_const+15);
      forward();
    } while (durat_L() > range_min and durat_LU() > side_range_max);
  }
}
}
