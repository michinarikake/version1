#include <DFRobot_QMC5883.h>
#include <Wire.h>

DFRobot_QMC5883 compass;
int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int offX = 0;
int offY = 0;

const int motor11 = 2;
const int motor12 = 3;
const int motor21 = 6;
const int motor22 = 5;
const int pinA = 7;
const int pinB = 8;
const int SV_PIN = 9;
const int PWMa = A4;
const int PWMb = A3;
const int PW_QMC = 10;

int dummy_qmc() {
  int theta;
  minX = 0;
  maxX = 0;
  minY = 0;
  maxY = 0;
  offX = 0;
  offY = 0;
  setup_qmc5883();
  theta = qmc5883();
  return theta;
  digitalWrite(PW_QMC, LOW);
}

void setup_motor() {
  pinMode(motor11, OUTPUT);
  pinMode(motor12, OUTPUT);
  pinMode(motor21, OUTPUT);
  pinMode(motor22, OUTPUT);
  pinMode (pinA, INPUT);
  pinMode (pinB, INPUT);
  pinMode(PWMb, OUTPUT);
  pinMode(PWMa, OUTPUT);
  pinMode(PW_QMC, OUTPUT);
  Serial.println("setup done");
}

void setup_qmc5883() {
  digitalWrite(PW_QMC, HIGH);
  delay(100);
  compass.begin();
  if (compass.isQMC()) {
    compass.setRange(QMC5883_RANGE_2GA);
    compass.setMeasurementMode(QMC5883_CONTINOUS);
    compass.setDataRate(QMC5883_DATARATE_50HZ);
    compass.setSamples(QMC5883_SAMPLES_8);
  }
}

int qmc5883() {
  float X = 0;
  float Y = 0;
  int theta = 0;
  int i;
  i = 0;
  unsigned long times = millis();
  while (millis() < times + 100) {
    Vector mag = compass.readRaw();
    X += mag.XAxis * 0.000118522 + 0.232580541;
    Y += mag.YAxis * 0.000115348 + 0.211307431;
    theta =  atan2 ( Y, X ) * 57.2958;
  }

  return theta;
}


void right() {
  analogWrite( PWMb, 240 );
  analogWrite( PWMa, 240 );
  digitalWrite(motor11, HIGH);
  digitalWrite(motor22, LOW);
  digitalWrite(motor12, LOW);
  digitalWrite(motor21, HIGH);
}
void stop_motor() {
  digitalWrite(motor11, LOW);
  digitalWrite(motor22, LOW);
  digitalWrite(motor12, LOW);
  digitalWrite(motor21, LOW);
}

void turn_right(int kakudo) {
  short int theta;
  int theta0 = dummy_qmc();
  theta = theta0;
  unsigned long times = millis();
  analogWrite( PWMb, 240 );
  analogWrite( PWMa, 240 );
  digitalWrite(motor11, HIGH);
  digitalWrite(motor22, LOW);
  digitalWrite(motor12, LOW);
  digitalWrite(motor21, HIGH);

  while (theta - theta0 < kakudo) {
    int a = dummy_qmc();
    if (a < theta0 - 10) {
      theta = a + 360;
      Serial.println(theta);
    }
    else {
      theta = a;
      Serial.println(theta);
    }
  }

  stop_motor();
  times = millis() - times;
  float omega = 1000.0 * (theta - theta0) / times; //角速度(度/s)
  theta = dummy_qmc();
  int difference = -theta + theta0 + kakudo;
  int t = difference / omega; //時間(ms)
  Serial.println("omega");
  Serial.println(omega);
  Serial.println("t");
  Serial.println(t);

  while (t > 100) {
    setup_qmc5883();
    int a = dummy_qmc();
    if (a < theta0 - 20) {
      theta = a + 360;
      Serial.println(theta);
    }
    else {
      theta = a;
      Serial.println(theta);
    }
    int difference = - theta + theta0 + kakudo;
    int t = difference * 1000 / omega;
    Serial.println("keisann done");
    if ( t > 0) {
      analogWrite( PWMb, 200 );
      analogWrite( PWMa, 200 );
      digitalWrite(motor11, HIGH);
      digitalWrite(motor22, LOW);
      digitalWrite(motor12, LOW);
      digitalWrite(motor21, HIGH);
      delay(t);
      Serial.println("t done");

      digitalWrite(motor11, LOW);
      digitalWrite(motor21, LOW);
      Serial.print("t;");
      Serial.println(t);
    }
    else {
      analogWrite( PWMb, 200 );
      analogWrite( PWMa, 200 );
      digitalWrite(motor11, LOW);
      digitalWrite(motor22, HIGH);
      digitalWrite(motor12, HIGH);
      digitalWrite(motor21, LOW);
      delay(-t);
      Serial.println("t' done");

      digitalWrite(motor22, LOW);
      digitalWrite(motor12, LOW);
      Serial.print("t';");
      Serial.println(t);

    }
  }
  Serial.println("while done");
}

void setup() {
  Serial.begin(9600);
  setup_motor();
}
void loop() {
  turn_right( 90 );
  delay(2000);
}
