
#include <Wire.h>
#include <DFRobot_QMC5883.h>
#include <Servo.h>
Servo myservo;
const int motor11 = 2;
const int motor12 = 3;
const int motor21 = 6;
const int motor22 = 5;
const int pinA = 7;
const int pinB = 8;
const int SV_PIN = 9;
const int PWMa = A4;
const int PWMb = A3;
int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int offX = 0;
int offY = 0;
int i = 0;

DFRobot_QMC5883 compass;

void setup_servo() {
  myservo.attach(SV_PIN, 500, 2400);  // サーボの割当(パルス幅500~2400msに指定)
}

void setup_qmc5883() {
  compass.begin();
  if (compass.isQMC()) {
    compass.setRange(QMC5883_RANGE_2GA);
    compass.setMeasurementMode(QMC5883_CONTINOUS);
    compass.setDataRate(QMC5883_DATARATE_50HZ);
    compass.setSamples(QMC5883_SAMPLES_8);
  }
}

int qmc5883() {
  double X = 0;
  double Y = 0;
  unsigned long times = millis();
  if (times <10000){
    
  
  for (int i = 0; i < 5; i++) {
    Vector mag = compass.readRaw();
    X += mag.XAxis * 0.000118522 + 0.232580541;
    Y += mag.YAxis * 0.000115348 + 0.211307431;
  }
  int theta;
  theta =  atan2 ( Y, X ) * 57.2958;
  return theta;
  }
  else {
    return 0;
  }
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
  Serial.println("setup done");
}

void right_rotate( int kakudo ) {
  setup_qmc5883();
  int theta;
  int theta0 = qmc5883();
  Serial.print("theta0; ");
  Serial.println(theta0);
  Serial.println("set done");
  analogWrite( PWMb, 150 );
  analogWrite( PWMa, 150 );
  digitalWrite(motor11, HIGH);
  digitalWrite(motor22, LOW);
  digitalWrite(motor12, LOW);
  digitalWrite(motor21, HIGH);
  Serial.println("motor done");
  delay(500);

  digitalWrite(motor11, LOW);
  digitalWrite(motor21, LOW);

  setup_qmc5883();
  int a = qmc5883();
  if (a < theta0 - 50) {
    theta = a + 360;
  }
  else {
    theta = a;
  }
  double omega = theta - theta0; //１秒当たりの角速度
  int difference = - theta + theta0 + kakudo;
  int t = difference * 1000 / omega; //ミリ秒
  Serial.println("omega");
  Serial.println(omega);
  Serial.println("t");
  Serial.println(t);



  while (abs(theta - theta0 - kakudo) > 10) {
    setup_qmc5883();
    a = qmc5883();
    if (a < theta0 - 50) {
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
  setup_qmc5883();
  setup_motor();
  Serial.begin(9600);
}
void loop() {
  right_rotate( 60 );
  delay(10000000000);
}
