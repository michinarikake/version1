#include <MadgwickAHRS.h>
#include <Wire.h>
#include <DFRobot_QMC5883.h>
#include <Servo.h>


//MPU6050
Madgwick MadgwickFilter;
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU_ADDRESS  0x68

void setup_mpu6050() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);  //MPU6050_PWR_MGMT_1レジスタの設定
  Wire.write(0x00);
  Wire.endTransmission();

  MadgwickFilter.begin(100); //100Hz
}

void mpu6050(float angle[]) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  while (Wire.available() < 14);
  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw, Temperature;

  axRaw = Wire.read() << 8 | Wire.read();
  ayRaw = Wire.read() << 8 | Wire.read();
  azRaw = Wire.read() << 8 | Wire.read();
  Temperature = Wire.read() << 8 | Wire.read();
  gxRaw = Wire.read() << 8 | Wire.read();
  gyRaw = Wire.read() << 8 | Wire.read();
  gzRaw = Wire.read() << 8 | Wire.read();

  // 加速度値を分解能で割って加速度(G)に変換する
  float acc_x = axRaw / 16384.0;  //FS_SEL_0 16,384 LSB / g
  float acc_y = ayRaw / 16384.0;
  float acc_z = azRaw / 16384.0;

  // 角速度値を分解能で割って角速度(degrees per sec)に変換する
  float gyro_x = gxRaw / 131.0;  // (度/s)
  float gyro_y = gyRaw / 131.0;
  float gyro_z = gzRaw / 131.0;

  //Madgwickフィルターを用いて、PRY（pitch, roll, yaw）を計算
  MadgwickFilter.updateIMU(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z);

  //PRYの計算結果を取得する
  angle[0] = MadgwickFilter.getRoll();
  angle[1] = MadgwickFilter.getPitch();
}


//QMC5883
DFRobot_QMC5883 compass;
int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int offX = 0;
int offY = 0;

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
  for (int i = 0; i < 100; i++) {
    Vector mag = compass.readRaw();
    X += mag.XAxis * 0.000105862 + 0.319926492;
    Y += mag.YAxis * 0.000113317 - 0.21266429 + 0.0000000000000001;
  }
  int theta;
  theta =  atan2 ( Y, X ) * 57.2958;
  return theta;
}


//encorder_DCモータ
//motor1が、motor2が
const int motor11 = 2;
const int motor12 = 3;
const int motor21 = 6;
const int motor22 = 5;
const int PWMa = A4;
const int PWMb = A5;
const int pinA = 7;
const int pinB = 8;
int encoderPosCount = 0;
int pinALast;
int aVal;
int d = 10; //ロータリエンコーダ一回で進む距離

void setup_encorder() {
  pinMode (pinA, INPUT);
  pinMode (pinB, INPUT);
  pinALast = digitalRead(pinA);
}

void straight( int l ){
  int encoderPosCount = 0;
  int pinALast = digitalRead(pinA);
  int aVal;
  analogWrite( PWMa, 255 );
  analogWrite( PWMb, 249 );
  digitalWrite(motor11, LOW);
  digitalWrite(motor22, LOW);
  digitalWrite(motor12, HIGH);
  digitalWrite(motor21, HIGH);
  
  while (-encoderPosCount < l * 40 / 15.707 ) {
    aVal = digitalRead(pinA);
    if (aVal != pinALast) {
      //回ってたらピンAの値が変わるためエンコーダーの値を増やす
      if (digitalRead(pinB) != aVal) {
        //ピンBで前進後退を判断
        encoderPosCount ++;
        }
      else {
        encoderPosCount--;
      }
    }
    pinALast = aVal;
  }
  
    digitalWrite(motor12, LOW);
    digitalWrite(motor21, LOW);
}



//setup、ピンの名前は省略
//tはミリ秒
void turn_right(){
  analogWrite( PWMa, 150 );
  analogWrite( PWMb, 150 );
  digitalWrite(motor11, HIGH);
  digitalWrite(motor22, LOW);
  digitalWrite(motor12, LOW);
  digitalWrite(motor21, HIGH);
}

void turn_left(){
  analogWrite( PWMa, 150 );
  analogWrite( PWMb, 150 );
  digitalWrite(motor11, LOW);
  digitalWrite(motor22, HIGH);
  digitalWrite(motor12, HIGH);
  digitalWrite(motor21, LOW);
}

void stop(){
  digitalWrite(motor11, LOW);
  digitalWrite(motor22, LOW);
  digitalWrite(motor12, LOW);
  digitalWrite(motor21, LOW);
}


void back( int l ){
  int encoderPosCount = 0;
  int pinALast = digitalRead(pinA);
  int aVal;
  analogWrite( PWMa, 150 );
  analogWrite( PWMb, 150 );
  digitalWrite(motor11, HIGH);
  digitalWrite(motor22, HIGH);
  digitalWrite(motor12, LOW);
  digitalWrite(motor21, LOW);
  
  while (encoderPosCount < l * 1000 / 3925) {
    aVal = digitalRead(pinA);
    if (aVal != pinALast) {
      if (digitalRead(pinB) != aVal) {
        encoderPosCount ++;
        }
      else {
        encoderPosCount--;
      }
    }
    pinALast = aVal;
  }
  
    digitalWrite(motor12, LOW);
    digitalWrite(motor21, LOW);
}


//ultrasonic
const int trigPin = 11;
const int echoPin = 12;

double duration = 0;
double distance = 0;

void setup_ultrasonic() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

float ultrasonic() {
  for ( int i = 0; i < 100; i++) {
    digitalWrite(trigPin, LOW);
    digitalWrite(echoPin, LOW);
    delayMicroseconds(1);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance += duration * 0.000001 * 34000 / 2;
  }
  digitalWrite(trigPin, LOW);
  digitalWrite(echoPin, LOW);
  distance = distance * 0.01;
  return distance;
}

//Servo
Servo myservo1;
Servo myservo2;
const int SV_PIN1 = 9;
const int SV_PIN2 = 10;

void setup_servo() {
  myservo1.attach(SV_PIN1, 500, 2400);  // サーボの割当(パルス幅500~2400msに指定)距離せんさ
  myservo2.attach(SV_PIN2, 500, 2400); //プーリー用
}
