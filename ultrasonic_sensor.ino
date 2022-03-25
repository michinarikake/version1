const int echoPin1 = A0; //前方
const int echoPin2 = A1; //後方
const int trigPin = A2;
double duration1 = 0;
double distance1 = 0;
double duration2 = 0;
double distance2 = 0;

void setup_ultrasonic() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(echoPin2, INPUT);
}

float ultrasonic1() {
   for ( int j = 0; j < 100; j++) {
      digitalWrite(trigPin, LOW);
      digitalWrite(echoPin1, LOW);
      delayMicroseconds(1);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      duration1 = pulseIn(echoPin1, HIGH);
      distance1 += duration1 * 0.000001 * 34000 / 2;
    }
    digitalWrite(trigPin, LOW);
    digitalWrite(echoPin1, LOW);
    distance1 = distance1 * 0.01;
    return distance1;
}

float ultrasonic2() {
   for ( int j = 0; j < 100; j++) {
      digitalWrite(trigPin, LOW);
      digitalWrite(echoPin2, LOW);
      delayMicroseconds(1);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      duration1 = pulseIn(echoPin2, HIGH);
      distance1 += duration1 * 0.000001 * 34000 / 2;
    }
    digitalWrite(trigPin, LOW);
    digitalWrite(echoPin2, LOW);
    distance2 = distance2 * 0.01;
    return distance2;
}

void setup(){
Serial.begin( 9600 );
setup_ultrasonic();
}

void loop(){
  Serial.print("前方");
  Serial.println(ultrasonic1());
  Serial.print("後方");
  Serial.println(ultrasonic2());
}
