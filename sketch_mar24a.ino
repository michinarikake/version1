const int PWM = 10;
void setup() {
 pinMode(PWM, OUTPUT);

}

void loop() {
analogWrite(PWM, 249);

}
