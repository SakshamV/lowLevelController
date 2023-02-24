int k = 0;
volatile int enc1 = 0;
volatile int enc2 = 0;
volatile unsigned long time1 = millis();
volatile unsigned long time2 = millis();

int pwm_left = 70;
int pwm_right = 70;

int demand = 100;
// the setup function runs once when you press reset or power the board

void setup() {
  Serial.begin(9600);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  //delay(2000);

  digitalWrite(4,LOW); // Left wheel, LOW for forward
  digitalWrite(7,HIGH); // Right wheel, HIGH for forward
  
  analogWrite(5,pwm_left); // Left wheel speed
  analogWrite(6,pwm_right); // Right wheel speed

  attachInterrupt(0, pin_ISR1, RISING);
  attachInterrupt(1, pin_ISR2, RISING);

 
}

// the loop function runs over and over again forever
void loop() {


  //int RE1 = analogRead(A0);
  //int RE2 = analogRead(A1);
  int enc1_old = enc1;
  int enc2_old = enc2;
  delay(100);

  // Serial.print(enc1);
  // Serial.print("\t");
  // Serial.print(enc2);
  // Serial.print("\n");

  int speedL = (enc1-enc1_old);
  int speedR = (enc2-enc2_old);
  //delay();

  int err1 = demand - speedL;
  int err2 = demand - speedR;

  Serial.print(err1);
  Serial.print("\t");
  Serial.print(err2);
  Serial.print("\n");

  pwm_left = pwm_left + err1;
  pwm_right = pwm_right + err2;

  pwmWrite(pwm_left,pwm_right);
}

void pwmWrite(int pwmL, int pwmR){
  analogWrite(5,pwmL); // Left wheel speed
  analogWrite(6,pwmR); // Right wheel speed
}

void pin_ISR1(){
  enc1 = enc1 + 1;
}
void pin_ISR2(){
  enc2 = enc1 + 1;
}

