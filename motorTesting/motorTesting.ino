typedef struct SystemConstants{
unsigned long baudRate;
float ticksPerRevOutput;
}SystemConstants;

static constexpr const SystemConstants sysCons= {
 .baudRate = 921600,.ticksPerRevOutput = 494
};

typedef struct MotorControl{
uint8_t leftMotorDirectionPin;
uint8_t rightMotorDirectionPin;
uint8_t leftMotorForwardState;
uint8_t rightMotorForwardState;
uint8_t leftMotorSpeedPin;
uint8_t rightMotorSpeedPin;
}MotorControl;

static constexpr const MotorControl motorControl = {
4,7,LOW,HIGH,5,6
};

typedef struct EncoderData{
float leftEncoderTicks;
float rightEncoderTicks;
uint8_t leftEncoderDirection;
uint8_t rightEncoderDirection;
}EncoderData;

volatile EncoderData encoderData = {
  .leftEncoderTicks = 0, .rightEncoderTicks = 0,.leftEncoderDirection=0,.rightEncoderDirection=0
};

typedef struct EncoderPins{
uint8_t leftEnc;
uint8_t leftEncDirection;
uint8_t rightEnc;
uint8_t rightEncDirection;
}EncoderPins;

static constexpr const EncoderPins encoderPins = {
  .leftEnc = 2,.leftEncDirection = 10,.rightEnc = 3,.rightEncDirection=11
};

typedef struct MotorState{
float rightMotorVelocity;
float leftMotorVelocity;
}MotorState;

volatile MotorState currentVelocity = {.rightMotorVelocity=0,.leftMotorVelocity=0};

typedef struct GlobalTime{
  unsigned long globalTimeInMs;
  unsigned long debugPrevTime;
  unsigned long velCalcPrevTime;
}GlobalTime;

volatile GlobalTime globalTimer = {
  .globalTimeInMs = 0,
  .debugPrevTime=0,
  .velCalcPrevTime=0
};

int pwm_left = 70;
int pwm_right = 70;
int demand = 0;

typedef struct Intervals{
uint8_t velocityCalculationInterval; // in milliseconds
uint16_t debugPrintInterval; // in milliseconds
}Intervals;

static const constexpr Intervals intervals = {
  .velocityCalculationInterval = 10,.debugPrintInterval = 1000
};

static const constexpr float rotationToRadians= 6.28318; // 1 rotation is 2Pi radians


// the setup function runs once when you press reset or power the board
void setup() {
  // Setup serial output:
  Serial.begin(sysCons.baudRate);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup Tick counting
  pinMode(encoderPins.leftEnc, INPUT_PULLUP);
  pinMode(encoderPins.rightEnc, INPUT_PULLUP);
  pinMode(encoderPins.leftEncDirection , INPUT);
  pinMode(encoderPins.rightEncDirection , INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPins.leftEnc), leftMotorISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPins.rightEnc), rightMotorISR, RISING);

  //Setup Motor direction pins
  pinMode(motorControl.leftMotorDirectionPin, OUTPUT);
  pinMode(motorControl.rightMotorDirectionPin, OUTPUT);
  
  //Setup Motor Speed Control pins
  pinMode(motorControl.leftMotorSpeedPin, OUTPUT);
  pinMode(motorControl.rightMotorSpeedPin, OUTPUT);

  // Currently, Set them up for the bot to go forward
  digitalWrite(motorControl.leftMotorDirectionPin,!motorControl.leftMotorForwardState); // Left wheel, LOW for forward
  digitalWrite(motorControl.rightMotorDirectionPin,!motorControl.rightMotorForwardState); // Right wheel, HIGH for forward
  /**
  Setup timer1 for 1Khz frequency
  */
  cli();  // Disable global interrupts

  TCCR1A = 0; //Register set to 0
  TCCR1B = 0; //Register set to 0
  TCNT1 = 0;
 
  OCR1A = 16000; //Counter for 1KHz interrupt 16*10^6/1000-1 no prescaler
  TCCR1B |= (1 << WGM12); //CTC mode
  TCCR1B |= (1 << CS10); //No prescaler
  TIMSK1 |= (1 << OCIE1A); //Compare interrupt mode
  /**
  Setup Timer2 for 250Hz
  */
  // TCCR2A = 0;  // Set Timer 2 to normal mode
  // TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);  // Set prescaler to 1024
  // TCNT2 = 0;   // Initialize counter to 0

  // OCR2A = 63;  // Set the compare match value (clock cycles = 16MHz / (prescaler * frequency) - 1)
  // TIMSK2 |= (1 << OCIE2A);  // Enable the compare match interrupt
  sei();  // Enable global interrupts

  // Set up constant speed rotation for test
  analogWrite(motorControl.leftMotorSpeedPin,100); // Left wheel speed
  analogWrite(motorControl.rightMotorSpeedPin,100); // Right wheel speed
}

// the loop function runs over and over again forever
void loop() {
/**
 Subthread to calculate and update current motor velocities runs every 5ms
*/
if ((globalTimer.globalTimeInMs - globalTimer.velCalcPrevTime) >= intervals.velocityCalculationInterval) {
    currentVelocity.rightMotorVelocity = getRPSFromTicks(getRotationsFromTicks(encoderData.rightEncoderTicks),globalTimer.globalTimeInMs - globalTimer.velCalcPrevTime);
    currentVelocity.leftMotorVelocity = getRPSFromTicks(getRotationsFromTicks(encoderData.leftEncoderTicks),globalTimer.globalTimeInMs - globalTimer.velCalcPrevTime);
    //Avoids race conditions.
    cli(); 
    encoderData.rightEncoderTicks=0;
    encoderData.leftEncoderTicks=0;
    sei();
    globalTimer.velCalcPrevTime = globalTimer.globalTimeInMs;
}
/**
Subthread to debug print every second
*/ 
if((globalTimer.globalTimeInMs - globalTimer.debugPrevTime)>=intervals.debugPrintInterval){

#if PRINT_ENCODER_TICK
  Serial.println("Encoder ticks:");
  Serial.print(encoderData.leftEncoderTicks);
  Serial.print("\t");
  Serial.println(encoderData.rightEncoderTicks);
#endif

#if PRINT_ENCODER_DIRECTION_PIN
  Serial.println("Encoder directionPins:");
  Serial.print(encoderData.leftEncoderDirection);
  Serial.print("\t");
  Serial.println(encoderData.rightEncoderDirection);
#endif

  Serial.println("Current Velocity in RPS:");
  Serial.print(currentVelocity.leftMotorVelocity);
  Serial.print("\t");
  Serial.println(currentVelocity.rightMotorVelocity);
  globalTimer.debugPrevTime = globalTimer.globalTimeInMs;
}

}

//Returns Radians per second velocity from rotations in 1Khz Intervals
//1 sec = 1000 ms //Encoder is on Motor Shaft
static inline float getRPSFromTicks(const float& rotations,float interval){
  return (float)rotations * rotationToRadians * ((float)(1000)/(float)interval);
}
static inline float getRotationsFromTicks(const float ticks){
  return  (float)(ticks/sysCons.ticksPerRevOutput);
}

// Writes actual PWM values to the motor
// Has a safety saturation check
void pwmWrite(int pwmL, int pwmR){
  if(pwmL>=UINT8_MAX){
    pwmL = UINT8_MAX;
  }
  if(pwmR>=UINT8_MAX){
    pwmR = UINT8_MAX;
  }
  analogWrite(motorControl.leftMotorSpeedPin,pwmL); // Left wheel speed
  analogWrite(motorControl.rightMotorSpeedPin,pwmR); // Right wheel speed
}
//Left motor Reverse Direction pin val =1
void leftMotorISR(){
  encoderData.leftEncoderDirection = digitalRead(encoderPins.leftEncDirection);
  (encoderData.leftEncoderDirection==1)?encoderData.leftEncoderTicks-=1:encoderData.leftEncoderTicks+=1;
}
//Right motor forward Direction pin val =1
void rightMotorISR(){
  encoderData.rightEncoderDirection = digitalRead(encoderPins.rightEncDirection);
  (encoderData.rightEncoderDirection==1)?encoderData.rightEncoderTicks+=1:encoderData.rightEncoderTicks-=1;
}

/**
Control loop
**/
ISR(TIMER2_COMPA_vect) {
  // Do something every 250Hz
}

/**
1 khz timer tick counter on Timer 1
*/ 
ISR(TIMER1_COMPA_vect) {
  globalTimer.globalTimeInMs+=1;
}