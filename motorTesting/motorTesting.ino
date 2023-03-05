typedef struct SystemConstants{
unsigned long baudRate;
float ticksPerRevOutput;
}SystemConstants;
///493.8
static constexpr const SystemConstants sysCons= {
 .baudRate = 921600,.ticksPerRevOutput = 336.6
};

typedef struct MotorControl{
uint8_t leftMotorDirectionPin;
uint8_t rightMotorDirectionPin;
bool leftMotorForwardState;
bool rightMotorForwardState;
uint8_t leftMotorSpeedPin;
uint8_t rightMotorSpeedPin;
}MotorControl;

static constexpr const MotorControl motorControl = {
4,7,LOW,HIGH,5,6
};

typedef struct EncoderData{
int rightEncoderPrev;
int leftEncoderPrev;
int leftEncoderTicks;
int rightEncoderTicks;
uint8_t leftEncoderDirection;
uint8_t rightEncoderDirection;
}EncoderData;

static EncoderData encoderData = {
  0,0,.leftEncoderTicks = 0, .rightEncoderTicks = 0,.leftEncoderDirection=0,.rightEncoderDirection=0
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
float filteredRightVel;
float filteredLeftVel;
float rawRight;
float rawLeft;
void reset(){
  filteredRightVel=0;
  filteredLeftVel=0;
}
}MotorState;

static MotorState currentVelocity = {.filteredRightVel=0,.filteredLeftVel=0,0,0};

typedef struct GlobalTime{
  unsigned long globalTimeInMs;
  unsigned long debugPrevTime;
  unsigned long velCalcPrevTime;
  unsigned long controlLoopPrevTime;
}GlobalTime;

static GlobalTime globalTimer = {
  .globalTimeInMs = 0,
  .debugPrevTime=0,
  .velCalcPrevTime=0,
  .controlLoopPrevTime=0
};

typedef struct Intervals{
unsigned long velocityCalculationInterval; // in milliseconds
uint16_t debugPrintInterval; // in milliseconds
}Intervals;

static const constexpr Intervals intervals = {
  .velocityCalculationInterval = 2,.debugPrintInterval = 1000
};

typedef struct Gains{
float kp;
float kd;
float ki;
float iSat;
}Gains;

typedef struct MotorsGain{
Gains leftMotorGains;
Gains rightMotorGains;
}MotorsGain;

static const constexpr MotorsGain motorGains = {
  .leftMotorGains = {50,0.0001,0.0001,100},.rightMotorGains= {50,0.0001,0.0001,100}
};

typedef struct ControlLoopVariables{
float motor_setpoint = 0;
float motor_input = 0;
float motor_output = 0;
float motor_error = 0;
float motor_prev_error =0;
float motor_integral =0;
float motor_derivative=0;
ControlLoopVariables(const Gains * const ipGains):motorGains(ipGains){
;
}
const Gains * const motorGains;
void resetController(){
motor_setpoint = 0;
motor_input = 0;
motor_output = 0;
motor_error = 0;
motor_prev_error =0;
motor_integral =0;
motor_derivative=0;
}
}ControlLoopVariables;

static const constexpr float rotationToRadians= 6.28318; // 1 rotation is 2Pi radians
static ControlLoopVariables leftMotor(&motorGains.leftMotorGains);
static ControlLoopVariables rightMotor(&motorGains.rightMotorGains);
volatile bool startControl = true;

#define DEBUG_VELOCITIES 0
#define TRAJECTORY 1
#define DEBUG_CONTROL 1
// wheel diameter 64.5mm
#if TRAJECTORY
static constexpr const int NO_OF_WAYPOINTS = 350;
//Diameter 64.5 mm
static const constexpr float vmax =0.8; //(m/s) actually some 0.406~
static constexpr float getXLinearVelocity(const float& t,const float& T){
    return  t*(4*vmax/T)+(-(4*vmax/(T*T))*t*t);
}
static constexpr  float getT(const float&  xInitial, const float& xFinal){
    return (6/(4*vmax))*(xFinal - xInitial);
}
static constexpr float convertLinearToRPS(const float& linearVelocity){
  return linearVelocity*15.503875969;
}
static constexpr float convertRPSToLinear(const float& RPS){
  return RPS*(1/15.503875969);
}
template<int Max>
struct Trajectory {
      constexpr Trajectory() : wayPoints(),noOfWayPoints(Max),totalTime(0) {
      float time = 0.0;
      const constexpr float loopRate = 100; // 250 Hz
      const constexpr float timeIncrement = 1/loopRate;
      const constexpr float xInit = 0;  
      const constexpr float xFinal = 0.4052654; // 1 rotation of the wheel
      const constexpr float totalTime = getT(xInit,xFinal);
      int i=0;
      while (true) {
          this->wayPoints[i] = convertLinearToRPS(getXLinearVelocity(time,totalTime));
          time += timeIncrement;
          if(time>=totalTime){
              this->wayPoints[i] = 0;
              if(i<Max){
                ++i;
              }else{
                this->wayPoints[i]=0;
              }
              break;
          }else{
            if(i<Max){
              ++i;
            }else{
              this->wayPoints[i]=0;
              break;
            }
          }
      }
      noOfWayPoints = i;
      this->totalTime = totalTime;
    }
    float wayPoints[Max];
    int noOfWayPoints;
    float totalTime;
};
static const constexpr auto wayPoints = Trajectory<NO_OF_WAYPOINTS>();
#endif 
// the setup function runs once when you press reset or power the board
void setup() {
  // Setup serial output:
  Serial.begin(sysCons.baudRate);
  delay(5000);
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
  digitalWrite(motorControl.leftMotorDirectionPin,motorControl.leftMotorForwardState); // Left wheel, LOW for forward
  digitalWrite(motorControl.rightMotorDirectionPin,motorControl.rightMotorForwardState); // Right wheel, HIGH for forward
  /**
  Setup timer1 for 1Khz frequency
  */
  cli();  // Disable global interrupts
  TCCR1A = 0; //Register set to 0
  TCCR1B = (1 << WGM12) | (1 << CS10);  // Set CTC mode and prescaler to 1
  TCNT1 = 0;
  OCR1A = 15999; //Counter for 1KHz interrupt 16*10^6/1000-1 no prescaler
  TIMSK1 |= (1 << OCIE1A); //Compare interrupt mode
  sei();  // Enable global interrupts

  #if TRAJECTORY
    #if DEBUG_WAYPOINT
      Serial.println("noOfWaypoints:");
      Serial.println(wayPoints.noOfWayPoints);
      for(int i=0;i<wayPoints.noOfWayPoints;++i){
        Serial.println(wayPoints.wayPoints[i],4);
        delay(10);
      }
    #endif
  #endif
  // pwmWrite(100,0);
  // delay(50);
  // pwmWrite(35,0);
  // delay(50);
  // pwmWrite(30,0);
}

volatile unsigned long current_time = 0;
volatile float elapsed_time = 0;
unsigned int controlActionStartTime = 0;
unsigned int controlActionEndTime = 0;
float integratedTargetDistance = 0;

//right motor and left motor work well with this
static const constexpr float alpha = 0.1;
static inline void filterVelocity(float& raw,float& filter)
{
  filter = filter + alpha*((raw - filter));
}
// the loop function runs over and over again forever
void loop() {
  /**
    Calculate velocity every 2ms
  */
  auto diff = globalTimer.globalTimeInMs - globalTimer.velCalcPrevTime;
  if ((diff>=intervals.velocityCalculationInterval)) {
    cli();
    globalTimer.velCalcPrevTime = globalTimer.globalTimeInMs;
    currentVelocity.rawRight = getRPSFromTicks(getRotationsFromTicks(encoderData.rightEncoderTicks),float(diff));
    currentVelocity.rawLeft = getRPSFromTicks(getRotationsFromTicks(encoderData.leftEncoderTicks),float(diff));
    filterVelocity(currentVelocity.rawRight,currentVelocity.filteredRightVel);
    filterVelocity(currentVelocity.rawLeft,currentVelocity.filteredLeftVel);
    encoderData.rightEncoderTicks=0;
    encoderData.leftEncoderTicks=0;
    sei();
  }
/**
Control loop sub thread every 4ms
**/
  if ((globalTimer.globalTimeInMs - globalTimer.controlLoopPrevTime >=10)){
    controlLoop();
  }
/**
Subthread to debug print every second
*/ 
  if((globalTimer.globalTimeInMs - globalTimer.debugPrevTime)>=intervals.debugPrintInterval){

    #if DEBUG_ENCODER_TICK
    Serial.println("Encoder ticks:");
    Serial.print(encoderData.leftEncoderTicks);
    Serial.print("\t");
    Serial.println(encoderData.rightEncoderTicks);
    #endif

    #if DEBUG_ENCODER_DIRECTION_PIN
    Serial.println("Encoder directionPins:");
    Serial.print(encoderData.leftEncoderDirection);
    Serial.print("\t");
    Serial.println(encoderData.rightEncoderDirection);
    #endif

    #if DEBUG_VELOCITIES
    Serial.println("Left vs Right motor Velocities:");
    Serial.print(currentVelocity.rawLeft,5);
    Serial.print("\t");
    Serial.println(currentVelocity.filteredRightVel,5);
    #endif

    #if DEBUG_CONTROL
    // Serial.println("Left Motor Control values:output vs error wrt isControllerRunning:");
    // Serial.print(leftMotor.motor_output,5);
    // Serial.print("\t");
    // Serial.print(leftMotor.motor_error,5);
    // Serial.print("\t");
    // Serial.println(startControl);
    // Serial.println("Right Motor Control values:output vs error wrt isControllerRunning:");
    // Serial.print(rightMotor.motor_output,5);
    // Serial.print("\t");
    // Serial.print(rightMotor.motor_error,5);
    // Serial.print("\t");
    // Serial.println(startControl);
    if(!startControl){
      Serial.print("Control action Time: ");
      Serial.println(controlActionEndTime-controlActionStartTime);
      Serial.print("Integrated Distance: ");
      Serial.println(integratedTargetDistance,5);
      startControl = false;
    }
    #endif
    globalTimer.debugPrevTime = globalTimer.globalTimeInMs;
  }
}
//Returns Radians per second velocity from rotations in 1Khz Intervals
//1 sec = 1000 ms //Encoder is on Motor Shaft
static inline float getRPSFromTicks(const float& rotations,const float& interval){
  return (float)rotations* rotationToRadians * ((float)(1000.0)/(float)interval);
}
static inline float getRotationsFromTicks(const int ticks){
  return  (float)(ticks/sysCons.ticksPerRevOutput);
}

// Writes actual PWM values to the motor
// Has a safety saturation check
void pwmWrite(uint8_t pwmL, uint8_t pwmR){
  uint8_t limit = 180;
  if(pwmL>=limit){
    pwmL = limit;
  }
  if(pwmR>=limit){
    pwmR = limit;
  }
  analogWrite(motorControl.leftMotorSpeedPin,pwmL); // Left wheel speed
  analogWrite(motorControl.rightMotorSpeedPin,pwmR); // Right wheel speed
}
// leftDirection && rightDirection == 1 ==> Forward direction
void setDirection(bool leftDirection,bool rightDirection){
  bool leftMotor=false;
  bool rightMotor=false;
  (leftDirection==true)?leftMotor = motorControl.leftMotorForwardState: leftMotor = !motorControl.leftMotorForwardState;
  (rightDirection==true)?rightMotor = motorControl.rightMotorForwardState: rightMotor = !motorControl.rightMotorForwardState;
  digitalWrite(motorControl.leftMotorDirectionPin,leftMotor); // Left wheel, LOW for forward
  digitalWrite(motorControl.rightMotorDirectionPin,rightMotor); // Right wheel, HIGH for forward
}
//Left motor Reverse Direction pin val =  1
void leftMotorISR(){
  encoderData.leftEncoderDirection = digitalRead(encoderPins.leftEncDirection);
  (encoderData.leftEncoderDirection==1)?encoderData.leftEncoderTicks-=1:encoderData.leftEncoderTicks+=1;
}
//Right motor Forward Direction pin val = 1
void rightMotorISR(){
  encoderData.rightEncoderDirection = digitalRead(encoderPins.rightEncDirection);
  (encoderData.rightEncoderDirection==1)?encoderData.rightEncoderTicks+=1:encoderData.rightEncoderTicks-=1;
}


/**
Control loop
**/
static void controlAction(ControlLoopVariables& motor){
  //Calculate Error
  motor.motor_error = motor.motor_setpoint - motor.motor_input;
  //calculate integral and derivative terms for each motor
  motor.motor_integral += motor.motor_error * elapsed_time; 
  motor.motor_integral = constrain(motor.motor_integral, -1*motor.motorGains->iSat, motor.motorGains->iSat);
  motor.motor_derivative = (motor.motor_error - motor.motor_prev_error) / elapsed_time;
  motor.motor_prev_error = motor.motor_error;
  //Calculate Outut
  motor.motor_output = motor.motorGains->kp * motor.motor_error +motor.motorGains->ki * motor.motor_integral + motor.motorGains->kd * motor.motor_derivative;
}

template <typename T> 
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
#if TRAJECTORY
int wayPointCounter = 0;
#endif
void controlLoop() {
  if(startControl){  
    #if TRAJECTORY
      if(wayPointCounter==0){
        controlActionStartTime = (unsigned int)globalTimer.globalTimeInMs;
        integratedTargetDistance = 0;
      }
    #endif
    // Calculate elapsed time
    current_time = globalTimer.globalTimeInMs;
    elapsed_time = (float)(current_time - globalTimer.controlLoopPrevTime ) / 1000.0;
    globalTimer.controlLoopPrevTime = current_time;

    // Set Input as Left and Right Motor velocity
    leftMotor.motor_input = currentVelocity.filteredLeftVel;
    rightMotor.motor_input = currentVelocity.filteredRightVel;

    #if TRAJECTORY

    //Set local waypoint:

    leftMotor.motor_setpoint = wayPoints.wayPoints[wayPointCounter];
    rightMotor.motor_setpoint = wayPoints.wayPoints[wayPointCounter];

    //Integrate velocity to find distance
    //Integrate Target
    // integratedTargetDistance += convertRPSToLinear(leftMotor.motor_setpoint)*elapsed_time;
    //Integrate Right Motor
    integratedTargetDistance +=convertRPSToLinear(rightMotor.motor_input)*elapsed_time;
    //Integrate Left Motor
    //integratedTargetDistance +=convertRPSToLinear(leftMotor.motor_input)*elapsed_time;
    #endif

    // run control loop and calculate output
    controlAction(leftMotor);
    controlAction(rightMotor);

    //Figure out direction of voltage and magnitude of voltage, and then apply it
    auto leftSign = sgn(leftMotor.motor_output);
    auto rightSign = sgn(rightMotor.motor_output);

    // Send the output to the motors
    setDirection((leftSign==1)?true:false,(rightSign==1)?true:false);
    pwmWrite((uint8_t)(leftMotor.motor_output*(float)leftSign),(uint8_t)(rightMotor.motor_output*(float)rightSign));

#if TRAJECTORY
    if(wayPointCounter<wayPoints.noOfWayPoints){
      ++wayPointCounter;
    }else{
      // Reset Everything
      controlActionEndTime = (unsigned int)globalTimer.globalTimeInMs;
      startControl = false;
      wayPointCounter = 0;
      setDirection(motorControl.leftMotorForwardState,motorControl.rightMotorForwardState);
      pwmWrite(0,0);
      leftMotor.resetController();
      rightMotor.resetController();
    }
#endif
  }
}

/**
1 khz timer tick counter on Timer 1
*/ 
ISR(TIMER1_COMPA_vect) {
  globalTimer.globalTimeInMs+=1;
}

