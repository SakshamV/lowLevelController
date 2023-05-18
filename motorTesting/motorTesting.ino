// #include <ros.h>
// #include <geometry_msgs/Pose2D.h>
// #include <terpbot_msgs/Gains.h>
// #include <std_msgs/Bool.h>
#include "./UARTParser.h"

#define DEBUG_VELOCITIES 0
#define TRAJECTORY 0
#define DEBUG_CONTROL 0
#define PRINT_ON_ROS 0
#define ENABLE_CONTROL 1
#define ENABLE_ROS 0
#define CALC_VEL 0

template<typename T>
class GlobalTime {
  private:
    volatile bool overflow_flag=false;
    T globalTimeInTicks;
    static const constexpr unsigned long inputFreq = 16000000;
    static const constexpr unsigned int prescaler = 1024;
    uint16_t noOfOverFlows = 0;
    static const constexpr float time_per_tick_ms = ((float)1000/(double)(inputFreq/prescaler));
    static const constexpr float time_per_ovf_ms = 0xFFFF * time_per_tick_ms;
  public:
    T getGlobalTimeinTicks() {
      // Making copies of these values to reduce data races
      auto tcnt1 = TCNT1;
      
      // Disable timer interrupts      
      TIMSK1 = 0;
      auto ovfs = noOfOverFlows;
      // Enable timer interrupts
      TIMSK1 |= (1 << TOIE1);
      
      this->globalTimeInTicks =((uint32_t)ovfs << 16) | tcnt1;
      return this->globalTimeInTicks;
    }
    static constexpr float convertTicksToTimeMs(uint32_t ticks){
        const auto overflows = ( (ticks & 0xFFFF0000) >>16);
        const auto tcnt1 = (ticks & 0x0000FFFF);
        return (float(tcnt1)*time_per_tick_ms + float(time_per_ovf_ms)*overflows);
    }
    static constexpr uint32_t convertTimeMsToTicks(uint16_t time_in_ms){
      const auto noOfOverflows = uint32_t(uint32_t(time_in_ms*1000) / uint32_t(time_per_ovf_ms*1000));
      const auto rem = uint32_t(uint32_t(time_in_ms) % uint32_t(time_per_ovf_ms));
      const auto tcnt1 = uint32_t(uint32_t(rem*10000)/uint32_t(time_per_tick_ms*10000));
      return ((uint32_t)noOfOverflows << 16) | tcnt1;
    }
    inline void incrementOvf(){
      this->noOfOverFlows+=1;
    }
    uint16_t getOvf(){
      return this->noOfOverFlows;
    }
    uint16_t getTCNT1(){
      return TCNT1;
    }


  T debugPrevTime;
  T rosSpinPrevTime;
  T controlLoopPrevTime;
  GlobalTime()
  : globalTimeInTicks(T(0)),
  debugPrevTime(T(0)),
  rosSpinPrevTime(T(0)),
  controlLoopPrevTime(T(0))
  {
    ;
  }
};


static GlobalTime<unsigned long> globalTimer;

typedef struct SystemConstants {
  unsigned long baudRate;
  float ticksPerRevOutput;
  uint8_t controlFreqInHz;  // minimum speed seen , 20 rpm => therefore, time for 1 tick is 6ms => choosing control freq 20ms => Freq = 50Hz
  uint32_t controlTimeInTicks;
  constexpr SystemConstants(unsigned long baud, float ticksPerRevOp, uint8_t controlFreq)
    : baudRate(baud),
      ticksPerRevOutput(ticksPerRevOp),
      controlFreqInHz(controlFreq),
      controlTimeInTicks(GlobalTime<uint32_t>::convertTimeMsToTicks(uint16_t((float)10000/ this->controlFreqInHz))/10) {}
} SystemConstants;

static constexpr const SystemConstants sysCons = SystemConstants(2000000, 495, 75);

typedef struct MotorControl {
  uint8_t leftMotorDirectionPin;
  uint8_t rightMotorDirectionPin;
  bool leftMotorForwardState;
  bool rightMotorForwardState;
  uint8_t leftMotorSpeedPin;
  uint8_t rightMotorSpeedPin;
} MotorControl;

static constexpr const MotorControl motorControl = {
  4, 7, LOW, HIGH, 5, 6
};

typedef struct EncoderData {
  int64_t rightEncoderPrev;
  int64_t leftEncoderPrev;
  
  int64_t leftEncoderTicks;
  int64_t rightEncoderTicks;

  uint8_t leftEncoderDirection;
  uint8_t rightEncoderDirection;
} EncoderData;

static EncoderData encoderData = {
  0, 0, .leftEncoderTicks = 0, .rightEncoderTicks = 0, .leftEncoderDirection = 0, .rightEncoderDirection = 0
};

typedef struct EncoderPins {
  uint8_t leftEnc;
  uint8_t leftEncDirection;
  uint8_t rightEnc;
  uint8_t rightEncDirection;
} EncoderPins;

static constexpr const EncoderPins encoderPins = {
  .leftEnc = 2, .leftEncDirection = 10, .rightEnc = 3, .rightEncDirection = 11
};

template<typename T>
class MotorState {
  public:
  T filteredRightVel=T(0);
  T filteredLeftVel=T(0);
  private:
  T rawRight=T(0);
  T rawLeft =T(0);
  public:
  MotorState(
  ){
  }  
  void reset() {
    filteredRightVel = 0;
    filteredLeftVel = 0;
  }
  T getRawRight() {
    return this->rawRight;
  }
  T getRawLeft() {
    return this->rawLeft;
  }
  T getFilteredRightVel() {
    return this->filteredRightVel;
  }
  T getFilteredLeftVel() {
    return this->filteredLeftVel;
  }
  void setFilteredLeftVel(T filterVal){
    return this->filteredLeftVel = filterVal;
  }
  void setFilteredRightVel(T filterVal){
    return this->filteredRightVel = filterVal;
  }
  void setRawRight(T rawRight) {
    this->rawRight = rawRight;
  }
  void setRawLeft(T rawLeft) {
    this->rawLeft = rawLeft;
  }
};

// static MotorState<float> currentVelocity;

/**
Everything here is in millisecodns
*/
typedef struct Intervals {
  uint32_t rosSpinRate;         // in Ticks From Milliseconds
  uint32_t debugPrintInterval;  // in Ticks From Milliseconds
} Intervals;


static const constexpr Intervals intervalsTicks = {
  .rosSpinRate = ((sysCons.controlTimeInTicks)),
  .debugPrintInterval = GlobalTime<unsigned long>::convertTimeMsToTicks(1000)
};


typedef struct MotorsGain {
  MotorsGain(double kp,double ki,double kd,double isat, double kpr,double kir, double kdr,double isatr){
    leftMotorGains.kp = kp;
    leftMotorGains.ki = ki;
    leftMotorGains.kd = kd;
    leftMotorGains.iSat = isat;

    rightMotorGains.kp = kpr;
    rightMotorGains.ki = kir;
    rightMotorGains.kd = kdr;
    rightMotorGains.iSat = isatr;
  }
  Gains leftMotorGains;
  Gains rightMotorGains;
} MotorsGain;

static auto motorGains = MotorsGain(19.5,263.33,0.361,100.0,21.0,283.592,0.595,100.0);

typedef struct ControlLoopVariables {
  float motor_setpoint = 0;  // Setpoint and input have to be in ticks per control_freq_time_period
  float motor_input = 0;     // 
  float motor_output = 0;
  float motor_error = 0;
  float motor_prev_error = 0;
  float motor_integral = 0;
  float motor_derivative = 0;
  ControlLoopVariables(Gains * const ipGains)
    : motorGains(ipGains) {
    ;
  }

  Gains* const motorGains;
  void resetController() {
    motor_setpoint = 0;
    motor_input = 0;
    motor_output = 0;
    motor_error = 0;
    motor_prev_error = 0;
    motor_integral = 0;
    motor_derivative = 0;
  }
}ControlLoopVariables;


#if CALC_VEL
  typedef struct VelCalculator {
    uint8_t currCon = 0;
    float oldTime = 0;
    void setOldTime(float oldTime){
      this->oldTime = oldTime;
    }
    float getOldTime(){
      return this->oldTime;
    }
  } VelCalculator;
  VelCalculator left;
  VelCalculator right;
#endif 


static Target globalTarget;
static const constexpr float rotationToRadians = 6.28318;  // 1 rotation is 2Pi radians
static ControlLoopVariables leftMotor(&motorGains.leftMotorGains);
static ControlLoopVariables rightMotor(&motorGains.rightMotorGains);
volatile bool startControl = false;

template<typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

#if TRAJECTORY
  // Utility functions to convert linear velocity to radians per sec, and vice versa
  static constexpr float convertLinearToRPS(const float& linearVelocity) {
    return linearVelocity * 15.503875969;
  }
  static constexpr float convertRPSToLinear(const float& RPS) {
    return RPS * (1 / 15.503875969);
  }
  // This section generates a parabolic trajectory in case TRAJECTORY is set to 1
  // wheel diameter 64.5mm
  static constexpr const int NO_OF_WAYPOINTS = 350;
  //Diameter 64.5 mm
  static const constexpr float vmax = 0.406;  //(m/s) actually some 0.406~
  static constexpr float getXLinearVelocity(const float& t, const float& T) {
    return t * (4 * vmax / T) + (-(4 * vmax / (T * T)) * t * t);
  }
  static constexpr float getT(const float& xInitial, const float& xFinal) {
    return (6.0 / (4.0 * vmax)) * (xFinal - xInitial);
  }
  template<int Max>
  struct Trajectory {
    constexpr Trajectory()
      : wayPoints(), noOfWayPoints(Max), totalTime(0) {
      float time = 0.0;
      const constexpr float loopRate = controlFreqInHz;
      const constexpr float timeIncrement = 1 / loopRate;
      const constexpr float xInit = 0;
      const constexpr float xFinal = 0.20263272615;  // 1 rotation of the wheel //0.20263272615
      const constexpr float totalTime = getT(xInit, xFinal);
      int i = 0;
      while (true) {
        this->wayPoints[i] = convertLinearToRPS(getXLinearVelocity(time, totalTime));
        time += timeIncrement;
        if (time > totalTime) {
          this->wayPoints[i] = 0;
          if (i < Max) {
            ++i;
          } else {
            this->wayPoints[i] = 0;
          }
          break;
        } else {
          if (i < Max) {
            ++i;
          } else {
            this->wayPoints[i] = 0;
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

/**
  ROS Variables and callbacks
**/
#if ENABLE_ROS

  #define TOPIC_NAME_1 "CURR_VEL"
  #define TOPIC_NAME_2 "TARG_VEL"
  #define TOPIC_NAME_3 "GAINS"

  static ros::NodeHandle nh;
  static geometry_msgs::Pose2D currentTickRate;
  static ros::Publisher currVelMsgPub(TOPIC_NAME_1, &currentTickRate);

  //140 rpm = 14.660765699999999

// Returns sign of the given number, +1 or -1

  void targetVelCallback(const geometry_msgs::Pose2D& msg) {
    target.leftMotorTarget = msg.x;
    target.rightMotorTarget = msg.y;
    if(msg.theta>0.5){
      startControl=true;
    }else{
      startControl=false;
      leftMotor.resetController();
      rightMotor.resetController();
      pwmWrite(0, 0);
    }
  }
  void setGainsCallback(const terpbot_msgs::Gains& msg){
    // True is interpreted as left motor
    // False is interpreted as right motor
    if(msg.isleft){
      leftMotor.motorGains->iSat = msg.i_clamp;
      leftMotor.motorGains->kp = msg.kp;
      leftMotor.motorGains->ki = msg.ki;
      leftMotor.motorGains->kd = msg.kd;
    }else{
      rightMotor.motorGains->iSat = msg.i_clamp;
      rightMotor.motorGains->kp = msg.kp;
      rightMotor.motorGains->ki = msg.ki;
      rightMotor.motorGains->kd = msg.kd;
    }
  }
  static ros::Subscriber<geometry_msgs::Pose2D> targetVelSub(TOPIC_NAME_2, targetVelCallback);
  static ros::Subscriber<terpbot_msgs::Gains> gainsSub(TOPIC_NAME_3, setGainsCallback);

#endif

void attachEncInt(){
  attachInterrupt(digitalPinToInterrupt(encoderPins.leftEnc), leftMotorISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPins.rightEnc), rightMotorISR, RISING);
}
void detachEncInt(){
  detachInterrupt(digitalPinToInterrupt(encoderPins.leftEnc));
  detachInterrupt(digitalPinToInterrupt(encoderPins.rightEnc));
}

void targetVelCallback(const Target& msg) {
  globalTarget.leftMotorTarget = msg.leftMotorTarget;
  globalTarget.rightMotorTarget = msg.rightMotorTarget;
  if(msg.theta>0.5){
    startControl=true;
  }else{
    startControl=false;
    leftMotor.resetController();
    rightMotor.resetController();
    pwmWrite(0, 0);
  }
}
void setGainsCallback(const Gains& msg){
  // True is interpreted as left motor
  // False is interpreted as right motor
  if(msg.isLeft){
    leftMotor.motorGains->iSat = msg.iSat;
    leftMotor.motorGains->kp = msg.kp;
    leftMotor.motorGains->ki = msg.ki;
    leftMotor.motorGains->kd = msg.kd;
  }else{
    rightMotor.motorGains->iSat = msg.iSat;
    rightMotor.motorGains->kp = msg.kp;
    rightMotor.motorGains->ki = msg.ki;
    rightMotor.motorGains->kd = msg.kd;
  }
}

static Gains gains;
static Target targ;

void process(uint8_t* buffer){
  if(buffer[1]==tags[Tags::TARGET]){
    targ.deserialize(buffer);
    targetVelCallback(targ);
  }else if(buffer[1]==tags[Tags::GAINS]){
    gains.deserialize(buffer);
    setGainsCallback(gains);
  }
}

enum SpinStates{
  First,
  TopicName,
  Data,
  End  
};

static SpinStates spinStates=SpinStates::First;
uint8_t in_buf[50];
void spinOnce(){
  uint8_t i=0;
  auto ctime = globalTimer.getGlobalTimeinTicks();
  while (true){
      if(globalTimer.getGlobalTimeinTicks() - ctime > globalTimer.convertTimeMsToTicks(5)){
        spinStates = SpinStates::First;
        break;
      }
      int data = readUart();
      if (data < 0){
        spinStates = SpinStates::First;
        break;
      }
      if(spinStates==SpinStates::First){
        if(data=='{'){
          in_buf[i] = data;
          spinStates = SpinStates::TopicName;
          i+=1;          
        }
      }else if(spinStates==SpinStates::TopicName){
        in_buf[i]=data;
        i++;
        spinStates = SpinStates::Data;
      }else if(spinStates==SpinStates::Data){
        if(i>20){
          spinStates=SpinStates::First;
          break;          
        }
        if(data=='}'){
          spinStates = SpinStates::End;
          i++;        
        }else{
          in_buf[i]=data;
          i++;          
        }     
      }else if(spinStates==SpinStates::End){
        if(data=='\n'){
          in_buf[i]=data;
          process(in_buf);
        }
        spinStates=SpinStates::First;
        break;
      }
  }
}

// the setup function runs once when you press reset or power the board
void setup() {
  // Setup serial output:
  Serial.begin(sysCons.baudRate);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup Tick counting
  pinMode(encoderPins.leftEnc, INPUT_PULLUP);
  pinMode(encoderPins.rightEnc, INPUT_PULLUP);
  pinMode(encoderPins.leftEncDirection, INPUT);
  pinMode(encoderPins.rightEncDirection, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPins.leftEnc), leftMotorISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPins.rightEnc), rightMotorISR, RISING);

  //Setup Motor direction pins
  pinMode(motorControl.leftMotorDirectionPin, OUTPUT);
  pinMode(motorControl.rightMotorDirectionPin, OUTPUT);

  //Setup Motor Speed Control pins
  pinMode(motorControl.leftMotorSpeedPin, OUTPUT);
  pinMode(motorControl.rightMotorSpeedPin, OUTPUT);

  // Currently, Set them up for the bot to go forward
  digitalWrite(motorControl.leftMotorDirectionPin, motorControl.leftMotorForwardState);    // Left wheel, LOW for forward
  digitalWrite(motorControl.rightMotorDirectionPin,motorControl.rightMotorForwardState);  // Right wheel, HIGH for forward
  /**
  TIMER Interrupts at Overflow
  We count overflows and get the time by overflow_time*
  */
  cli();  // Disable global interrupts
  TCCR1A = 0;

  // Set up Timer to tick at prescaler of 1024
  // This means time for each time in milliseconds = (1/(16000000/1024)) * 1000 = 0.064 milliseconds
  // This means time for 2^16 ticks if 4194.304 milliseconds
  TCCR1B = (1 << CS12)|(1 << CS10); 

  // Enable TIMER1 overflow interrupt
  TIMSK1 |= (1 << TOIE1);
  sei();  // Enable global interrupts

  #if TRAJECTORY
    #if DEBUG_WAYPOINT
      Serial.println("noOfWaypoints:");
      Serial.println(wayPoints.noOfWayPoints);
      for (int i = 0; i < wayPoints.noOfWayPoints; ++i) {
        Serial.println(wayPoints.wayPoints[i], 4);
        delay(10);
      }
    #endif
  #endif

  #if ENABLE_ROS
    nh.getHardware()->setBaud(sysCons.baudRate);
    nh.initNode();
    nh.advertise(currVelMsgPub);
    nh.subscribe(targetVelSub);
    nh.subscribe(gainsSub);
    while(!nh.connected()){
      nh.spinOnce();
    }
  #endif
}

#if ENABLE_CONTROL
  volatile float elapsed_time = 0;
  float integratedTargetDistance = 0;
#endif

#if TRAJECTORY
  float controlActionStartTime = 0;
  float controlActionEndTime = 0;
#endif



// Right motor and Left motor work well with this value
// As of this commit,they are not needed.
static const constexpr float alpha = 0.7;
static inline float filterVelocity(const float raw, float& filter) {
  return filter = filter + alpha * ((raw - filter));
}

// static CurrentTickRate currentTickRate;
// the loop function runs over and over again forever
static CurrentTickRate currentTickRate;
void loop() {

  /**
  Control loop sub thread every 
  **/
  #if ENABLE_CONTROL
    auto ctrlInterval = (globalTimer.getGlobalTimeinTicks() - globalTimer.controlLoopPrevTime);
    if (( ctrlInterval >= sysCons.controlTimeInTicks)) {
      globalTimer.controlLoopPrevTime = globalTimer.getGlobalTimeinTicks();
      controlLoop(ctrlInterval);
      // Get the current velocity as a function of ticks passed in control loop time
      // Making a copy on purpose
      // detachEncInt();
      auto leftEncTicks = encoderData.leftEncoderTicks;
      auto rightEncTicks = encoderData.rightEncoderTicks; 
      // attachEncInt();
           
      currentTickRate.leftTickRate = float(leftEncTicks - encoderData.leftEncoderPrev);
      currentTickRate.rightTickRate = float(rightEncTicks - encoderData.rightEncoderPrev);

      encoderData.leftEncoderPrev = leftEncTicks;
      encoderData.rightEncoderPrev = rightEncTicks;
  
      publishCustomMsg( currentTickRate );
    }
  #endif
  /**
  ROS SPIN subthread
  **/
  if ( (globalTimer.getGlobalTimeinTicks() - globalTimer.rosSpinPrevTime) >= (intervalsTicks.rosSpinRate)/2) {
    globalTimer.rosSpinPrevTime = globalTimer.getGlobalTimeinTicks();
    spinOnce();
  }
  /**
  Subthread to debug print every second
  */
  auto dbgInterval = ((globalTimer.getGlobalTimeinTicks() - globalTimer.debugPrevTime));
  if ( dbgInterval>= intervalsTicks.debugPrintInterval) {

  globalTimer.debugPrevTime = globalTimer.getGlobalTimeinTicks();

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
    Serial.print(currentVelocity.getFilteredLeftVel(), 5);
    Serial.print("\t");
    Serial.print(currentVelocity.getFilteredRightVel(), 5);
    Serial.print("\t");
    Serial.println(globalTimer.convertTicksToTimeMs(dbgInterval), 5);
  #endif

  #if DEBUG_CONTROL
    if (!startControl) {
      Serial.print("Control action Time: ");
      Serial.println(controlActionEndTime - controlActionStartTime);
      Serial.print("Integrated Distance: ");
      Serial.println(integratedTargetDistance, 5);
      startControl = false;
    }
  #endif
  }
}
//Returns Radians per second velocity from rotations in 1Khz Intervals
//1 sec = 1000 ms //Encoder is on Motor Shaft
static inline float getRPSFromTicks(const float& rotations, const float& intervalMs) {
  auto abs_interval = (float)fabs(intervalMs);
  if(abs_interval<0.4){
    return 0;
  }
  if(abs_interval>10){
    return 0;
  }
  return (float)rotations * rotationToRadians * ((float)(1000.0) / (float)(abs_interval));
}
static inline float getRotationsFromTicks(const int8_t& ticks) {
  return (float(ticks) / sysCons.ticksPerRevOutput);
}

// Writes actual PWM values to the motor
// Has a safety saturation check
void pwmWrite(uint8_t pwmL, uint8_t pwmR) {
  uint8_t limit = 255;
  if (pwmL >= limit) {
    pwmL = limit;
  }
  if (pwmR >= limit) {
    pwmR = limit;
  }
  analogWrite(motorControl.leftMotorSpeedPin, pwmL);   // Left wheel speed
  analogWrite(motorControl.rightMotorSpeedPin, pwmR);  // Right wheel speed
}
// leftDirection && rightDirection == 1 ==> Forward direction
void setDirection(bool leftDirection, bool rightDirection) {
  bool leftMotor = false;
  bool rightMotor = false;
  (leftDirection == true) ? leftMotor = motorControl.leftMotorForwardState : leftMotor = !motorControl.leftMotorForwardState;
  (rightDirection == true) ? rightMotor = motorControl.rightMotorForwardState : rightMotor = !motorControl.rightMotorForwardState;
  digitalWrite(motorControl.leftMotorDirectionPin, leftMotor);    // Left wheel, LOW for forward
  digitalWrite(motorControl.rightMotorDirectionPin, rightMotor);  // Right wheel, HIGH for forward
}

#define VELC_COUNTER 1  // wait for 1 ticks

#if CALC_VEL
// if left motor, set true
static void calculateVelocity(EncoderData& encoderData, MotorState<float>& currVel, VelCalculator& velCal, const bool left) {
  if (velCal.currCon == VELC_COUNTER) {
    auto currTime = globalTimer.getGlobalTimeinTicks();
    auto interval = currTime - velCal.getOldTime();

    // if( (interval<0.4) || (interval>10) ){
    //   interval = 0;
    // }else{
    //   interval = 1/interval;
    // }
    if (left) {
      currVel.setRawLeft( float(encoderData.leftEncoderTicks)*(1/interval) );
      encoderData.leftEncoderTicks = 0;
    } else {
      currVel.setRawRight(float(encoderData.rightEncoderTicks)*(1/interval ));
      encoderData.rightEncoderTicks = 0;
    }
    velCal.currCon = 0;
    velCal.setOldTime(currTime);
  }
}
#endif 

//Left motor Reverse Direction pin val =  1
void leftMotorISR() {
  encoderData.leftEncoderDirection = digitalRead(encoderPins.leftEncDirection);
  (encoderData.leftEncoderDirection == 1) ? encoderData.leftEncoderTicks -= 1 : encoderData.leftEncoderTicks += 1;
}

void rightMotorISR() {
  encoderData.rightEncoderDirection = digitalRead(encoderPins.rightEncDirection);
  (encoderData.rightEncoderDirection == 1) ? encoderData.rightEncoderTicks += 1 : encoderData.rightEncoderTicks -= 1;
}


/**
Control loop
**/
#if ENABLE_CONTROL
static void controlAction(ControlLoopVariables& motor) {
  //Calculate Error
  motor.motor_error = motor.motor_setpoint - motor.motor_input;
  //calculate integral and derivative terms for each motor
  motor.motor_integral += motor.motor_error * elapsed_time;
  motor.motor_integral = constrain(motor.motor_integral, -1 * motor.motorGains->iSat, motor.motorGains->iSat);
  motor.motor_derivative = (motor.motor_error - motor.motor_prev_error) / elapsed_time;
  motor.motor_prev_error = motor.motor_error;
  //Calculate Outut
  motor.motor_output = motor.motorGains->kp * motor.motor_error + motor.motorGains->ki * motor.motor_integral + motor.motorGains->kd * motor.motor_derivative;
}


#if TRAJECTORY
  int wayPointCounter = 0;
#endif
float leftFilt = 0;
float rightFilt = 0;
void controlLoop(unsigned long& time_delta) {
  if (startControl) {
  #if TRAJECTORY
    if (wayPointCounter == 0) {
      controlActionStartTime = (unsigned int)globalTimer.getGlobalTimeinMs();
      integratedTargetDistance = 0;
    }
  #endif
    // Calculate elapsed time in seconds
    elapsed_time = GlobalTime<unsigned long>::convertTicksToTimeMs(time_delta) / 1000.0;

    // Set Input as Left and Right Motor velocity
    // filterVelocity(currentVelocity.rawRight, currentVelocity.filteredRightVel);

    // filterVelocity(currentVelocity.rawLeft, currentVelocity.filteredLeftVel);
    leftMotor.motor_input = filterVelocity(currentTickRate.leftTickRate,leftFilt);
    rightMotor.motor_input = filterVelocity(currentTickRate.rightTickRate,rightFilt);

#if TRAJECTORY
    //Set local waypoint:
    leftMotor.motor_setpoint = wayPoints.wayPoints[wayPointCounter];
    rightMotor.motor_setpoint = wayPoints.wayPoints[wayPointCounter];
#endif

    leftMotor.motor_setpoint = globalTarget.leftMotorTarget;
    rightMotor.motor_setpoint = globalTarget.rightMotorTarget;

//Integrate velocity to find distance
  #if INT_TARGET
    //Integrate Target
    integratedTargetDistance += convertRPSToLinear(leftMotor.motor_setpoint) * elapsed_time;
  #endif
  #if INT_RIGHT
    //Integrate Right Motor
    integratedTargetDistance += convertRPSToLinear(rightMotor.motor_input) * elapsed_time;
  #endif
  #if INT_LEFT
    //Integrate Left Motor
    integratedTargetDistance += convertRPSToLinear(leftMotor.motor_input) * elapsed_time;
  #endif

    // Run control loop and calculate output
    controlAction(leftMotor);
    controlAction(rightMotor);

    //Figure out direction of voltage and magnitude of voltage, and then apply it
    auto leftSign = sgn(leftMotor.motor_output);
    auto rightSign = sgn(rightMotor.motor_output);

    // Send the output to the motors
    setDirection((leftSign == 1) ? true : false, (rightSign == 1) ? true : false);
    auto absLeft = (leftMotor.motor_output * float(leftSign));
    auto absRight = (rightMotor.motor_output * float(rightSign));
    pwmWrite(uint8_t( absLeft ) , uint8_t(float(absRight)));

#if TRAJECTORY
    if (wayPointCounter < wayPoints.noOfWayPoints) {
      ++wayPointCounter;
    } else {
      // Reset Everything
      controlActionEndTime = (unsigned int)globalTimer.getGlobalTime;
      startControl = false;
      wayPointCounter = 0;
      setDirection(motorControl.leftMotorForwardState, motorControl.rightMotorForwardState);
      pwmWrite(0, 0);
      leftMotor.resetController();
      rightMotor.resetController();
    }
#endif
  }
}
#endif

/**
 Timer interrupts every 1 overflow
*/
ISR(TIMER1_OVF_vect) {
  globalTimer.incrementOvf();
}
