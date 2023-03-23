#include <ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>

#define DEBUG_VELOCITIES 1
#define TRAJECTORY 0
#define DEBUG_CONTROL 0
#define PRINT_ON_ROS 1
#define ENABLE_CONTROL 0
#define DISABLE_ROS 1

typedef struct SystemConstants {
  unsigned long baudRate;
  float ticksPerRevOutput;
  uint8_t controlFreqInHz;  // minimum speed seen , 20 rpm => therefore, time for 1 tick is 6ms => choosing control freq 20ms => Freq = 50Hz
  float controlTimeinMs;
  constexpr SystemConstants(unsigned long baud, float ticksPerRevOp, uint8_t controlFreq)
    : baudRate(baud),
      ticksPerRevOutput(ticksPerRevOp),
      controlFreqInHz(controlFreq),
      controlTimeinMs((float)1000 / this->controlFreqInHz) {}
} SystemConstants;

static constexpr const SystemConstants sysCons = SystemConstants(2000000, 495, 50);

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
  uint8_t rightEncoderPrev;
  uint8_t leftEncoderPrev;
  uint8_t leftEncoderTicks;
  uint8_t rightEncoderTicks;
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
struct MotorState {
  T filteredRightVel;
  T filteredLeftVel;
  T rawRight;
  T rawLeft;
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
  void setRawRight(T rawRight) {
    this->rawRight = rawRight;
  }
  void setRawLeft(T rawLeft) {
    this->rawLeft = rawLeft;
  }
};

static struct MotorState<float> currentVelocity = { .filteredRightVel = 0, .filteredLeftVel = 0, 0, 0 };

template<typename T>
struct GlobalTime {
  T globalTimeInMs;
  T debugPrevTime;
  T velCalcPrevTime;
  T controlLoopPrevTime;
  T getGlobalTimeinMs() {
    return this->globalTimeInMs;
  }
  void setGlobalTimeInMs(T globTime) {
    this->globalTimeInMs = globTime;
  }
};


static struct GlobalTime<float> globalTimer = {
  .globalTimeInMs = 0,
  .debugPrevTime = 0,
  .velCalcPrevTime = 0,
  .controlLoopPrevTime = 0
};

/**
Everything here is in millisecodns
*/
typedef struct Intervals {
  float rosSpinRate;         // in milliseconds
  float debugPrintInterval;  // in milliseconds
  float velCheckInterval; // in ms
} Intervals;


static const constexpr Intervals intervals = {
  .rosSpinRate = sysCons.controlTimeinMs, .debugPrintInterval = 1000,.velCheckInterval = 8
};

typedef struct Gains {
  float kp;
  float kd;
  float ki;
  float iSat;
} Gains;

typedef struct MotorsGain {
  Gains leftMotorGains;
  Gains rightMotorGains;
} MotorsGain;

static const constexpr MotorsGain motorGains = {
  .leftMotorGains = { 45, 0.0, 0.0, 50 }, .rightMotorGains = { 45, 0.0, 0.0, 50 }
};

typedef struct ControlLoopVariables {
  float motor_setpoint = 0;  // Velocity setpoint has to be in Radians per second
  float motor_input = 0;     // Velocity Input has to be in Radians per second
  float motor_output = 0;
  float motor_error = 0;
  float motor_prev_error = 0;
  float motor_integral = 0;
  float motor_derivative = 0;
  ControlLoopVariables(const Gains* ipGains)
    : motorGains(ipGains) {
    ;
  }

  const Gains* motorGains;
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

typedef struct Target {
  float leftMotorTarget;
  float rightMotorTarget;
}Target;

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

static Target target = { 0.0, 0.0 };
static const constexpr float rotationToRadians = 6.28318;  // 1 rotation is 2Pi radians
static ControlLoopVariables leftMotor(&motorGains.leftMotorGains);
static ControlLoopVariables rightMotor(&motorGains.rightMotorGains);
volatile bool startControl = false;

// Utility functions to convert linear velocity to radians per sec, and vice versa
static constexpr float convertLinearToRPS(const float& linearVelocity) {
  return linearVelocity * 15.503875969;
}
static constexpr float convertRPSToLinear(const float& RPS) {
  return RPS * (1 / 15.503875969);
}

// This section generates a parabolic trajectory in case TRAJECTORY is set to 1
// wheel diameter 64.5mm
#if TRAJECTORY
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
#if !DISABLE_ROS
#define TOPIC_NAME_1 "CURR_VEL"
#define TOPIC_NAME_2 "TARG_VEL"
static ros::NodeHandle nh;
geometry_msgs::Pose2D currentVel;
static ros::Publisher currVelMsgPub(TOPIC_NAME_1, &currentVel);
void targetVelCallback(const geometry_msgs::Pose2D& msg) {
  target.leftMotorTarget = msg.x;
  target.rightMotorTarget = msg.y;
  if (msg.theta > 0.5) {
    startControl = true;
  } else {
    startControl = false;
    leftMotor.resetController();
    rightMotor.resetController();
    pwmWrite(0, 0);
  }
}
static ros::Subscriber<geometry_msgs::Pose2D> targetVelSub(TOPIC_NAME_2, targetVelCallback);
#endif

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
  digitalWrite(motorControl.rightMotorDirectionPin, motorControl.rightMotorForwardState);  // Right wheel, HIGH for forward
  /**
  Setup timer1 for 1Khz frequency
  */
  cli();  // Disable global interrupts
  // Set TIMER1 prescaler to divide by 8, giving a 2MHz clock frequency
  TCCR1B = (1 << CS11);

  // Set TIMER1 to clear on match and set TOP to 199
  TCCR1A = 0;
  TCCR1B |= (1 << WGM12);
  OCR1A = 199;

  // Enable TIMER1 interrupt
  TIMSK1 |= (1 << OCIE1A);
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
#if !DISABLE_ROS
  nh.getHardware()->setBaud(sysCons.baudRate);
  nh.initNode();
  nh.advertise(currVelMsgPub);
  nh.subscribe(targetVelSub);
  delay(1000);
#endif
  pwmWrite(100, 100);
}

volatile float current_time = 0;
volatile float elapsed_time = 0;
#if TRAJECTORY
float controlActionStartTime = 0;
float controlActionEndTime = 0;
#endif
float integratedTargetDistance = 0;

// Right motor and Left motor work well with this
static const constexpr float alpha = 0.9;
static inline void filterVelocity(const float raw, float& filter) {
  filter = filter + alpha * ((raw - filter));
}

// the loop function runs over and over again forever
void loop() {
  if(globalTimer.getGlobalTimeinMs() - left.getOldTime() >= intervals.velCheckInterval){
    currentVelocity.setRawLeft(0.0);
    left.setOldTime(globalTimer.getGlobalTimeinMs());
  }
  if(globalTimer.getGlobalTimeinMs() - right.getOldTime() >= intervals.velCheckInterval){
    currentVelocity.setRawRight(0.0);
    right.setOldTime(globalTimer.getGlobalTimeinMs());
  }
  /**
  Control loop sub thread every 
  **/
  #if ENABLE_CONTROL
    if ((globalTimer.getGlobalTimeinMs() - globalTimer.controlLoopPrevTime >= sysCons.controlTimeinMs)) {
      controlLoop();
    }
  #endif
  /**
  ROS SPIN subthread
  **/
  if (globalTimer.getGlobalTimeinMs() - globalTimer.velCalcPrevTime >= intervals.rosSpinRate) {

    #if !DISABLE_ROS
    // #if PRINT_ON_ROS
    //   currentVel.x = target.leftMotorTarget;
    //   currentVel.y = target.rightMotorTarget;
    //   currVelMsgPub.publish( &currentVel );
    // #endif
    // nh.spinOnce();
    #endif
  }
  /**
  Subthread to debug print every second
  */
  if ((globalTimer.getGlobalTimeinMs() - globalTimer.debugPrevTime) >= intervals.debugPrintInterval) {

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
    Serial.print(currentVelocity.getRawLeft(), 5);
    Serial.print("\t");
    Serial.println(currentVelocity.getRawRight(), 5);
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
    globalTimer.debugPrevTime = globalTimer.getGlobalTimeinMs();
  }
}
//Returns Radians per second velocity from rotations in 1Khz Intervals
//1 sec = 1000 ms //Encoder is on Motor Shaft
static inline float getRPSFromTicks(const float& rotations, const float& intervalMs) {
  if(intervalMs<0.4){
    return 0;
  }
  return (float)rotations * rotationToRadians * ((float)(1000.0) / (float)intervalMs);
}
static inline float getRotationsFromTicks(const int& ticks) {
  return (float)(ticks / sysCons.ticksPerRevOutput);
}

// Writes actual PWM values to the motor
// Has a safety saturation check
void pwmWrite(uint8_t pwmL, uint8_t pwmR) {
  uint8_t limit = 190;
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

// if left motor, set true
static inline void calculateVelocity(EncoderData& encoderData, MotorState<float>& currVel, VelCalculator& velCal, bool left) {
  if (velCal.currCon == VELC_COUNTER) {
    auto currTime = globalTimer.getGlobalTimeinMs();
    if (left) {
      currVel.setRawLeft(getRPSFromTicks(getRotationsFromTicks(encoderData.leftEncoderTicks), float(currTime - velCal.oldTime)));
      encoderData.leftEncoderTicks = 0;
    } else {
      currVel.setRawRight(getRPSFromTicks(getRotationsFromTicks(encoderData.rightEncoderTicks), float(currTime - velCal.oldTime)));
      encoderData.rightEncoderTicks = 0;
    }
    velCal.currCon = 0;
    velCal.setOldTime(currTime);
  }
}

//Left motor Reverse Direction pin val =  1
void leftMotorISR() {
  left.currCon += 1;
  encoderData.leftEncoderDirection = digitalRead(encoderPins.leftEncDirection);
  (encoderData.leftEncoderDirection == 1) ? encoderData.leftEncoderTicks -= 1 : encoderData.leftEncoderTicks += 1;
  calculateVelocity(encoderData, currentVelocity, left, true);
}

void rightMotorISR() {
  right.currCon += 1;
  encoderData.rightEncoderDirection = digitalRead(encoderPins.rightEncDirection);
  (encoderData.rightEncoderDirection == 1) ? encoderData.rightEncoderTicks += 1 : encoderData.rightEncoderTicks -= 1;
  calculateVelocity(encoderData, currentVelocity, right, false);
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
  motor.motor_integral = constrain(motor.motor_integral, -1 * motor.motorGains.iSat, motor.motorGains.iSat);
  motor.motor_derivative = (motor.motor_error - motor.motor_prev_error) / elapsed_time;
  motor.motor_prev_error = motor.motor_error;
  //Calculate Outut
  motor.motor_output = motor.motorGains.kp * motor.motor_error + motor.motorGains.ki * motor.motor_integral + motor.motorGains.kd * motor.motor_derivative;
}

template<typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

#if TRAJECTORY
  int wayPointCounter = 0;
#endif

void controlLoop() {
  if (startControl) {
#if TRAJECTORY
    if (wayPointCounter == 0) {
      controlActionStartTime = (unsigned int)globalTimer.getGlobalTimeinMs();
      integratedTargetDistance = 0;
    }
#endif
    // Calculate elapsed time
    current_time = globalTimer.getGlobalTimeinMs();
    elapsed_time = (float)(current_time - globalTimer.controlLoopPrevTime) / 1000.0;
    globalTimer.controlLoopPrevTime = current_time;

    // Set Input as Left and Right Motor velocity
    filterVelocity(currentVelocity.rawRight, currentVelocity.filteredRightVel);

    filterVelocity(currentVelocity.rawLeft, currentVelocity.filteredLeftVel);

    rightMotor.motor_input = currentVelocity.getFilteredRightVel();
    leftMotor.motor_input = currentVelocity.getFilteredLeftVel();

#if TRAJECTORY
    //Set local waypoint:
    leftMotor.motor_setpoint = wayPoints.wayPoints[wayPointCounter];
    rightMotor.motor_setpoint = wayPoints.wayPoints[wayPointCounter];
#endif

    leftMotor.motor_setpoint = target.leftMotorTarget;
    rightMotor.motor_setpoint = target.rightMotorTarget;

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
    pwmWrite((uint8_t)(leftMotor.motor_output * (float)leftSign), (uint8_t)(rightMotor.motor_output * (float)rightSign));

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
10 khz timer tick counter on Timer 1
*/
ISR(TIMER1_COMPA_vect) {
  globalTimer.globalTimeInMs += 0.1;
}
