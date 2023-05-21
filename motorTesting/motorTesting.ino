/**
 * @file motorTesting.ino
 * \mainpage
*/
#include "./UARTParser.h"

#define DEBUG_VELOCITIES 0
#define TRAJECTORY 0
#define DEBUG_CONTROL 0
#define PRINT_ON_ROS 0
#define ENABLE_CONTROL 1
#define ENABLE_ROS 0
#define CALC_VEL 0

/**
 *@brief: Structure to manage time globally
 */
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
  T uartSpinPrevTime;
  T controlLoopPrevTime;
  GlobalTime()
  : globalTimeInTicks(T(0)),
  debugPrevTime(T(0)),
  uartSpinPrevTime(T(0)),
  controlLoopPrevTime(T(0))
  {
    ;
  }
};

/**
 * @brief: Used for maintaining time globally
*/
static GlobalTime<unsigned long> globalTimer;

/**
 *@brief: Structure to store System constants
 */
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

/**
 * @brief: System Constants, defined at compile time. Baud Rate = 2Mbps, ticks per revolution = 495, control frequency = 75Hz
*/
static constexpr const SystemConstants sysCons = SystemConstants(2000000, 495, 75);

/**
 *@brief: Structure to store Motor actuation details
 */
typedef struct MotorControl {
  uint8_t leftMotorDirectionPin;
  uint8_t rightMotorDirectionPin;
  bool leftMotorForwardState;
  bool rightMotorForwardState;
  uint8_t leftMotorSpeedPin;
  uint8_t rightMotorSpeedPin;
} MotorControl;

/**
 * @brief: Motor Actuation details: first 2 define the motor direction pins, next 2 define the voltage at motor direction pin which takes the robot forward, last 2 define the motor speed control pins
*/
static constexpr const MotorControl motorControl = {
  4, 7, LOW, HIGH, 5, 6
};

/**
 *@brief: Structure to store encoder data
 */
typedef struct EncoderData {
  int64_t rightEncoderPrev;
  int64_t leftEncoderPrev;
  
  int64_t leftEncoderTicks;
  int64_t rightEncoderTicks;

  uint8_t leftEncoderDirection;
  uint8_t rightEncoderDirection;
} EncoderData;

/**
 * @brief: Encoder data structure, stores encoder ticks, encoder direction state, and previous encoder ticks
*/
static EncoderData encoderData = {
  0, 0, .leftEncoderTicks = 0, .rightEncoderTicks = 0, .leftEncoderDirection = 0, .rightEncoderDirection = 0
};

/**
 *@brief: Structure to store encoder wiring details
 */
typedef struct EncoderPins {
  uint8_t leftEnc;
  uint8_t leftEncDirection;
  uint8_t rightEnc;
  uint8_t rightEncDirection;
} EncoderPins;

/**
 * @brief: Encoder wiring details: first 2 define the left encoder pins, next 2 define the right encoder pins
*/
static constexpr const EncoderPins encoderPins = {
  .leftEnc = 2, .leftEncDirection = 10, .rightEnc = 3, .rightEncDirection = 11
};

/**
 *@brief: Structure to store constant intervals as timer ticks converted from ms
 */
typedef struct Intervals {
  uint32_t uartSpinRate;         // in Ticks From Milliseconds
  uint32_t debugPrintInterval;  // in Ticks From Milliseconds
} Intervals;

/**
 * @brief: Intervals structure: UART spin rate is the same as control frequency, debug print interval = 1000ms
*/
static const constexpr Intervals intervalsTicks = {
  .uartSpinRate = ((sysCons.controlTimeInTicks)),
  .debugPrintInterval = GlobalTime<unsigned long>::convertTimeMsToTicks(1000)
};

/**
 *@brief: Structure to store left and right motor gains
*/
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

/**
 * @brief: Motor gains, initialized at run time
*/
static auto motorGains = MotorsGain(19.5,263.33,0.361,100.0,21.0,283.592,0.595,100.0);

/**
 *@brief: Structure to store control loop variables per motor
*/
typedef struct ControlLoopVariables {
  /**
   * @brief: Constructor for ControlLoopVariables
   * @param ipGains: Pointer to Gains object
  */
  ControlLoopVariables(Gains * const ipGains)
    : motorGains(ipGains) {
    ;
  }
  /**
   * @brief: Resets the controller variables
  */
  void resetController() {
    motor_setpoint = 0;
    motor_input = 0;
    motor_output = 0;
    motor_error = 0;
    motor_prev_error = 0;
    motor_integral = 0;
    motor_derivative = 0;
  }
  Gains* const motorGains;
  float motor_setpoint = 0;  // Setpoint and input have to be in ticks per control_freq_time_period
  float motor_input = 0;     // 
  float motor_output = 0;
  float motor_error = 0;
  float motor_prev_error = 0;
  float motor_integral = 0;
  float motor_derivative = 0;
}ControlLoopVariables;


/**
 * @brief: Custom message for publishing current tick rates
*/
static CurrentTickRate currentTickRate;
/**
 * @brief: Custom message for receiving gains
*/
static Gains gains;
/**
 * @brief: Custom message for receiving targets
*/
static Target targ;
/**
 * @brief: Global object for storing target velocities once parsng is done
*/
static Target globalTarget;
/**
 * @brief: Stores value of 2*pi
*/
static const constexpr float rotationToRadians = 6.28318;  // 1 rotation is 2Pi radians
/**
 * @brief: stores control loop variables for left motor
*/
static ControlLoopVariables leftMotor(&motorGains.leftMotorGains);
/**
 * @brief: stores control loop variables for right motor
*/
static ControlLoopVariables rightMotor(&motorGains.rightMotorGains);
/**
 * @brief: Global Flag to start control
*/
volatile bool startControl = false;

template<typename T>
/**
 * @brief: Utility function to find sgn() of a number
 * @param val: Number to find sgn() of
*/
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

/**
 * @brief: Utility function to attach encoder interrupts
*/
void attachEncInt(){
  attachInterrupt(digitalPinToInterrupt(encoderPins.leftEnc), leftMotorISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPins.rightEnc), rightMotorISR, RISING);
}

/**
 * @brief: Utility function to detach encoder interrupts
*/
void detachEncInt(){
  detachInterrupt(digitalPinToInterrupt(encoderPins.leftEnc));
  detachInterrupt(digitalPinToInterrupt(encoderPins.rightEnc));
}

/**
 * @brief: Callback for setting target velocities
 * @details: Sets the target velocities for the left and right motors , and also sets the startControl flag if msg.theta > 0.5
 * @param msg: Target message
*/
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

/**
 * @brief: Callback for setting gains
*/
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

/**
 * @brief: Processes a parsed message and calls the appropriate callback
*/
void process(uint8_t* buffer){
  if(buffer[1]==tags[Tags::TARGET]){
    targ.deserialize(buffer);
    targetVelCallback(targ);
  }else if(buffer[1]==tags[Tags::GAINS]){
    gains.deserialize(buffer);
    setGainsCallback(gains);
  }
}

/**
* @brief: Defines states for the state machine of the UART parser
*/
enum SpinStates{
  First,
  TopicName,
  Data,
  End  
};

/**
 * @brief: Defines the current state of the UART parser
*/
static SpinStates spinStates=SpinStates::First;

/**
 *  @brief: Input buffer for the serial data
*/
uint8_t in_buf[50];

/**
 *  @brief: Reads a byte from the serial buffer , parses the byte string , manages the parser's state machine
 * @details: Manages the serial buffers and parses it, for messages defined under UARTParser.h. Uses a read timeout, to ensure 1 message doesnt take too long to process
 * @param: None
 * @return: None
 */
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

/**
 *  @brief: Setup function, called once at the start of the program
 * @details: Sets up serial output, sets up encoder pins, sets up motor pins, sets up timer interrupts
 * @param: None
 * @return: None
 */
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

}

#if ENABLE_CONTROL
  volatile float elapsed_time = 0;
  float integratedTargetDistance = 0;
#endif

/**
 * @brief: IIR filter constant
*/
static const constexpr float alpha = 0.7;
/**
 *  @brief: First order IIR filter for velocity 
 * @note: alpha = filter constant = 0.7
 * @param raw Raw velocity
 * @param filter Filtered velocity
 * @return: Filtered velocity
*/
static inline float filterVelocity(const float raw, float& filter) {
  return filter = filter + alpha * ((raw - filter));
}


/**
 *  @brief: Acts as the main execution thread, controls the execution of other subthreads
 * @details: Calls Control subthread, calculates velocities, calls UART spin subthread, calls debug print subthread
 * @param: None
 * @return: None
 */
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
      auto leftEncTicks = encoderData.leftEncoderTicks;
      auto rightEncTicks = encoderData.rightEncoderTicks; 
           
      currentTickRate.leftTickRate = float(leftEncTicks - encoderData.leftEncoderPrev);
      currentTickRate.rightTickRate = float(rightEncTicks - encoderData.rightEncoderPrev);

      encoderData.leftEncoderPrev = leftEncTicks;
      encoderData.rightEncoderPrev = rightEncTicks;
  
      publishCustomMsg( currentTickRate );
    }
  #endif

  /**
  UART spin subthread
  Spins twice as fast as control loop, to ensure that we don't miss any messages
  **/
  if ( (globalTimer.getGlobalTimeinTicks() - globalTimer.uartSpinPrevTime) >= (intervalsTicks.uartSpinRate)/2) {
    globalTimer.uartSpinPrevTime = globalTimer.getGlobalTimeinTicks();
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

/**
 * @brief: This utility function converts rotations and interval to angular velocity in radians per second
 * @note: 1 sec = 1000ms, Encoder is on Motor Shaft
 * @param rotations No of rotations
 * @param intervalMs Interval in milliseconds
 * @return: No of rotations
 */
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

/**
 * @brief: This utility function converts encoder ticks to No of rotations
 * @param ticks Encoder ticks
 * @return: No of rotations
 */
static inline float getRotationsFromTicks(const int8_t& ticks) {
  return (float(ticks) / sysCons.ticksPerRevOutput);
}


/**
 *  @brief: Writes PWM values to the motor pins
 * @details: Writes PWM values to the motor pins, and also checks for saturation
 * @param pwmL PWM value for left motor (0-255)
 * @param pwmR PWM value for right motor (0-255)
 * @return: None
 */
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

/**
 *  @brief: Sets the direction of the motors
 * @details: Sets the direction of the motors, interally, the direction is converted to actual necessary voltage sign
 * @param leftDirection true for forward, false for backward
 * @param rightDirection true for forward, false for backward
 * @return: None
 */
void setDirection(bool leftDirection, bool rightDirection) {
  bool leftMotor = false;
  bool rightMotor = false;
  (leftDirection == true) ? leftMotor = motorControl.leftMotorForwardState : leftMotor = !motorControl.leftMotorForwardState;
  (rightDirection == true) ? rightMotor = motorControl.rightMotorForwardState : rightMotor = !motorControl.rightMotorForwardState;
  digitalWrite(motorControl.leftMotorDirectionPin, leftMotor);    // Left wheel, LOW for forward
  digitalWrite(motorControl.rightMotorDirectionPin, rightMotor);  // Right wheel, HIGH for forward
}


#if ENABLE_CONTROL

/** 
 * @brief: Takes control action on specified ControlLoopVariables object
  * @details: Calculates error, integral and derivative terms, applies PID weights, implements anti-windup and generates a control output.
  * @param motor ControlLoopVariables& object on which control action is to be taken
  * @return: None
  */
static void controlAction(ControlLoopVariables& motor) {
  //Calculate Error
  motor.motor_error = motor.motor_setpoint - motor.motor_input;
  //calculate Intgral and Derivative terms for each motor
  motor.motor_integral += motor.motor_error * elapsed_time;
  motor.motor_integral = constrain(motor.motor_integral, -1 * motor.motorGains->iSat, motor.motorGains->iSat);
  motor.motor_derivative = (motor.motor_error - motor.motor_prev_error) / elapsed_time;
  motor.motor_prev_error = motor.motor_error;
  //Calculate Outut
  motor.motor_output = motor.motorGains->kp * motor.motor_error + motor.motorGains->ki * motor.motor_integral + motor.motorGains->kd * motor.motor_derivative;
}



float leftFilt = 0;
float rightFilt = 0;
/**
 *  @brief: Sets controller target and input, runs control loop and applies voltage to motors
 *  @details: Sets controller target and input, filters input velocity, runs control loop per motor and applies voltage to motors
 *  @param time_delta time elapsed in timer ticks since last control loop call
 *  @return: None
*/
void controlLoop(unsigned long& time_delta) {

  if (startControl) {

    // Calculate elapsed time in seconds
    elapsed_time = GlobalTime<unsigned long>::convertTicksToTimeMs(time_delta) / 1000.0;

    // Set Input as filtered Left and Right Motor velocity
    leftMotor.motor_input = filterVelocity(currentTickRate.leftTickRate,leftFilt);
    rightMotor.motor_input = filterVelocity(currentTickRate.rightTickRate,rightFilt);

    // Target is received from the UART channel
    leftMotor.motor_setpoint = globalTarget.leftMotorTarget;
    rightMotor.motor_setpoint = globalTarget.rightMotorTarget;

    // Run control loop and calculate output
    controlAction(leftMotor);
    controlAction(rightMotor);

    // Figure out direction of voltage and magnitude of voltage, and then apply it
    auto leftSign = sgn(leftMotor.motor_output);
    auto rightSign = sgn(rightMotor.motor_output);

    // Set Voltage direction
    setDirection((leftSign == 1) ? true : false, (rightSign == 1) ? true : false);
    // Find absolute of the voltage
    auto absLeft = (leftMotor.motor_output * float(leftSign));
    auto absRight = (rightMotor.motor_output * float(rightSign));
    // Send voltage to motor
    pwmWrite(uint8_t( absLeft ) , uint8_t(float(absRight)));
  }
}
#endif

/**
 *  @brief: Interrupt Service Routine for 16 bit Timer 1
 * @details: This ISR is called when the timer overflows, and the overflow count is incremented
 * @param: None
 * @return: None
 */
ISR(TIMER1_OVF_vect) {
  globalTimer.incrementOvf();
}

/**
 *  @brief: Interrupt Service Routine for the left motor encoder
 * @details: This ISR is called when a tick on the left motor encoder is detected, and according to the direction of the motor, the encoder ticks are incremented or decremented
 * @param: None
 * @return: None
 */
void leftMotorISR() {
  encoderData.leftEncoderDirection = digitalRead(encoderPins.leftEncDirection);
  (encoderData.leftEncoderDirection == 1) ? encoderData.leftEncoderTicks -= 1 : encoderData.leftEncoderTicks += 1;
}

/**
 *  @brief: Interrupt Service Routine for the right motor encoder
 * @details: This ISR is called when a tick on the right motor encoder is detected, and according to the direction of the motor, the encoder ticks are incremented or decremented
 * @param: None
 * @return: None
 */
void rightMotorISR() {
  encoderData.rightEncoderDirection = digitalRead(encoderPins.rightEncDirection);
  (encoderData.rightEncoderDirection == 1) ? encoderData.rightEncoderTicks += 1 : encoderData.rightEncoderTicks -= 1;
}
