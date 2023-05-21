#ifndef PARSE_UART_H
#define PARSE_UART_H
#include <HardwareSerial.h>



/**
 * @brief:  Abstract class for custom messages
 * @details:  All custom messages must inherit from this class, 3 custom messages are supported. Gains, Target and CurrentTickRate
 * @note: Gains ={'G',k_p,k_i,k_d,i_sat,isLeft}\n
 * @note: Target = {'T',x,y,theta}\n
 * @note: CurrentTickRate = {'C',x,y}\n
*/
class CustomMsg
{
public:
  /**
   * @brief:  Serializes Member variables to outbuffer
   * @param  outbuffer:  Valid pointer to buffer where data has to be written
   * @note the serialization logic has been taken from ros_serial library
  */
  virtual uint8_t serialize(unsigned char *const outbuffer) const = 0;
  /**
   * @brief:  Deserializes custom message from inpt data buffer and updates the member variables
   * @param  inbuffer:  Valid pointer to a buffer
   * @note the deserialization logic has been taken from ros_serial library
  */
  virtual uint8_t deserialize(const unsigned char  *const inbuffer) = 0;
};

/**
 * @brief:  Enum for tags
*/
enum Tags{TARGET,GAINS,CURRENT};

/**
 * @brief:  Tags for each message
*/
static const unsigned char tags[] = {'T','G','C'};

/**
 * @brief:  Serial port object , initialized to Serial port 0, the default port on Arduino Uno, used for Serial.begin() etc
*/
HardwareSerial* hw= &Serial;

/**
 * @brief: Custom message to receive control gains from RPi
*/
class Gains:public CustomMsg{
  public:
  /// \name Gains
  /// @brief:Defines the gains for the PID controller,and the integral saturation

  ///@{
  /// @brief:  Proportional gain
  float kp;
  /// @brief:  Derivative gain
  float kd;
  /// @brief:  Integral gain
  float ki;
  /// @brief:  Integral Saturation
  float iSat;
  ///@}

  /// @brief:  is the message meant for left motor or right motor
  bool isLeft;

  virtual uint8_t serialize(unsigned char *const outbuffer) const override
  {
      uint8_t offset = 0;

      *(outbuffer+offset) = '{';
      offset+=1;

      *(outbuffer+offset) = tags[Tags::GAINS];
      offset+=1;

      union {
        float real;
        uint32_t base;
      } u_kp;
      u_kp.real = this->kp;
      *(outbuffer + offset + 0) = (u_kp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kp);
      union {
        float real;
        uint32_t base;
      } u_ki;
      u_ki.real = this->ki;
      *(outbuffer + offset + 0) = (u_ki.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ki.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ki.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ki.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ki);
      union {
        float real;
        uint32_t base;
      } u_kd;
      u_kd.real = this->kd;
      *(outbuffer + offset + 0) = (u_kd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kd);
      union {
        float real;
        uint32_t base;
      } u_i_clamp;
      u_i_clamp.real = this->iSat;
      *(outbuffer + offset + 0) = (u_i_clamp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_i_clamp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_i_clamp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_i_clamp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->iSat);
      union {
        bool real;
        uint8_t base;
      } u_isleft;
      u_isleft.real = this->isLeft;
      *(outbuffer + offset + 0) = (u_isleft.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isLeft);
      
      *(outbuffer+offset) = '}';
      offset+=1;

      *(outbuffer+offset) = '\n';
      offset+=1;

      return offset;
  }
  virtual uint8_t deserialize(const unsigned char  *const inbuffer) override
  {
    uint8_t offset = 0;
    offset+=2;
    union {
      float real;
      uint32_t base;
    } u_kp;
    u_kp.base = 0;
    u_kp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
    u_kp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    u_kp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    u_kp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    this->kp = u_kp.real;
    offset += sizeof(this->kp);
    union {
      float real;
      uint32_t base;
    } u_ki;
    u_ki.base = 0;
    u_ki.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
    u_ki.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    u_ki.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    u_ki.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    this->ki = u_ki.real;
    offset += sizeof(this->ki);
    union {
      float real;
      uint32_t base;
    } u_kd;
    u_kd.base = 0;
    u_kd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
    u_kd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    u_kd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    u_kd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    this->kd = u_kd.real;
    offset += sizeof(this->kd);
    union {
      float real;
      uint32_t base;
    } u_i_clamp;
    u_i_clamp.base = 0;
    u_i_clamp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
    u_i_clamp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    u_i_clamp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    u_i_clamp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    this->iSat = u_i_clamp.real;
    offset += sizeof(this->iSat);
    union {
      bool real;
      uint8_t base;
    } u_isleft;
    u_isleft.base = 0;
    u_isleft.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
    this->isLeft = u_isleft.real;
    offset += sizeof(this->isLeft);

    offset+=2;
    return offset;
  }
};

/**
 * @brief: Custom message to receive Left and right motor target speeds from RPi
*/
class Target :public CustomMsg{
  public:
  /**
   * @brief:  Target left motor speed
  */
  float leftMotorTarget;
  /**
   * @brief:  Target right motor speed
  */
  float rightMotorTarget;
  /**
   * @brief:  Theta is used to control start and stop of control action. When theta is 0, control action is stopped, when theta is 1, control action is started
  */
  float theta;
  Target()
  :leftMotorTarget(0),
  rightMotorTarget(0),
  theta(0)
  {
    ;
  }
  virtual uint8_t serialize(unsigned char *const outbuffer) const override{
    uint8_t offset = 0;

    *(outbuffer+offset) = '{';
    offset+=1;

    *(outbuffer+offset) = tags[Tags::TARGET];
    offset+=1;

    union {
      float real;
      uint32_t base;
    } u_i_leftMotorTarget;
    u_i_leftMotorTarget.real = this->leftMotorTarget;
    *(outbuffer + offset + 0) = (u_i_leftMotorTarget.base >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (u_i_leftMotorTarget.base >> (8 * 1)) & 0xFF;
    *(outbuffer + offset + 2) = (u_i_leftMotorTarget.base >> (8 * 2)) & 0xFF;
    *(outbuffer + offset + 3) = (u_i_leftMotorTarget.base >> (8 * 3)) & 0xFF;
    offset += sizeof(this->leftMotorTarget);

    union {
      float real;
      uint32_t base;
    } u_i_rightMotorTarget;
    u_i_rightMotorTarget.real = this->rightMotorTarget;
    *(outbuffer + offset + 0) = (u_i_rightMotorTarget.base >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (u_i_rightMotorTarget.base >> (8 * 1)) & 0xFF;
    *(outbuffer + offset + 2) = (u_i_rightMotorTarget.base >> (8 * 2)) & 0xFF;
    *(outbuffer + offset + 3) = (u_i_rightMotorTarget.base >> (8 * 3)) & 0xFF;
    offset += sizeof(this->rightMotorTarget);

    union {
      float real;
      uint32_t base;
    } u_i_theta;
    u_i_theta.real = this->theta;
    *(outbuffer + offset + 0) = (u_i_theta.base >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (u_i_theta.base >> (8 * 1)) & 0xFF;
    *(outbuffer + offset + 2) = (u_i_theta.base >> (8 * 2)) & 0xFF;
    *(outbuffer + offset + 3) = (u_i_theta.base >> (8 * 3)) & 0xFF;
    offset += sizeof(this->theta);

    *(outbuffer+offset) = '}';
    offset+=1;

    *(outbuffer+offset) = '\n';
    offset+=1;

    return offset;
  }
  virtual uint8_t deserialize(const unsigned char  *const inbuffer) override{
    uint8_t offset = 0;
    offset+=2;
    union {
      float real;
      uint32_t base;
    } u_leftTarg;
    u_leftTarg.base = 0;
    u_leftTarg.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
    u_leftTarg.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    u_leftTarg.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    u_leftTarg.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    this->leftMotorTarget = u_leftTarg.real;
    offset += sizeof(this->leftMotorTarget);

    union {
      float real;
      uint32_t base;
    } u_rightTarg;
    u_rightTarg.base = 0;
    u_rightTarg.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
    u_rightTarg.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    u_rightTarg.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    u_rightTarg.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    this->rightMotorTarget = u_rightTarg.real;
    offset += sizeof(this->rightMotorTarget);

    union {
      float real;
      uint32_t base;
    } u_theta;
    u_theta.base = 0;
    u_theta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
    u_theta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    u_theta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    u_theta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    this->theta = u_theta.real;
    offset += sizeof(this->theta);

    offset+=2;
    return offset;
  }
};

/**
 * @brief: Custom message to send current tick rate of left and right motors to RPi
*/
class CurrentTickRate:public CustomMsg{
  public:
  float leftTickRate=0;
  float rightTickRate=0;
  virtual uint8_t serialize(unsigned char *const outbuffer) const override{
    uint8_t offset = 0;

    *(outbuffer+offset) = '{';
    offset+=1;

    *(outbuffer+offset) = tags[Tags::CURRENT];
    offset+=1;

    union {
      float real;
      uint32_t base;
    } u_i_leftMotorTicks;
    u_i_leftMotorTicks.real = this->leftTickRate;

    *(outbuffer + offset + 0) = (u_i_leftMotorTicks.base >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (u_i_leftMotorTicks.base >> (8 * 1)) & 0xFF;
    *(outbuffer + offset + 2) = (u_i_leftMotorTicks.base >> (8 * 2)) & 0xFF;
    *(outbuffer + offset + 3) = (u_i_leftMotorTicks.base >> (8 * 3)) & 0xFF;

    offset += sizeof(this->leftTickRate);

    union {
      float real;
      uint32_t base;
    } u_i_rightMotorTicks;
    u_i_rightMotorTicks.real = this->rightTickRate;

    *(outbuffer + offset + 0) = (u_i_rightMotorTicks.base >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (u_i_rightMotorTicks.base >> (8 * 1)) & 0xFF;
    *(outbuffer + offset + 2) = (u_i_rightMotorTicks.base >> (8 * 2)) & 0xFF;
    *(outbuffer + offset + 3) = (u_i_rightMotorTicks.base >> (8 * 3)) & 0xFF;
    
    offset += sizeof(this->rightTickRate);


    *(outbuffer+offset) = '}';
    offset+=1;

    *(outbuffer+offset) = '\n';
    offset+=1;

    return offset;
  }
  virtual uint8_t deserialize(const unsigned char  *const inbuffer) override{
    uint8_t offset = 0;
    offset+=2;

    union {
      float real;
      uint32_t base;
    } u_leftTarg;
    u_leftTarg.base = 0;
    u_leftTarg.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
    u_leftTarg.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    u_leftTarg.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    u_leftTarg.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    this->leftTickRate = u_leftTarg.real;
    offset += sizeof(this->leftTickRate);

    union {
      float real;
      uint32_t base;
    } u_rightTarg;
    u_rightTarg.base = 0;
    u_rightTarg.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
    u_rightTarg.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    u_rightTarg.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    u_rightTarg.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    this->rightTickRate = u_rightTarg.real;
    offset += sizeof(this->rightTickRate);

    offset+=2;
    return offset;
  }
};

/**
 * @brief:  Reads bytes from serial port's cyclic buffer
*/
inline int readUart(){return hw->read();}

/**
 * @brief: Output buffer for serial port
*/
uint8_t out_buf[50];

template<typename T>
/**
 * @brief:  Publishes custom message to serial port
 * @param  input Custom message object, must have a serialize method
*/
void publishCustomMsg(T& input){
  auto length = input.serialize(out_buf);
  hw->write(out_buf, length);
}
#endif