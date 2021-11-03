// 无极六自由度矢量变姿系列飞行器，机型选择7
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_MotorsTri.h"
#include "AP_Motors_Class.h"
extern const AP_HAL::HAL& hal;
#define SERVO_OUTPUT_RANGE  4500

// init
void AP_MotorsTri::init(motor_frame_class frame_class, motor_frame_type frame_type)
{   uint8_t chan;     // 设置默认的电机和伺服映射

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleLeft, CH_1);
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleLeft,  chan)) { motor_enabled[chan] = true; }
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleRight, CH_3);
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleRight, chan)) { motor_enabled[chan] = true; }
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleTail, CH_2);
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleTail,  chan)) { motor_enabled[chan] = true; }

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltLeft, CH_4);
    SRV_Channels::set_angle(SRV_Channel::k_tiltLeft, SERVO_OUTPUT_RANGE);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltRight, CH_5);
    SRV_Channels::set_angle(SRV_Channel::k_tiltRight, SERVO_OUTPUT_RANGE);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltTail, CH_6);
    SRV_Channels::set_angle(SRV_Channel::k_tiltTail, SERVO_OUTPUT_RANGE);

    _flags.initialised_ok = (frame_class == MOTOR_FRAME_TRI); // 记录初始化成功
}

// 申请线程，以一定频率调用 Constructor
AP_MotorsTri::AP_MotorsTri(uint16_t loop_rate, uint16_t speed_hz):
AP_MotorsMulticopter(loop_rate, speed_hz) { set_update_rate(speed_hz); }

// set update rate to motors - a value in hertz
void AP_MotorsTri::set_update_rate(uint16_t speed_hz)
{
    _speed_hz = speed_hz; // record requested speed
    // set update rate for the 3 motors (but not the servo on channel 7)
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleLeft , speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleRight, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleTail, speed_hz);
}

void AP_MotorsTri::output_to_motors()
{
    if (!_flags.initialised_ok) { return; }
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, get_pwm_output_min());
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, get_pwm_output_min());
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleTail, get_pwm_output_min());
            break;
        case SpoolState::GROUND_IDLE:
            // sends output to motors when armed but not flying
            set_actuator_with_slew(_actuator[1], actuator_spin_up_to_ground_idle());
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, output_to_pwm(actuator_spin_up_to_ground_idle()));
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, output_to_pwm(actuator_spin_up_to_ground_idle()));
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleTail, output_to_pwm(actuator_spin_up_to_ground_idle()));
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, output_to_pwm(thrust_to_actuator(_thrust_left)));
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, output_to_pwm(thrust_to_actuator(_thrust_right)));
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleTail, output_to_pwm(thrust_to_actuator(_thrust_tail)));
            break;    }
    // 始终输出给倾转舵机 Always output to tilt servos
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltLeft, _tilt_left*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltRight, _tilt_right*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltTail, _tilt_tail*SERVO_OUTPUT_RANGE);
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsTri::get_motor_mask()
{
    uint32_t motor_mask = 0;
    uint8_t chan;
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleLeft, chan)) {
        motor_mask |= 1U << chan;    }
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleRight, chan)) {
        motor_mask |= 1U << chan;    }
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleTail, chan)) {
        motor_mask |= 1U << chan;    }
    motor_mask |= AP_MotorsMulticopter::get_motor_mask(); // add parent's mask
    return motor_mask;
}

// output_armed - sends commands to the motors
void AP_MotorsTri::output_armed_stabilizing()
{
    float   roll_thrust;         // roll thrust input value, +/- 1.0
    float   pitch_thrust;        // pitch thrust input value, +/- 1.0
    float   yaw_thrust;          // yaw thrust input value, +/- 1.0
    float   throttle_thrust;     // throttle thrust input value, 0.0 - 1.0
    float   thrust_max;          // highest motor value
    float   thr_adj = 0.0f;      // 飞行员期望油门和throttle_thrust_best_rpy之间的差值
    float   rotate_angle;        // 旋转角输入值，最左为0，最右为1
    float   rate;                // 俯仰控制权重，水平为0，朝下为1，朝上为-1
    float   m_rate;              // 电机俯仰控制权重，水平为1，朝下为0，朝上为0
    float   s_rate;              // 舵机俯仰控制权重，水平为0，朝下为1，朝上为1

    // apply voltage and air pressure compensation
    const float compensation_gain = get_compensation_gain();
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain; 
    throttle_thrust = get_throttle() * compensation_gain;
   
    // 旋转角与俯仰控制权重
    rotate_angle = RC_Channels::get_radio_in(CH_6);          // 获取遥控器第6通道信号
    rotate_angle= (rotate_angle -1500) *0.002f;              // 转换为标准值
    rate = rotate_angle;
    if (rate >= 0.0f) { m_rate= 1.0f -rate; s_rate= rate; }  // 电机俯仰控制权重恒正
    if (rate <  0.0f) { m_rate= 1.0f +rate; s_rate=-rate; }

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) 
    { throttle_thrust = 0.0f; limit.throttle_lower = true; }
    if (throttle_thrust >= _throttle_thrust_max) 
    { throttle_thrust = _throttle_thrust_max; limit.throttle_upper = true; }

    // 电机控制分配 calculate left and right throttle outputs
    _thrust_left  =  throttle_thrust + m_rate *pitch_thrust *0.4f - roll_thrust *0.45f;
    _thrust_right =  throttle_thrust + m_rate *pitch_thrust *0.4f + roll_thrust *0.45f;
    _thrust_tail  =  throttle_thrust - m_rate *pitch_thrust *0.6f ;

    // 航向控制在大角度时适度减弱
     yaw_thrust = yaw_thrust - s_rate *yaw_thrust *0.4f;

    // 舵机控制分配，向上和向下的俯仰控制参与者不同
    if (rotate_angle <= 0.0f)  {
        _tilt_left    = -rotate_angle *0.7f - s_rate *pitch_thrust *0.2f - yaw_thrust *0.3f;
        _tilt_right   =  rotate_angle *0.7f + s_rate *pitch_thrust *0.2f - yaw_thrust *0.3f;
        _tilt_tail    =  rotate_angle *0.7f - s_rate *pitch_thrust *0.5f; }
    if (rotate_angle > 0.0f)   {
        _tilt_left    = -rotate_angle *0.7f + s_rate *pitch_thrust *0.3f - yaw_thrust *0.3f;
        _tilt_right   =  rotate_angle *0.7f - s_rate *pitch_thrust *0.3f - yaw_thrust *0.3f;
        _tilt_tail    =  rotate_angle *0.7f + s_rate *pitch_thrust *0.4f; }

   // 如果最大推力大于1，则降低平均油门
    thrust_max = MAX(_thrust_right,_thrust_left);
    if (thrust_max > 1.0f) { thr_adj = 1.0f - thrust_max;
    limit.throttle_upper = true; limit.roll = true; limit.pitch = true; }

    // 应用调整以降低平均油门
    _thrust_left  = constrain_float(_thrust_left  + thr_adj, 0.0f, 1.0f);
    _thrust_right = constrain_float(_thrust_right + thr_adj, 0.0f, 1.0f);
    _thrust_tail  = constrain_float(_thrust_tail  + thr_adj, 0.0f, 1.0f);
    _throttle = throttle_thrust + thr_adj;
    _throttle_out = _throttle / compensation_gain;
}
    // 执行器信号输出
void AP_MotorsTri::output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    if (!armed()) { return; } // exit immediately if not armed
    switch (motor_seq)      { // output to motors and servos
        case 1: SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, pwm);  break;
        case 2: SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, pwm); break;
        case 3: SRV_Channels::set_output_pwm(SRV_Channel::k_throttleTail, pwm);  break;
        case 4: SRV_Channels::set_output_pwm(SRV_Channel::k_tiltLeft, pwm);      break;
        case 5: SRV_Channels::set_output_pwm(SRV_Channel::k_tiltRight, pwm);     break;
        case 6: SRV_Channels::set_output_pwm(SRV_Channel::k_tiltTail, pwm);      break;
        default:     break; }
}