//  AP_MotorsTailsitter.cpp - ArduCopter motors library for tailsitters and bicopters

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsTailsitter.h"
#include <GCS_MAVLink/GCS.h>
#include "AP_Motors_Class.h"

extern const AP_HAL::HAL& hal;
#define SERVO_OUTPUT_RANGE  4500

// init
void AP_MotorsTailsitter::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // ����Ĭ�ϵĵ�����ŷ�ӳ�� setup default motor and servo mappings
    uint8_t chan;

    // k_motor1 throttleLeft  ����   73 1
    // k_motor2 throttleRight �ҵ��   74 3
    // k_motor3 throttleTailL β���� 133 2
    // k_motor4 throttleTailR β���� 134 4
    // k_motor5 tiltLeft      ����ת   135
    // k_motor6 tiltRight     ����ת   136
    // k_motor7 tiltTail      β��ת   137

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleLeft, CH_1);
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleLeft, chan)) {
        motor_enabled[chan] = true;    }

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleRight, CH_3);
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleRight, chan)) {
        motor_enabled[chan] = true;    }

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleTailL, CH_2);
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleTailL, chan)) {
        motor_enabled[chan] = true;    }

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleTailR, CH_4);
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleTailR, chan)) {
        motor_enabled[chan] = true;    }

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltLeft, CH_5);
    SRV_Channels::set_angle(SRV_Channel::k_tiltLeft, SERVO_OUTPUT_RANGE);

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltRight, CH_6);
    SRV_Channels::set_angle(SRV_Channel::k_tiltRight, SERVO_OUTPUT_RANGE);

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltTail, CH_7);
    SRV_Channels::set_angle(SRV_Channel::k_tiltTail, SERVO_OUTPUT_RANGE);

    // record successful initialisation if what we setup was the desired frame_class
    _flags.initialised_ok = (frame_class == MOTOR_FRAME_TAILSITTER);
}

// �����̣߳���һ��Ƶ�ʵ��� Constructor
AP_MotorsTailsitter::AP_MotorsTailsitter(uint16_t loop_rate, uint16_t speed_hz) :
    AP_MotorsMulticopter(loop_rate, speed_hz)
    {    set_update_rate(speed_hz);    }

// �趨����ĸ���Ƶ�ʣ�������� set update rate to motors - a value in hertz
void AP_MotorsTailsitter::set_update_rate(uint16_t speed_hz)
{
    _speed_hz = speed_hz;  // record requested speed
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleLeft, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleRight, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleTailL, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleTailR, speed_hz);
}

// ��ͬ״̬�Ե���Ͷ����ָ�ͬ
void AP_MotorsTailsitter::output_to_motors()
{
    if (!_flags.initialised_ok) {
        return;    }

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN: // �ػ�
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, get_pwm_output_min());
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, get_pwm_output_min());
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleTailL, get_pwm_output_min());
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleTailR, get_pwm_output_min());
            break;
        case SpoolState::GROUND_IDLE: // ��������
            set_actuator_with_slew(_actuator[1], actuator_spin_up_to_ground_idle());
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, output_to_pwm(actuator_spin_up_to_ground_idle()));
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, output_to_pwm(actuator_spin_up_to_ground_idle()));
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleTailL, output_to_pwm(actuator_spin_up_to_ground_idle()));
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleTailR, output_to_pwm(actuator_spin_up_to_ground_idle()));
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, output_to_pwm(thrust_to_actuator(_thrust_left)));
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, output_to_pwm(thrust_to_actuator(_thrust_right)));
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleTailL, output_to_pwm(thrust_to_actuator(_thrust_taill)));
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleTailR, output_to_pwm(thrust_to_actuator(_thrust_tailr)));
            break;
    }

    // ʼ���������ת��� Always output to tilt servos
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltLeft, _tilt_left*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltRight, _tilt_right*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltTail, _tilt_tail*SERVO_OUTPUT_RANGE);
}

//  get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  ȷ�������������ͻ this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsTailsitter::get_motor_mask()
{
    uint32_t motor_mask = 0;
    uint8_t chan;
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleLeft, chan)) {
        motor_mask |= 1U << chan;    }
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleRight, chan)) {
        motor_mask |= 1U << chan;    }
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleTailL, chan)) {
        motor_mask |= 1U << chan;    }
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleTailR, chan)) {
        motor_mask |= 1U << chan;    }
    motor_mask |= AP_MotorsMulticopter::get_motor_mask(); // add parent's mask

    return motor_mask;
}

// �ؼ����裺��������� calculate outputs to the motors
void AP_MotorsTailsitter::output_armed_stabilizing()
{
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   thrust_max;                 // highest motor value
    float   thr_adj = 0.0f;             // the difference between the pilot's desired throttle and throttle_thrust_best_rpy
    float   rotate_angle;

    // Ӧ�õ�ѹ����ѹ���� apply voltage and air pressure compensation
    const float compensation_gain = get_compensation_gain();
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain;
    throttle_thrust = get_throttle() * compensation_gain;
    rotate_angle= _rotate_radio_passthrough;

    // ����������Ÿ����㣬���ڵ�ǰ�������� sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;    }

    // �����ǹؼ��Ŀ��Ʒ��䷽�� calculate left and right throttle outputs
    _thrust_left  = throttle_thrust + roll_thrust * 0.5f + pitch_thrust * 0.5f;
    _thrust_right = throttle_thrust - roll_thrust * 0.5f + pitch_thrust * 0.5f;
    _thrust_taill = throttle_thrust - pitch_thrust * 0.5f;
    _thrust_tailr = throttle_thrust - pitch_thrust * 0.5f;
    _tilt_left    = yaw_thrust - rotate_angle* 0.5f;
    _tilt_right   = yaw_thrust + rotate_angle* 0.5f;
    _tilt_tail    = rotate_angle* 0.5f;
    //_tilt_left  = pitch_thrust* 0.5f - yaw_thrust;
    //_tilt_right = pitch_thrust* 0.5f + yaw_thrust;
    //_tilt_tail  = pitch_thrust* 0.5f;

    // ��������������1���򽵵�ƽ������ if max thrust is more than one, reduce average throttle
    thrust_max = MAX(_thrust_right,_thrust_left);
    if (thrust_max > 1.0f) {
        thr_adj = 1.0f - thrust_max;
        limit.throttle_upper = true;
        limit.roll = true;
        limit.pitch = true;    }

    // ���ӵ����Լ���ƽ������ Add adjustment to reduce average throttle
    _thrust_left  = constrain_float(_thrust_left  + thr_adj, 0.0f, 1.0f);
    _thrust_right = constrain_float(_thrust_right + thr_adj, 0.0f, 1.0f);
    _thrust_taill  = constrain_float(_thrust_taill  + thr_adj, 0.0f, 1.0f);
    _thrust_tailr = constrain_float(_thrust_tailr + thr_adj, 0.0f, 1.0f);
    _throttle = throttle_thrust + thr_adj;
    _throttle_out = _throttle / compensation_gain; // ����������Զ����Ϊ�� compensation_gain can never be zero
}

//  output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsTailsitter::output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;    }

    // output to motors and servos
    switch (motor_seq) {
        case 1:
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, pwm);
            break;
        case 2:
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, pwm);
            break;
        case 3:
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleTailL, pwm);
            break;
        case 4:
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleTailR, pwm);
            break;
        case 5:
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltLeft, pwm);
            break;
        case 6:
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltRight, pwm);
            break;
        case 7:
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltTail, pwm);
            break;
        default:
            break;
    }
}
