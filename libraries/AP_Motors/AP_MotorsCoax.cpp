// ���������������������Ϊ9������4�����
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsCoax.h"
#include <GCS_MAVLink/GCS.h>
extern const AP_HAL::HAL& hal;

void AP_MotorsCoax::init(motor_frame_class frame_class, motor_frame_type frame_type) // ��ʼ��
{   for (uint8_t i = 0; i < 4; i++) {  add_motor_num(CH_1 + i);  }   // ȷ�����4�����ͨ��
    motor_enabled[AP_MOTORS_MOT_3] = true;                           // ���õ��ʹ�ܱ�־��ʹ����Խ��е��У׼
    motor_enabled[AP_MOTORS_MOT_4] = true;
    for (uint8_t i = 0; i < NUM_ACTUATORS; i++)                      // ����ִ�������� setup actuator scaling
    { SRV_Channels::set_angle(SRV_Channels::get_motor_function(i), AP_MOTORS_COAX_SERVO_INPUT_RANGE); }
    _flags.initialised_ok = (frame_class == MOTOR_FRAME_COAX);       // ������ǰ�װ����Ԥ�ڻ������ͣ����¼��ʼ���ɹ�
}

void AP_MotorsCoax::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{   _flags.initialised_ok = (frame_class == MOTOR_FRAME_COAX);   }   // ���û�������

void AP_MotorsCoax::set_update_rate(uint16_t speed_hz)               // ���õ����ˢ��Ƶ�ʣ���λΪ���ȵ�ֵ��
{   _speed_hz = speed_hz;                                            // ��¼�����ٶ� record requested speed
    uint32_t mask =  1U << AP_MOTORS_MOT_3 | 1U << AP_MOTORS_MOT_4 ;
    rc_set_freq(mask, _speed_hz);
}

void AP_MotorsCoax::output_to_motors()
{   switch (_spool_state) {
        case SpoolState::SHUT_DOWN:                                  // �ػ���������Сֵ
            rc_write_angle(AP_MOTORS_MOT_1, _roll_radio_passthrough * AP_MOTORS_COAX_SERVO_INPUT_RANGE);
            rc_write_angle(AP_MOTORS_MOT_2, _pitch_radio_passthrough * AP_MOTORS_COAX_SERVO_INPUT_RANGE);
            rc_write(AP_MOTORS_MOT_3, output_to_pwm(0));
            rc_write(AP_MOTORS_MOT_4, output_to_pwm(0));             break;
        case SpoolState::GROUND_IDLE:                                // �������ã�������û����ʱ�������
            for (uint8_t i = 0; i < NUM_ACTUATORS; i++)
            { rc_write_angle(AP_MOTORS_MOT_1 + i, _spin_up_ratio * _actuator_out[i] * AP_MOTORS_COAX_SERVO_INPUT_RANGE); }
            set_actuator_with_slew(_actuator[3], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[4], actuator_spin_up_to_ground_idle());
            rc_write(AP_MOTORS_MOT_3, output_to_pwm(_actuator[3]));
            rc_write(AP_MOTORS_MOT_4, output_to_pwm(_actuator[4]));  break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:                         // ���Ų�����
        case SpoolState::SPOOLING_DOWN:                              // ��������Ҫ�������
            for (uint8_t i = 0; i < NUM_ACTUATORS; i++)
            { rc_write_angle(AP_MOTORS_MOT_1 + i, _actuator_out[i] * AP_MOTORS_COAX_SERVO_INPUT_RANGE); }
            set_actuator_with_slew(_actuator[3], thrust_to_actuator(_thrust_yt_ccw));
            set_actuator_with_slew(_actuator[4], thrust_to_actuator(_thrust_yt_cw));
            rc_write(AP_MOTORS_MOT_3, output_to_pwm(_actuator[3]));
            rc_write(AP_MOTORS_MOT_4, output_to_pwm(_actuator[4]));  break; }
}

// ��ȡִ�������룺����һ��λ���룬���1��ʾִ��������ʹ�ã�����ȷ������PWM�������ͻ
uint16_t AP_MotorsCoax::get_motor_mask()
{   uint32_t motor_mask =  //  1U�����޷������͵�ֵΪ1�����Ա�ʾΪ00000000��1(���һλΪ1��31��0��1��1)
        1U << AP_MOTORS_MOT_1 | 1U << AP_MOTORS_MOT_2 | 1U << AP_MOTORS_MOT_3 | 1U << AP_MOTORS_MOT_4;
    uint16_t mask = rc_map_mask(motor_mask);
    mask |= AP_MotorsMulticopter::get_motor_mask();                 // ��Ӹ������� add parent's mask
    return mask;
}

void AP_MotorsCoax::output_armed_stabilizing()                      // ��ִ��������ָ��
{   float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   throttle_avg_max;           // throttle thrust average maximum value, 0.0 - 1.0
    float   thrust_min_rpy;             // the minimum throttle setting that will not limit the roll and pitch output
    float   thr_adj;                    // the difference between the pilot's desired throttle and throttle_thrust_best_rpy
    float   thrust_out;                 //
    float   rp_scale = 1.0f;            // this is used to scale the roll, pitch and yaw to fit within the motor limits
    float   actuator_allowed = 0.0f;    // amount of yaw we can fit in

    // Ӧ�õ�ѹ����ѹ���� apply voltage and air pressure compensation
    const float compensation_gain = get_compensation_gain();
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain;
    throttle_thrust = get_throttle() * compensation_gain;
    throttle_avg_max = _throttle_avg_max * compensation_gain;

    // ��ȷ������Ÿ����㣬�ҵ��ڵ������Ƶ����� sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;    }
    throttle_avg_max = constrain_float(throttle_avg_max, throttle_thrust, _throttle_thrust_max);
    float rp_thrust_max = MAX(fabsf(roll_thrust), fabsf(pitch_thrust));

    // ������Ҫ���Ŷ��ٹ�ת�͸�����Ϊ��Сƫ�������㹻�ķ�Χ
    if (is_zero(rp_thrust_max)) {  rp_scale = 1.0f;  }
    else {  rp_scale = constrain_float((1.0f - MIN(fabsf(yaw_thrust), 0.5f * (float)_yaw_headroom / 1000.0f)) / rp_thrust_max, 0.0f, 1.0f);
        if (rp_scale < 1.0f)    {
            limit.roll = true;
            limit.pitch = true; } }

    actuator_allowed = 2.0f * (1.0f - rp_scale * rp_thrust_max);
    if (fabsf(yaw_thrust) > actuator_allowed) {
        yaw_thrust = constrain_float(yaw_thrust, -actuator_allowed, actuator_allowed);
        limit.yaw = true;    }

    // ���㲻���ƹ�ת��������ƫ��������С���� calculate the minimum thrust that doesn't limit the roll, pitch and yaw forces
    thrust_min_rpy = MAX(fabsf(rp_scale * rp_thrust_max), fabsf(yaw_thrust));

    thr_adj = throttle_thrust - throttle_avg_max;       // ���Ų���̫�ͣ�����������ؽ���������������
    if (thr_adj < (thrust_min_rpy - throttle_avg_max))                       {
        thr_adj = MIN(thrust_min_rpy, throttle_avg_max) - throttle_avg_max;  }

    thrust_out = throttle_avg_max + thr_adj;            // �������Ŵ�С
    _throttle_out = thrust_out / compensation_gain;
    if (fabsf(yaw_thrust) > thrust_out) {
        yaw_thrust = constrain_float(yaw_thrust, -thrust_out, thrust_out);
        limit.yaw = true;    }
    _thrust_yt_ccw = thrust_out + 0.5f * yaw_thrust;
    _thrust_yt_cw  = thrust_out - 0.5f * yaw_thrust;
    // ����ִ�л�����õļ������� limit thrust out for calculation of actuator gains
    float thrust_out_actuator = constrain_float(MAX(_throttle_hover * 0.5f, thrust_out), 0.5f, 1.0f);
    if (is_zero(thrust_out)) {
        limit.roll = true;
        limit.pitch = true;  }

    _actuator_out[0] = roll_thrust / thrust_out_actuator; // ִ����ƫ����������س����ȣ������ųɷ��ȣ�������Խ��ƫ��ԽС
    _actuator_out[1] = pitch_thrust / thrust_out_actuator;
    if (fabsf(_actuator_out[0]) > 1.0f) {
        limit.roll = true;
        _actuator_out[0] = constrain_float(_actuator_out[0], -1.0f, 1.0f);    }
    if (fabsf(_actuator_out[1]) > 1.0f) {
        limit.pitch = true;
        _actuator_out[1] = constrain_float(_actuator_out[1], -1.0f, 1.0f);    }
}

// ������Ͷ�����ʵ�ʵ�pwmֵ��motor_seq�ǵ������ţ���1��ʼ����
void AP_MotorsCoax::output_test_seq(uint8_t motor_seq, int16_t pwm)
{   if (!armed()) {   return;  }
    switch (motor_seq)    {
        case 1: rc_write(AP_MOTORS_MOT_1, pwm);  break; // ����1
        case 2: rc_write(AP_MOTORS_MOT_2, pwm);  break; // ����2
        case 3: rc_write(AP_MOTORS_MOT_3, pwm);  break; // ���1
        case 4: rc_write(AP_MOTORS_MOT_4, pwm);  break; // ���2
        default:  break;  }
}
