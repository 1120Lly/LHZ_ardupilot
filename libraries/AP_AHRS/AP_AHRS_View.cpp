// AHRS View class - ���ڴ�����������̬�ĵڶ��ӽ� for creating a 2nd view of the vehicle attitude

#include "AP_AHRS_View.h"
#include <stdio.h>

AP_AHRS_View::AP_AHRS_View(AP_AHRS &_ahrs, enum Rotation _rotation, float pitch_trim_deg) :
    rotation(_rotation),
    ahrs(_ahrs)
{
    switch (rotation) {
    case ROTATION_NONE:
        y_angle = 0;
        break;
    case ROTATION_PITCH_90:
        y_angle = 90;
        break;
    case ROTATION_PITCH_270:
        y_angle =  270;
        break;
    default:
        AP_HAL::panic("Unsupported AHRS view %u\n", (unsigned)rotation);
    }

    _pitch_trim_deg = pitch_trim_deg;
    // Add pitch trim
    rot_view.from_euler(0, radians(wrap_360(y_angle + pitch_trim_deg)), 0);
    rot_view_T = rot_view;
    rot_view_T.transpose();

    // setup initial state
    update();
}

// apply pitch trim
void AP_AHRS_View::set_pitch_trim(float trim_deg) {
    _pitch_trim_deg = trim_deg; 
    rot_view.from_euler(0, radians(wrap_360(y_angle + _pitch_trim_deg)), 0);
    rot_view_T = rot_view;
    rot_view_T.transpose();
};

// update state
void AP_AHRS_View::update(bool skip_ins_update)
{
    Matrix3f board_rotation;
    float board_rotate = RC_Channels::get_radio_in(CH_6);       // ��ȡң������6ͨ���ź�
    board_rotate= (board_rotate -1500) *0.2f;                   // ת��Ϊ�����б�Ƕ�
    board_rotation.from_euler(radians(0), radians(board_rotate), radians(0));

    rot_body_to_ned = ahrs.get_rotation_body_to_ned();
    gyro = ahrs.get_gyro();                                     // ��AP_AHRS���鶨���ʼֵΪ0
    if (!is_zero(y_angle + _pitch_trim_deg))
    {   rot_body_to_ned = rot_body_to_ned * rot_view_T;
        gyro = rot_view * gyro;   }

    gyro = gyro * board_rotation;                               // �¼���䣬���������ݽ����Զ��帩���Ƕ���ת

    rot_body_to_ned.to_euler(&roll, &pitch, &yaw);

    roll_sensor  = degrees(roll) * 100;
    pitch_sensor = degrees(pitch) * 100;
    yaw_sensor   = degrees(yaw) * 100;
    if (yaw_sensor < 0) {  yaw_sensor += 36000;  }

    ahrs.calc_trig(rot_body_to_ned,
                   trig.cos_roll, trig.cos_pitch, trig.cos_yaw,
                   trig.sin_roll, trig.sin_pitch, trig.sin_yaw);
}

// ʹ�����µ�ins����(���ܻ�û�б�EKFʹ��)����һ��ƽ����У����������ʸ��
// ���о����˺�����AC_AttitudeControl�б��������Σ���ϵ���������׻���
// return a smoothed and corrected gyro vector using the latest ins data (which may not have been consumed by the EKF yet)
Vector3f AP_AHRS_View::get_gyro_latest(void) const
{
    // �˴���get_gyro_latest���и�����ת����������֤�������������ȶ�����������̬�����ʿ���
    Matrix3f board_rotation;
    Vector3f gyro_latest = ahrs.get_gyro_latest();              // ��ȡ���µ�����������
    float board_rotate = RC_Channels::get_radio_in(CH_6);       // ��ȡң������6ͨ���ź�
    board_rotate= (board_rotate -1500) *0.2f;                   // ת��Ϊ�����б�Ƕ�
    board_rotation.from_euler(radians(0), radians(board_rotate), radians(0));

    gyro_latest.rotate(rotation);                               // ���������ݽ���ԭ������ĽǶ���ת
    gyro_latest = gyro_latest * board_rotation;                 // ���������ݽ����Զ��帩���Ƕ���ת
    return gyro_latest;                                         // ���ش���������������
}

// rotate a 2D vector from earth frame to body frame
Vector2f AP_AHRS_View::rotate_earth_to_body2D(const Vector2f &ef) const
{   return Vector2f(ef.x * trig.cos_yaw + ef.y * trig.sin_yaw,
                   -ef.x * trig.sin_yaw + ef.y * trig.cos_yaw);
}

// rotate a 2D vector from earth frame to body frame
Vector2f AP_AHRS_View::rotate_body_to_earth2D(const Vector2f &bf) const
{   return Vector2f(bf.x * trig.cos_yaw - bf.y * trig.sin_yaw,
                    bf.x * trig.sin_yaw + bf.y * trig.cos_yaw);
}
