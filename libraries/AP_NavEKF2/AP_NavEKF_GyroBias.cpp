#include <AP_HAL/AP_HAL.h>
#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <stdio.h>
extern const AP_HAL::HAL& hal;

// ����������ϵ������ƫ��״̬����Ϊ�㣬�����³�ʼ����Ӧ��Э����
// ����У׼�ľ���Ϊ0.5��/�룬����Ҫ�ھ�̬������ȡƽ��ֵ
// ����-����ʹ�÷�����У׼����
// reset the body axis gyro bias states to zero and re-initialise the corresponding covariances
// Assume that the calibration is performed to an accuracy of 0.5 deg/sec which will require averaging under static conditions
// WARNING - a non-blocking calibration method must be used
void NavEKF2_core::resetGyroBias(void)
{
    stateStruct.gyro_bias.zero();
    zeroRows(P,9,11);
    zeroCols(P,9,11);
    P[9][9] = sq(radians(0.5f * dtIMUavg));
    P[10][10] = P[9][9];
    P[11][11] = P[9][9];
}

// �������ض��ĳ�ʼ������ƫ�ò�ȷ���ȣ���λΪ�� vehicle specific initial gyro bias uncertainty in deg/sec
float NavEKF2_core::InitialGyroBiasUncertainty(void) const
{   return 2.5f;   }
