#include "Copter.h"

#if MODE_GUIDED_ENABLED == ENABLED

// Init and run calls for guided flight mode 初始化并运行引导飞行模式的调用

// 初始化引导控制器 guided_init - initialise guided controller 
bool ModeYawGuide::init(bool ignore_checks)
{
    // start in position control mode
    pos_control_start(); // 初始化引导模式的位置控制器
    return true;
}

// initialise guided mode's position controller 初始化引导模式的位置控制器
void ModeYawGuide::pos_control_start()
{
    // initialise waypoint and spline controller 初始化航点和样条曲线控制器
    wp_nav->wp_and_spline_init();

    // initialise wpnav to stopping point 初始化导航航点为停止点
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);

    // no need to check return status because terrain data is not used 不需要检查返回状态，因为没有使用地形数据
    wp_nav->set_wp_destination(stopping_point, false);

    // initialise yaw 初始化航向
    auto_yaw.set_mode_to_default(false);
}

// guided_set_destination - sets guided mode's target destination
// Returns true if the fence is enabled and guided waypoint is within the fence
// else return false if the waypoint is outside the fence
// 设置引导模式的目标目的地
// 如果栅栏被启用并且引导的路标点在栅栏内，则返回true，否则如果路径点在栅栏外，则返回false
bool ModeYawGuide::set_destination(const Vector3f& destination, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    // ensure we are in position control mode
    if (guided_mode != Guided_WP) {
        pos_control_start();
    }

#if AC_FENCE == ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination);
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // 设置偏航状态 set yaw state 
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // 设置航点目的地 no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(destination, false);

    // 记录航点日志 log target
    copter.Log_Write_GuidedTarget(guided_mode, destination, Vector3f());
    return true;
}

// 获取目标航点
bool ModeYawGuide::get_wp(Location& destination)
{
    if (guided_mode != Guided_WP) {
        return false;
    }
    return wp_nav->get_oa_wp_destination(destination);
}

// sets guided mode's target from a Location object
// returns false if destination could not be set (probably caused by missing terrain data)
// or if the fence is enabled and guided waypoint is outside the fence
// 从定位对象中设置引导模式的目标（设置的是标准经纬度目标航点）
// 如果无法设置目的地，或围栏启用，引导航点在围栏外，则返回false(可能是由于缺少地形数据造成的)
bool ModeYawGuide::set_destination(const Location& dest_loc, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    // ensure we are in position control mode
    if (guided_mode != Guided_WP) {
        pos_control_start();
    }

#if AC_FENCE == ENABLED
    // reject destination outside the fence.
    // Note: there is a danger that a target specified as a terrain altitude might not be checked if the conversion to alt-above-home fails
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    if (!wp_nav->set_wp_destination(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        // failure is propagated to GCS with NAK
        return false;
    }

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // log target
    copter.Log_Write_GuidedTarget(guided_mode, Vector3f(dest_loc.lat, dest_loc.lng, dest_loc.alt),Vector3f());
    return true;
}


// 引导模式以400hz运行 guided_run - runs the guided controller
// should be called at 100hz or more
void ModeYawGuide::run()
{    // run position controller
    pos_control_run();
}

// 运行引导模式位置控制器 guided_pos_control_run - runs the guided position controller
// called from guided_run
void ModeYawGuide::pos_control_run()
{
    // 处理飞手的航向输入，将目标航向速率设为零 process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) { // 如果遥控器没有失控保护
        // 将遥控器的航向摇杆直接映射为期望/目标航向速率 get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) { // 如果目标航向速率不为零
            auto_yaw.set_mode(AUTO_YAW_HOLD); // 则设为自动航向锁定
        }
    }

    // 如果没有解锁，则将油门设为零并立即退出 if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;    }

    // 将电机设为满油门行程 set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 运行航点控制器，控制水平方向的运动 run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav()); // 主要运行update_wpnav

    // 运行竖直位置控制器，应当已经更新目标高度 call z-axis position controller 
    // wpnav should have already updated it's alt target
    pos_control->update_z_controller();

    // 水平和竖直的位置控制器运行后，会得到三个轴的目标角度，赋值给下列函数
    // 运行姿态控制器 call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) { // 如果处于自动航向锁定模式
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) { // 和上面一样
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());
    } else { // 三个轴全部为角度控制
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
}

// helper function to update position controller's desired velocity while respecting acceleration limits
void ModeYawGuide::set_desired_velocity_with_accel_and_fence_limits(const Vector3f& vel_des)
{
    // get current desired velocity
    Vector3f curr_vel_des = pos_control->get_desired_velocity();

    // get change in desired velocity
    Vector3f vel_delta = vel_des - curr_vel_des;

    // limit xy change
    float vel_delta_xy = safe_sqrt(sq(vel_delta.x)+sq(vel_delta.y));
    float vel_delta_xy_max = G_Dt * pos_control->get_max_accel_xy();
    float ratio_xy = 1.0f;
    if (!is_zero(vel_delta_xy) && (vel_delta_xy > vel_delta_xy_max)) {
        ratio_xy = vel_delta_xy_max / vel_delta_xy;
    }
    curr_vel_des.x += (vel_delta.x * ratio_xy);
    curr_vel_des.y += (vel_delta.y * ratio_xy);

    // limit z change
    float vel_delta_z_max = G_Dt * pos_control->get_max_accel_z();
    curr_vel_des.z += constrain_float(vel_delta.z, -vel_delta_z_max, vel_delta_z_max);

#if AC_AVOID_ENABLED
    // limit the velocity to prevent fence violations
    copter.avoid.adjust_velocity(pos_control->get_pos_xy_p().kP(), pos_control->get_max_accel_xy(), curr_vel_des, G_Dt);
    // get avoidance adjusted climb rate
    curr_vel_des.z = get_avoidance_adjusted_climbrate(curr_vel_des.z);
#endif

    // update position controller with new target
    pos_control->set_desired_velocity(curr_vel_des);
}

// helper function to set yaw state and targets
void ModeYawGuide::set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle)
{
    if (use_yaw) {
        auto_yaw.set_fixed_yaw(yaw_cd * 0.01f, 0.0f, 0, relative_angle);
    } else if (use_yaw_rate) {
        auto_yaw.set_rate(yaw_rate_cds);
    }
}

// Guided Limit code

// guided_limit_clear - clear/turn off guided limits
void ModeYawGuide::limit_clear()
{
    guided_limit.timeout_ms = 0;
    guided_limit.alt_min_cm = 0.0f;
    guided_limit.alt_max_cm = 0.0f;
    guided_limit.horiz_max_cm = 0.0f;
}

// guided_limit_set - set guided timeout and movement limits
void ModeYawGuide::limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm)
{
    guided_limit.timeout_ms = timeout_ms;
    guided_limit.alt_min_cm = alt_min_cm;
    guided_limit.alt_max_cm = alt_max_cm;
    guided_limit.horiz_max_cm = horiz_max_cm;
}

// guided_limit_init_time_and_pos - initialise guided start time and position as reference for limit checking
//  only called from AUTO mode's auto_nav_guided_start function
void ModeYawGuide::limit_init_time_and_pos()
{
    // initialise start time
    guided_limit.start_time = AP_HAL::millis();

    // initialise start position from current position
    guided_limit.start_pos = inertial_nav.get_position();
}

// guided_limit_check - returns true if guided mode has breached a limit
//  used when guided is invoked from the NAV_GUIDED_ENABLE mission command
bool ModeYawGuide::limit_check()
{
    // check if we have passed the timeout
    if ((guided_limit.timeout_ms > 0) && (millis() - guided_limit.start_time >= guided_limit.timeout_ms)) {
        return true;
    }

    // get current location
    const Vector3f& curr_pos = inertial_nav.get_position();

    // check if we have gone below min alt
    if (!is_zero(guided_limit.alt_min_cm) && (curr_pos.z < guided_limit.alt_min_cm)) {
        return true;
    }

    // check if we have gone above max alt
    if (!is_zero(guided_limit.alt_max_cm) && (curr_pos.z > guided_limit.alt_max_cm)) {
        return true;
    }

    // check if we have gone beyond horizontal limit
    if (guided_limit.horiz_max_cm > 0.0f) {
        float horiz_move = get_horizontal_distance_cm(guided_limit.start_pos, curr_pos);
        if (horiz_move > guided_limit.horiz_max_cm) {
            return true;
        }
    }

    // if we got this far we must be within limits
    return false;
}


uint32_t ModeYawGuide::wp_distance() const
{
    switch(mode()) {
    case Guided_WP:
        return wp_nav->get_wp_distance_to_destination();
        break;
    case Guided_PosVel:
        return pos_control->get_distance_to_target();
        break;
    default:
        return 0;
    }
}

int32_t ModeYawGuide::wp_bearing() const
{
    switch(mode()) {
    case Guided_WP:
        return wp_nav->get_wp_bearing_to_destination();
        break;
    case Guided_PosVel:
        return pos_control->get_bearing_to_target();
        break;
    default:
        return 0;
    }
}

float ModeYawGuide::crosstrack_error() const
{
    if (mode() == Guided_WP) {
        return wp_nav->crosstrack_error();
    } else {
        return 0;
    }
}

#endif
