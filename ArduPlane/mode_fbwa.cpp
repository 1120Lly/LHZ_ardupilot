#include "mode.h"
#include "Plane.h"

void ModeFBWA::update()
{
    // set nav_roll and nav_pitch using sticks，通过遥控器控制滚转和俯仰
    // 从遥控器的滚转通道获取标准化的输入值
    plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    // 更新负载因子
    plane.update_load_factor();
    // 控制俯仰角度，从遥控器的俯仰通道获取标准化输入值，抬头和低头
    float pitch_input = plane.channel_pitch->norm_input();
    if (pitch_input > 0) { plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max*100;
    } else { plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min*100); }
    // 调整俯仰角和油门的关系，确保在不同的俯仰角下，飞机能保持适当的飞行速度
    // 在油门较低时（尤其是在着陆或减速时）自动将机头略微向下倾斜，避免飞机因速度过低而失速
    plane.adjust_nav_pitch_throttle();
    // 限制俯仰角度范围
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
    
    // 处理倒飞情况
    if (plane.fly_inverted()) { plane.nav_pitch_cd = -plane.nav_pitch_cd; }
    // 处理遥控信号丢失的紧急情况
    if (plane.failsafe.rc_failsafe && plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
        // FBWA failsafe glide
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;
        SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
    }
    
    // 处理FBWA尾拖模式的起飞，通过辅助通道判断，用于保证转向轮压在地面上，本项目不需要这个功能
    RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FBWA_TAILDRAGGER);
    if (chan != nullptr) {
        // check for the user enabling FBWA taildrag takeoff mode
        bool tdrag_mode = chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH;
        if (tdrag_mode && !plane.auto_state.fbwa_tdrag_takeoff_mode) {
            if (plane.auto_state.highest_airspeed < plane.g.takeoff_tdrag_speed1) {
                plane.auto_state.fbwa_tdrag_takeoff_mode = true;
                plane.gcs().send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
            }
        }
    }
}
