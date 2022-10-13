// RC_Channels.cpp - class containing an array of RC_Channel objects

#include <stdlib.h>
#include <cmath>
#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>
#include "RC_Channel.h"

uint32_t now_ms;
uint32_t last_ms;
uint32_t dt_ms;
uint16_t chan6 = 1515;
uint8_t  unit_time = 15; // 每增加1个PWM单位所需时间
/*
  channels group object constructor
 */
RC_Channels::RC_Channels(void)
{
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("RC_Channels must be singleton");
    }
    _singleton = this;
}

void RC_Channels::init(void)
{
    // setup ch_in on channels
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        channel(i)->ch_in = i;
    }

    init_aux_all();
}

uint8_t RC_Channels::get_radio_in(uint16_t *chans, const uint8_t num_channels)
{
    memset(chans, 0, num_channels*sizeof(*chans));
    const uint8_t read_channels = MIN(num_channels, NUM_RC_CHANNELS);
    now_ms = AP_HAL::millis();
    dt_ms = dt_ms + now_ms - last_ms;

    for (uint8_t i = 0; i < read_channels; i++) 
    {
        if ( i == 6 )
        {   
            if( chans[5] <= 1265 ) // 旋钮位于下端，则向下移动
            {   if ( dt_ms > unit_time ) { chan6 = chan6 - 1; dt_ms = 0; }
                else  { chan6 = chan6; } }
            if( chans[5] > 1265 && chans[5] < 1465 ) // 旋钮位于下部，则保持不动
            {   chan6 = chan6; }
            if( chans[5] >= 1465 && chans[5] <= 1565 ) // 旋钮位于中间，则向中间移动
            { 
                if( chan6 < 1514 ) // 如果在下，就上行
                {   if ( dt_ms > unit_time )  { chan6 = chan6 + 1; dt_ms = 0; }
                    else  { chan6 = chan6; } }
                if( chan6 >= 1514 && chan6 <= 1516 )  { chan6 = 1515; }
                if( chan6 > 1516 ) // 如果在上，就下行
                {   if ( dt_ms > unit_time )  { chan6 = chan6 - 1; dt_ms = 0; }
                    else  { chan6 = chan6; } }
            } 
            if( chans[5] > 1565 && chans[5] < 1765 ) // 旋钮位于上部，则保持不动
            {   chan6 = chan6; }
            if( chans[5] >= 1765 ) // 旋钮位于上端，则向上移动
            {   if ( dt_ms > unit_time )  { chan6 = chan6 + 1; dt_ms = 0; }
                else  { chan6 = chan6; } }

            if( chan6 >= 1965 ) { chan6 = 1965; } // 第7通道最大限幅为1965
            if( chan6 <= 1065 ) { chan6 = 1065; } // 第7通道最小限幅为1065


            // 0度，45度，90度
            // if( chans[5] <= 1265 ) // 旋钮位于下端，则向下移动
            // {   if ( dt_ms > unit_time ) { chan6 = chan6 - 1; dt_ms = 0; }
            //     else  { chan6 = chan6; } }
            // if( chans[5] > 1265 && chans[5] < 1465 ) // 旋钮位于下部，则保持不动
            // {   chan6 = chan6; }
            // if( chans[5] >= 1465 && chans[5] <= 1565 ) // 旋钮位于中间，则向中间移动
            // { 
            //     if( chan6 < 1739 ) // 如果在下，就上行
            //     {   if ( dt_ms > unit_time )  { chan6 = chan6 + 1; dt_ms = 0; }
            //         else  { chan6 = chan6; } }
            //     if( chan6 >= 1739 && chan6 <= 1741 )  { chan6 = 1740; }
            //     if( chan6 > 1741 ) // 如果在上，就下行
            //     {   if ( dt_ms > unit_time )  { chan6 = chan6 - 1; dt_ms = 0; }
            //         else  { chan6 = chan6; } }
            // } 
            // if( chans[5] > 1565 && chans[5] < 1765 ) // 旋钮位于上部，则保持不动
            // {   chan6 = chan6; }
            // if( chans[5] >= 1765 ) // 旋钮位于上端，则向上移动
            // {   if ( dt_ms > unit_time )  { chan6 = chan6 + 1; dt_ms = 0; }
            //     else  { chan6 = chan6; } }

            // if( chan6 >= 1965 ) { chan6 = 1965; } // 第7通道最大限幅为1965
            // if( chan6 <= 1515 ) { chan6 = 1515; } // 第7通道最小限幅为1515


            // chans[6] = chan6;    // 自动变姿，为第七通道赋值
            chans[6] = chans[5]; // 手动变姿，将第六通道赋值给第七通道
        }
        else
        { chans[i] = channel(i)->get_radio_in(); }
    }

    last_ms = AP_HAL::millis();
    return read_channels;
}

// update all the input channels
bool RC_Channels::read_input(void)
{
    if (!hal.rcin->new_input() && !has_new_overrides) { return false; }
    has_new_overrides = false;
    bool success = false;
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) { success |= channel(i)->update(); }

    return success;
}

uint8_t RC_Channels::get_valid_channel_count(void)
{
    return MIN(NUM_RC_CHANNELS, hal.rcin->num_channels());
}

int16_t RC_Channels::get_receiver_rssi(void)
{
    return hal.rcin->get_rssi();
}

void RC_Channels::clear_overrides(void)
{
    RC_Channels &_rc = rc();
    for (uint8_t i = 0; i < NUM_RC_CHANNELS; i++) {
        _rc.channel(i)->clear_override();
    }
    // we really should set has_new_overrides to true, and rerun read_input from
    // the vehicle code however doing so currently breaks the failsafe system on
    // copter and plane, RC_Channels needs to control failsafes to resolve this
}

void RC_Channels::set_override(const uint8_t chan, const int16_t value, const uint32_t timestamp_ms)
{
    RC_Channels &_rc = rc();
    if (chan < NUM_RC_CHANNELS) {
        _rc.channel(chan)->set_override(value, timestamp_ms);
    }
}

bool RC_Channels::has_active_overrides()
{
    RC_Channels &_rc = rc();
    for (uint8_t i = 0; i < NUM_RC_CHANNELS; i++) {
        if (_rc.channel(i)->has_override()) {
            return true;
        }
    }

    return false;
}

bool RC_Channels::receiver_bind(const int dsmMode)
{
    return hal.rcin->rc_bind(dsmMode);
}


// support for auxillary switches:
// read_aux_switches - checks aux switch positions and invokes configured actions
void RC_Channels::read_aux_all()
{
    if (!has_valid_input()) {
        // exit immediately when no RC input
        return;
    }
    bool need_log = false;

    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        RC_Channel *c = channel(i);
        if (c == nullptr) {
            continue;
        }
        need_log |= c->read_aux();
    }
    if (need_log) {
        // guarantee that we log when a switch changes
        AP::logger().Write_RCIN();
    }
}

void RC_Channels::init_aux_all()
{
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        RC_Channel *c = channel(i);
        if (c == nullptr) {
            continue;
        }
        c->init_aux();
    }
    reset_mode_switch();
}

//
// Support for mode switches
//
RC_Channel *RC_Channels::flight_mode_channel()
{
    const int8_t num = flight_mode_channel_number();
    if (num <= 0) {
        return nullptr;
    }
    if (num >= NUM_RC_CHANNELS) {
        return nullptr;
    }
    return channel(num-1);
}

void RC_Channels::reset_mode_switch()
{
    RC_Channel *c = flight_mode_channel();
    if (c == nullptr) {
        return;
    }
    c->reset_mode_switch();
}

void RC_Channels::read_mode_switch()
{
    if (!has_valid_input()) {
        // exit immediately when no RC input
        return;
    }
    RC_Channel *c = flight_mode_channel();
    if (c == nullptr) {
        return;
    }
    c->read_mode_switch();
}


// singleton instance
RC_Channels *RC_Channels::_singleton;


RC_Channels &rc()
{
    return *RC_Channels::get_singleton();
}
