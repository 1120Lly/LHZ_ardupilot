#include "AP_FOCCAN.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Param/AP_Param.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <stdio.h>

extern const AP_HAL::HAL &hal;

void AP_FOCCAN::init(uint8_t driver_index, bool enable_filters) // 初始化
{   _driver_index = driver_index;
    if (_initialized) { return; } // 如果为真，则已经初始化过，直接返回，避免重复初始化
    if (!hal.scheduler->thread_create( // 创建一个新的线程，用于执行类成员函数update
    FUNCTOR_BIND_MEMBER(&AP_FOCCAN::update, void), _thread_name, 4096,
    AP_HAL::Scheduler::PRIORITY_MAIN, 1)) { return; } // 如果为假，则线程创建失败
    _initialized = true; // 已经成功初始化
    snprintf(_thread_name, sizeof(_thread_name), "FOCCAN_%u", driver_index);
}

bool AP_FOCCAN::add_interface(AP_HAL::CANIface *can_iface) // 添加CAN接口
{   if (_can_iface != nullptr) { return false; } // 已经存在一个CAN接口，添加接口失败
    _can_iface = can_iface; // 将can_iface指针赋值给_can_iface，表示成功添加CAN接口
    if (_can_iface == nullptr) { return false; }
    if (!_can_iface->is_initialized()) { return false; }
    return true;
}

AP_FOCCAN *AP_FOCCAN::get_foccan(uint8_t driver_index) // 通过驱动器索引获取对象
{   if (driver_index >= AP::can().get_num_drivers() ||
    AP::can().get_driver_type(driver_index) != AP_CANManager::Driver_Type_FOCCAN) 
    { return nullptr; }
    return static_cast<AP_FOCCAN*>(AP::can().get_driver(driver_index));
}

void AP_FOCCAN::update() 
{   AP_HAL::CANFrame txFrame{}; // 创建用于发送CAN帧的对象
    AP_HAL::CANFrame rxFrame{}; // 创建用于接收CAN帧的对象
    while (true) 
    {   if (!_initialized) { hal.scheduler->delay_microseconds(10000);
            continue; } // 如果未初始化，则延迟10毫秒，继续下一次循环
        send_current(target_current[0], target_current[1], target_current[2], target_current[3]);
        hal.scheduler->delay_microseconds(2500); // 延迟2.5毫秒，等待一段时间
        uint64_t timeout = AP_HAL::micros64() + 250ULL; // 获取当前微秒数并加上250，得到超时时间
        while (read_frame(rxFrame, timeout)) // 查找CAN总线上的任何消息响应，读取CAN帧
        {   switch (rxFrame.id) { // 根据接收到的CAN帧ID进行不同处理
            case CAN_M1_ID:
            case CAN_M2_ID:
            case CAN_M3_ID:
            case CAN_M4_ID:
            uint8_t i = rxFrame.id - CAN_M1_ID;
            handle_motor_measure(rxFrame, i); } } } // 处理电机的测量数据
}

// read frame on CAN bus, returns true on succses
bool AP_FOCCAN::read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout) 
{   if (!_initialized) { return false; }
    bool read_select = true;
    bool write_select = false;
    bool ret = _can_iface->select(read_select, write_select, nullptr, timeout);
    if (!ret || !read_select) { return false; } // No frame available
    uint64_t time;
    AP_HAL::CANIface::CanIOFlags flags{};
    return (_can_iface->receive(recv_frame, time, flags) == 1);
}

// write frame on CAN bus, returns true on success
bool AP_FOCCAN::write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout) 
{   if (!_initialized) { return false; }
    bool read_select = false;
    bool write_select = true;
    bool ret = _can_iface->select(read_select, write_select, &out_frame, timeout);
    if (!ret || !write_select) { return false; }
    return (_can_iface->send(out_frame, timeout, AP_HAL::CANIface::AbortOnError) == 1);
}

// parse inbound frames
void AP_FOCCAN::handle_motor_measure(AP_HAL::CANFrame &frame, uint8_t id) 
{   real_speed[id] = (frame.data[4] | frame.data[5] << 8 ); }

bool AP_FOCCAN::send_current(const int16_t m1, const int16_t m2, const int16_t m3, const int16_t mode) 
{   
    AP_HAL::CANFrame frame = AP_HAL::CANFrame(0x141, send_current_buffer, 0x08);  
    if (mode < 0)  // 拨杆位于上位，动量摆模式
    {   frame.data[0] = 0xA1; // 转矩闭环控制命令
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = m1;
        frame.data[5] = m1 >> 8;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
    } else         // 拨杆位于下位，常规模式，无电流
    {   frame.data[0] = 0xA1; // 转矩闭环控制命令
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        // frame.data[0] = 0xA4; // 位置闭环控制命令2
        // frame.data[1] = 0x00;
        // frame.data[2] = 0xF0; // 转速限制为240dps
        // frame.data[3] = 0x00;
        // frame.data[4] = 0xB0; // 位置保持在12度: 1200 = 04 B0
        // frame.data[5] = 0x04;
        // frame.data[6] = 0x00;
        // frame.data[7] = 0x00;
    }
    uint64_t timeout_us = 10000UL;
    return write_frame(frame, timeout_us);
}

AP_FOCCAN *AP_FOCCAN::_singleton;

namespace AP { AP_FOCCAN *FOCCAN() { return AP_FOCCAN::get_singleton(); 
} }; // namespace AP