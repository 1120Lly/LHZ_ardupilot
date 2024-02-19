#pragma once
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Param/AP_Param.h>

class AP_FOCCAN : public AP_CANDriver {
public:
  AP_FOCCAN() {
    if (_singleton != nullptr) { return; }
    _singleton = this;
  }

  CLASS_NO_COPY(AP_FOCCAN);
  static const struct AP_Param::GroupInfo var_info[];

  // initialize PiccoloCAN bus
  void init(uint8_t driver_index, bool enable_filters) override;
  bool add_interface(AP_HAL::CANIface *can_iface) override;
  int16_t getSpeed(uint8_t id) { return real_speed[id]; } // 获取转速
  void setCurrent(uint8_t id, int16_t _cuur) { target_current[id] = _cuur; }
  int16_t getCurrent(uint8_t id) { return target_current[id]; } // 用于在调试时获取电流值
  bool read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout);
  bool write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout);
  void update();
  static AP_FOCCAN *get_foccan(uint8_t driver_index);
  static AP_FOCCAN *get_singleton() { return _singleton; }

private:
  static AP_FOCCAN *_singleton;

  /*CAN发送或是接收的ID*/
  typedef enum {
    CAN_ALL_ID = 0x280,
    CAN_M1_ID = 0x141,
    CAN_M2_ID = 0x142,
    CAN_M3_ID = 0x143,
    CAN_M4_ID = 0x144,
  } CAN_Message_ID;

  AP_Int8 _num_poles;
  AP_HAL::CANIface *_can_iface;
  uint8_t _driver_index;
  bool _initialized;
  char _thread_name[16];
  void handle_motor_measure(AP_HAL::CANFrame &frame, uint8_t id);
  bool send_current(const int16_t d1, const int16_t d2, const int16_t d3, const int16_t d4);
  int16_t target_current[4];
  int16_t real_speed[4];
  uint8_t send_current_buffer[8];
};

namespace AP { AP_FOCCAN *FOCCAN(); 
};