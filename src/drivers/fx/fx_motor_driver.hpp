// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 lixiangyi-2006

#pragma once

#include <atomic>
#include <string>

#include "motor_driver.hpp"
#include "protocol/can/socket_can.hpp"
#include "utils.hpp"

// FX error codes (reported via fault frame 0x15)
enum FXError {
    FX_NO_ERROR           = 0x00,
    FX_OVER_TEMP          = 0x01,
    FX_DRIVER_FAULT       = 0x02,
    FX_UNDER_VOLTAGE      = 0x04,
    FX_OVER_VOLTAGE       = 0x08,
    FX_ENCODER_FAULT      = 0x80,
    FX_STALL_OVERLOAD     = 0x4000,
};

enum FX_Motor_Model {
    FX_57C,
    FX_120C,
    FX_Num_Of_Model
};

// FX Motor modes
enum FXMotorMode : uint8_t {
    FX_MODE_MIT      = 0x00,
    FX_MODE_POSITION = 0x05,
    FX_MODE_SPEED    = 0x02,
    FX_MODE_CURRENT  = 0x03,
};

// FX CAN Message IDs (communication types, placed in bits 28-24)
enum FXMsgId : uint8_t {
    FX_MSG_GET_ID        = 0x00,
    FX_MSG_CONTROL       = 0x01,
    FX_MSG_FEEDBACK      = 0x02,
    FX_MSG_ENABLE        = 0x03,
    FX_MSG_STOP          = 0x04,
    FX_MSG_SET_ZERO      = 0x06,
    FX_MSG_SET_CANID     = 0x07,
    FX_MSG_READ_PARAM    = 0x11,
    FX_MSG_WRITE_PARAM   = 0x12,
    FX_MSG_FAULT         = 0x15,
    FX_MSG_SAVE          = 0x16,
    FX_MSG_BAUDRATE      = 0x17,
    FX_MSG_ACTIVE_REPORT = 0x18,
    FX_MSG_PROTOCOL      = 0x19,
};

// Parameter ranges for MIT mode (configurable per motor)
typedef struct {
    float PosMax;
    float SpdMax;
    float TauMax;
    float OKpMax;
    float OKdMax;
} FX_Limit_Param;

class FXMotorDriver : public MotorDriver {
   public:
    FXMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface,
                   FX_Motor_Model motor_model, double motor_zero_offset = 0.0);
    ~FXMotorDriver();

    virtual void lock_motor() override;
    virtual void unlock_motor() override;
    virtual uint8_t init_motor() override;
    virtual void deinit_motor() override;
    virtual bool set_motor_zero() override;
    virtual bool write_motor_flash() override;
    virtual void get_motor_param(uint8_t param_cmd) override;

    virtual void motor_pos_cmd(float pos, float spd, bool ignore_limit) override;
    virtual void motor_spd_cmd(float spd) override;
    virtual void motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) override;
    virtual void motor_mit_cmd(float* f_p, float* f_v, float* f_kp, float* f_kd, float* f_t) override;
    virtual void set_motor_control_mode(uint8_t motor_control_mode) override;
    virtual int get_response_count() const override {
        return response_count_;
    }
    virtual void set_motor_id(uint8_t old_id, uint8_t new_id) override;
    virtual void reset_motor_id() override;
    virtual void refresh_motor_status() override;
    virtual void clear_motor_error() override;

   private:
    uint8_t motor_index_{0};

    std::atomic<int> response_count_{0};
    FX_Motor_Model motor_model_;
    FX_Limit_Param limit_param_;
    uint8_t device_id_;
    
    //Only CAN communication is supported.
    std::string can_interface_;              
    enum CommType { CAN } comm_type_; 

    void set_motor_zero_fx();
    void clear_motor_error_fx();
    void write_param_float_fx(uint16_t index, float value);
    void write_param_uint8_fx(uint16_t index, uint8_t value);
    void write_register_fx(uint16_t index, int32_t value);
    void save_register_fx();

    virtual void can_rx_cbk(const can_frame& rx_frame);
    std::shared_ptr<MotorsSocketCAN> can_;

    inline static std::mutex bus_registry_mutex_;
    inline static std::unordered_map<std::string, std::vector<FXMotorDriver*>> bus_registry_;
};
