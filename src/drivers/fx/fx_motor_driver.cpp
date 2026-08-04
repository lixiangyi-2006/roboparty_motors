// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 lixiangyi-2006

#include "fx_motor_driver.hpp"

FX_Limit_Param fx_limit_param[FX_Num_Of_Model] = {
    {12.57f, 33.0f, 14.0f, 500.0f, 5.0f},    // FX_57C
    {12.57f, 15.0f, 120.0f, 5000.0f, 100.0f}, // FX_120C
};

FXMotorDriver::FXMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface,
                             FX_Motor_Model motor_model, double motor_zero_offset)
    : MotorDriver(), motor_model_(motor_model) {
    if (interface_type != "can") {
        throw std::runtime_error("FX driver only supports CAN interface");
    }
    motor_id_ = motor_id;
    limit_param_ = fx_limit_param[motor_model_];
    can_interface_ = can_interface;
    motor_zero_offset_ = motor_zero_offset;
    device_id_ = motor_id & 0x7F;
    motor_index_ = (device_id_ > 0 && device_id_ <= 7) ? (device_id_ - 1) : 0;

    comm_type_ = CommType::CAN;
    can_ = MotorsSocketCAN::get(can_interface);

    CanCbkFunc can_callback = std::bind(&FXMotorDriver::can_rx_cbk, this, std::placeholders::_1);
    can_->set_can_key_extractor([](const can_frame& frame) -> CanCbkId {
        uint32_t raw_id = frame.can_id & CAN_EFF_MASK;
        return static_cast<CanCbkId>((raw_id >> 8) & 0x7F);
    });
    can_->add_can_callback(can_callback, static_cast<CanCbkId>(device_id_));
    std::lock_guard<std::mutex> lock(bus_registry_mutex_);
    bus_registry_[can_interface].push_back(this);
}

FXMotorDriver::~FXMotorDriver() {
    if (comm_type_ == CommType::CAN) {
        can_->remove_can_callback(static_cast<CanCbkId>(device_id_));
        std::lock_guard<std::mutex> lock(bus_registry_mutex_);
        auto it = bus_registry_.find(can_interface_);
        if (it != bus_registry_.end()) {
            auto& motors = it->second;
            motors.erase(std::remove(motors.begin(), motors.end(), this), motors.end());
            if (motors.empty()) {
                bus_registry_.erase(it);
            }
        }
    }
}

void FXMotorDriver::lock_motor() {
    if (comm_type_ == CommType::CAN) {
        can_frame tx_frame{};
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(FX_MSG_ENABLE) << 24) | static_cast<uint32_t>(device_id_);
        tx_frame.can_dlc = 0x00;
        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void FXMotorDriver::unlock_motor() {
    if (comm_type_ == CommType::CAN) {
        can_frame tx_frame{};
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(FX_MSG_STOP) << 24) | static_cast<uint32_t>(device_id_);
        tx_frame.can_dlc = 0x00;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

uint8_t FXMotorDriver::init_motor() {
    if (comm_type_ == CommType::CAN) {
        FXMotorDriver::unlock_motor();
        Timer::sleep_for(normal_sleep_time);
        FXMotorDriver::set_motor_control_mode(MIT);
        Timer::sleep_for(normal_sleep_time);
        FXMotorDriver::lock_motor();
        Timer::sleep_for(normal_sleep_time);
        FXMotorDriver::refresh_motor_status();
        Timer::sleep_for(normal_sleep_time);
    }

    switch (error_id_) {
        case FX_OVER_VOLTAGE:
            return FX_OVER_VOLTAGE;
        case FX_UNDER_VOLTAGE:
            return FX_UNDER_VOLTAGE;
        case FX_OVER_TEMP:
            return FX_OVER_TEMP;
        case FX_DRIVER_FAULT:
            return FX_DRIVER_FAULT;
        case FX_ENCODER_FAULT:
            return FX_ENCODER_FAULT;
        default:
            return error_id_;
    }
    return 0;
}

void FXMotorDriver::deinit_motor() {
    FXMotorDriver::unlock_motor();
    Timer::sleep_for(normal_sleep_time);
}

bool FXMotorDriver::write_motor_flash() {
    if (comm_type_ == CommType::CAN) {
        can_frame tx_frame{};

        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(FX_MSG_SAVE) << 24) |static_cast<uint32_t>(device_id_);
        tx_frame.can_dlc = 0x08;
        tx_frame.data[0] = 0x01;
        tx_frame.data[1] = 0x02;
        tx_frame.data[2] = 0x03;
        tx_frame.data[3] = 0x04;
        tx_frame.data[4] = 0x05;
        tx_frame.data[5] = 0x06;
        tx_frame.data[6] = 0x07;
        tx_frame.data[7] = 0x08;

        can_->transmit(tx_frame);
        {
            response_count_++;
        }
        Timer::sleep_for(setup_sleep_time);
    }

    return true;
}

bool FXMotorDriver::set_motor_zero() {
    FXMotorDriver::set_motor_zero_fx();
    Timer::sleep_for(setup_sleep_time);
    FXMotorDriver::refresh_motor_status();
    Timer::sleep_for(setup_sleep_time);
    logger_->info("motor_id: {0}\tposition: {1}", motor_id_, get_motor_pos());
    if (std::abs(get_motor_pos()) > judgment_accuracy_threshold) {
        logger_->warn("set zero error");
        return false;
    } else {
        logger_->info("set zero success");
        return true;
    }
}

void FXMotorDriver::can_rx_cbk(const can_frame& rx_frame) {
    {
        response_count_ = 0;
    }
    if (rx_frame.can_dlc < 8) return;
    uint8_t comm_type = static_cast<uint8_t>((rx_frame.can_id >> 24) & 0x1F);
    if (comm_type != FX_MSG_GET_ID && comm_type != FX_MSG_FEEDBACK) return;
    uint8_t target_id = static_cast<uint8_t>((rx_frame.can_id >> 8) & 0x7F);
    if (target_id != device_id_) return;

    // GET_ID (0x00) replies use a different data layout; only parse feedback frames
    if (comm_type != FX_MSG_FEEDBACK) return;

    const uint8_t* data = rx_frame.data;
    int angle_uint = (data[0] << 8) | data[1];
    int spd_uint = (data[2] << 8) | data[3];
    int temp_raw = (data[6] << 8) | data[7];

    motor_pos_ = range_map(static_cast<float>(angle_uint), 0.0f, 65535.0f, -limit_param_.PosMax, limit_param_.PosMax) + static_cast<float>(motor_zero_offset_);
    motor_spd_ = range_map(static_cast<float>(spd_uint), 0.0f, 65535.0f, -limit_param_.SpdMax, limit_param_.SpdMax);
    motor_current_ = range_map(static_cast<float>((data[4] << 8) | data[5]), 0.0f, 65535.0f, -limit_param_.TauMax, limit_param_.TauMax);
    motor_temperature_ = temp_raw / 10.0f;
}

void FXMotorDriver::get_motor_param(uint8_t param_cmd) {
    if (comm_type_ == CommType::CAN) {
        can_frame tx_frame{};
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(FX_MSG_READ_PARAM) << 24) | static_cast<uint32_t>(device_id_);
        tx_frame.can_dlc = 0x02;
        tx_frame.data[0] = param_cmd;
        tx_frame.data[1] = 0x00;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void FXMotorDriver::motor_pos_cmd(float pos, float spd, bool ignore_limit) {
    if (motor_control_mode_ != POS) {
        set_motor_control_mode(POS);
        return;
    }
    float spd_rad = spd;
    union32_t rv_type_convert;
    rv_type_convert.f = spd_rad;

    if (comm_type_ == CommType::CAN) {
        can_frame tx_frame;
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(FX_MSG_WRITE_PARAM) << 24) | static_cast<uint32_t>(device_id_);
        tx_frame.can_dlc = 0x08;
        tx_frame.data[0] = 0x17;
        tx_frame.data[1] = 0x70;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = 0x00;
        tx_frame.data[4] = rv_type_convert.buf[0];
        tx_frame.data[5] = rv_type_convert.buf[1];
        tx_frame.data[6] = rv_type_convert.buf[2];
        tx_frame.data[7] = rv_type_convert.buf[3];

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }

    float pos_rad = pos - static_cast<float>(motor_zero_offset_);
    rv_type_convert.f = pos_rad;
    if (comm_type_ == CommType::CAN) {
        can_frame tx_frame;
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(FX_MSG_WRITE_PARAM) << 24) | static_cast<uint32_t>(device_id_);
        tx_frame.can_dlc = 0x08;
        tx_frame.data[0] = 0x16;
        tx_frame.data[1] = 0x70;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = 0x00;
        tx_frame.data[4] = rv_type_convert.buf[0];
        tx_frame.data[5] = rv_type_convert.buf[1];
        tx_frame.data[6] = rv_type_convert.buf[2];
        tx_frame.data[7] = rv_type_convert.buf[3];

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void FXMotorDriver::motor_spd_cmd(float spd) {
    if (motor_control_mode_ != SPD) {
        set_motor_control_mode(SPD);
        return;
    }
    float spd_rad = spd;
    union32_t rv_type_convert;
    rv_type_convert.f = spd_rad;

    if (comm_type_ == CommType::CAN) {
        can_frame tx_frame;
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(FX_MSG_WRITE_PARAM) << 24) |static_cast<uint32_t>(device_id_);
        tx_frame.can_dlc = 0x08;
        tx_frame.data[0] = 0x0A;
        tx_frame.data[1] = 0x70;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = 0x00;
        tx_frame.data[4] = rv_type_convert.buf[0];
        tx_frame.data[5] = rv_type_convert.buf[1];
        tx_frame.data[6] = rv_type_convert.buf[2];
        tx_frame.data[7] = rv_type_convert.buf[3];

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void FXMotorDriver::motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
    if (motor_control_mode_ != MIT) {
        set_motor_control_mode(MIT);
        return;
    }
    
    //Offset and Clamping
    f_p = limit(f_p - static_cast<float>(motor_zero_offset_), -limit_param_.PosMax, limit_param_.PosMax);
    f_v = limit(f_v, -limit_param_.SpdMax, limit_param_.SpdMax);
    f_kp = limit(f_kp, 0.0f, limit_param_.OKpMax);
    f_kd = limit(f_kd, 0.0f, limit_param_.OKdMax);
    f_t = limit(f_t, -limit_param_.TauMax, limit_param_.TauMax);

    //Converting floating-point numbers to integers
    uint16_t p = range_map(f_p, -limit_param_.PosMax, limit_param_.PosMax, uint16_t(0), uint16_t(0xFFFF));
    uint16_t v = range_map(f_v, -limit_param_.SpdMax, limit_param_.SpdMax, uint16_t(0), uint16_t(0xFFFF));
    uint16_t kp = range_map(f_kp, 0.0f, limit_param_.OKpMax, uint16_t(0), uint16_t(0xFFFF));
    uint16_t kd = range_map(f_kd, 0.0f, limit_param_.OKdMax, uint16_t(0), uint16_t(0xFFFF));

    //Page 19 of the official manual states that frames in control mode contain only angle, angular velocity, KP, and KD, and do not include T.
    if (comm_type_ == CommType::CAN) {
        can_frame tx_frame;
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(FX_MSG_CONTROL) << 24) | static_cast<uint32_t>(device_id_);
        tx_frame.can_dlc = 0x08;
        tx_frame.data[0] = (p >> 8) & 0xFF;
        tx_frame.data[1] = p & 0xFF;
        tx_frame.data[2] = (v >> 8) & 0xFF;
        tx_frame.data[3] = v & 0xFF;
        tx_frame.data[4] = (kp >> 8) & 0xFF;
        tx_frame.data[5] = kp & 0xFF;
        tx_frame.data[6] = (kd >> 8) & 0xFF;
        tx_frame.data[7] = kd  & 0xFF;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void FXMotorDriver::motor_mit_cmd(float* f_p, float* f_v, float* f_kp, float* f_kd, float* f_t) {
    if (!f_p || !f_v || !f_kp || !f_kd || !f_t) {
        return;
    }
    if (motor_control_mode_ != MIT) {
        set_motor_control_mode(MIT);
        return;
    }

    std::lock_guard<std::mutex> lock(bus_registry_mutex_);
    auto it = bus_registry_.find(can_interface_);
    if (it != bus_registry_.end()) {
        for (FXMotorDriver* motor : it->second) {
            if (!motor || motor->motor_index_ >= 8) {
                continue;
            }
            const uint8_t slot = motor->motor_index_;

            float p_f, v_f, kp_f, kd_f, t_f;
            uint16_t p, v, kp, kd;

            p_f = limit(f_p[slot] - static_cast<float>(motor->motor_zero_offset_), -motor->limit_param_.PosMax, motor->limit_param_.PosMax);
            v_f = limit(f_v[slot], -motor->limit_param_.SpdMax, motor->limit_param_.SpdMax);
            kp_f = limit(f_kp[slot], 0.0f, motor->limit_param_.OKpMax);
            kd_f = limit(f_kd[slot], 0.0f, motor->limit_param_.OKdMax);
            t_f = limit(f_t[slot], -motor->limit_param_.TauMax, motor->limit_param_.TauMax);

            p = range_map(p_f, -motor->limit_param_.PosMax, motor->limit_param_.PosMax, uint16_t(0), uint16_t(0xFFFF));
            v = range_map(v_f, -motor->limit_param_.SpdMax, motor->limit_param_.SpdMax, uint16_t(0), uint16_t(0xFFFF));
            kp = range_map(kp_f, 0.0f, motor->limit_param_.OKpMax, uint16_t(0), uint16_t(0xFFFF));
            kd = range_map(kd_f, 0.0f, motor->limit_param_.OKdMax, uint16_t(0), uint16_t(0xFFFF));

            if (motor->comm_type_ == CommType::CAN) {
                can_frame tx_frame;
                tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(FX_MSG_CONTROL) << 24) | static_cast<uint32_t>(motor->device_id_);
                tx_frame.can_dlc = 0x08;
                tx_frame.data[0] = (p >> 8) & 0xFF;
                tx_frame.data[1] = p & 0xFF;
                tx_frame.data[2] = (v >> 8) & 0xFF;
                tx_frame.data[3] = v & 0xFF;
                tx_frame.data[4] = (kp >> 8) & 0xFF;
                tx_frame.data[5] = kp & 0xFF;
                tx_frame.data[6] = (kd >> 8) & 0xFF;
                tx_frame.data[7] = kd & 0xFF;

                motor->can_->transmit(tx_frame);
            }
            {
                motor->response_count_++;
            }
        }
    }
}

void FXMotorDriver::set_motor_control_mode(uint8_t motor_control_mode) {
    uint8_t fx_mode;
    switch (motor_control_mode) {
        case POS:
            fx_mode = FX_MODE_POSITION;
            break;
        case SPD:
            fx_mode = FX_MODE_SPEED;
            break;
        case MIT:
            fx_mode = FX_MODE_MIT;
            break;
        default:
            fx_mode = FX_MODE_POSITION;
            break;
    }

    if (comm_type_ == CommType::CAN) {
        can_frame tx_frame{};
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(FX_MSG_STOP) << 24) | static_cast<uint32_t>(device_id_);
        tx_frame.can_dlc = 0x00;

        can_->transmit(tx_frame);
        {
            response_count_++;
        }
        Timer::sleep_for(normal_sleep_time);

        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(FX_MSG_WRITE_PARAM) << 24) | static_cast<uint32_t>(device_id_);
        tx_frame.can_dlc = 0x08;
        tx_frame.data[0] = 0x05;
        tx_frame.data[1] = 0x70;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = 0x00;
        tx_frame.data[4] = fx_mode;
        tx_frame.data[5] = 0x00;
        tx_frame.data[6] = 0x00;
        tx_frame.data[7] = 0x00;

        can_->transmit(tx_frame);
        {
            response_count_++;
        }
        Timer::sleep_for(normal_sleep_time);

        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(FX_MSG_ENABLE) << 24) | static_cast<uint32_t>(device_id_);
        tx_frame.can_dlc = 0x00;

        can_->transmit(tx_frame);
        {
            response_count_++;
        }
        Timer::sleep_for(normal_sleep_time);
    }

    motor_control_mode_ = motor_control_mode;
}

void FXMotorDriver::set_motor_id(uint8_t old_id, uint8_t new_id) {
    if (comm_type_ == CommType::CAN) {
        can_frame tx_frame{};
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(FX_MSG_SET_CANID) << 24) |static_cast<uint32_t>(old_id & 0x7F);
        tx_frame.can_dlc = 0x01;
        tx_frame.data[0] = new_id & 0x7F;

        can_->transmit(tx_frame);
        {
            response_count_++;
        }
        Timer::sleep_for(setup_sleep_time);
    }
}

void FXMotorDriver::reset_motor_id() {
    FXMotorDriver::set_motor_id(motor_id_ & 0x7F, 0x01);
}

void FXMotorDriver::set_motor_zero_fx() {
    //When this command is sent, the motor immediately records its current position as the mechanical zero point and automatically saves it to non-volatile memory.
    if (comm_type_ == CommType::CAN) {
        {
            can_frame tx_frame{};
            tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(FX_MSG_SET_ZERO) << 24) | static_cast<uint32_t>(device_id_);
            tx_frame.can_dlc = 0x01;
            tx_frame.data[0] = 0x01;

            can_->transmit(tx_frame);
        }
        {
            response_count_++;
        }
        Timer::sleep_for(normal_sleep_time);
    }
}

void FXMotorDriver::clear_motor_error_fx() {
    if (comm_type_ == CommType::CAN) {
        can_frame tx_frame{};
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(FX_MSG_STOP) << 24) | static_cast<uint32_t>(device_id_);
        tx_frame.can_dlc = 0x01;
        tx_frame.data[0] = 0x01;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

//The parameter table contains both 4-byte types (float/int32_t) and 1-byte types (uint8_t), and they must be distinguished.
void FXMotorDriver::write_param_float_fx(uint16_t index, float value) {
    union32_t rv_type_convert;
    rv_type_convert.f = value;

    if (comm_type_ == CommType::CAN) {
        can_frame tx_frame{};
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(FX_MSG_WRITE_PARAM) << 24) | static_cast<uint32_t>(device_id_);
        tx_frame.can_dlc = 0x08;
        tx_frame.data[0] = index & 0xFF;
        tx_frame.data[1] = (index >> 8) & 0xFF;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = 0x00;
        tx_frame.data[4] = rv_type_convert.buf[0];
        tx_frame.data[5] = rv_type_convert.buf[1];
        tx_frame.data[6] = rv_type_convert.buf[2];
        tx_frame.data[7] = rv_type_convert.buf[3];

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void FXMotorDriver::write_param_uint8_fx(uint16_t index, uint8_t value) {
    if (comm_type_ == CommType::CAN) {
        can_frame tx_frame{};
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(FX_MSG_WRITE_PARAM) << 24) | static_cast<uint32_t>(device_id_);
        tx_frame.can_dlc = 0x08;
        tx_frame.data[0] = index & 0xFF;
        tx_frame.data[1] = (index >> 8) & 0xFF;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = 0x00;
        tx_frame.data[4] = value;
        tx_frame.data[5] = 0x00;
        tx_frame.data[6] = 0x00;
        tx_frame.data[7] = 0x00;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void FXMotorDriver::write_register_fx(uint16_t index, int32_t value) {
    if (comm_type_ == CommType::CAN) {
        can_frame tx_frame{};
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(FX_MSG_WRITE_PARAM) << 24) | static_cast<uint32_t>(device_id_);
        tx_frame.can_dlc = 0x08;
        tx_frame.data[0] = index & 0xFF;          
        tx_frame.data[1] = (index >> 8) & 0xFF;   
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = 0x00;
        uint8_t* b = (uint8_t*)&value;
        tx_frame.data[4] = b[0];
        tx_frame.data[5] = b[1];
        tx_frame.data[6] = b[2];
        tx_frame.data[7] = b[3];

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void FXMotorDriver::save_register_fx() {
    write_motor_flash();
}

void FXMotorDriver::refresh_motor_status() {
    if (comm_type_ == CommType::CAN) {
        can_frame tx_frame{};
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(FX_MSG_GET_ID) << 24) | static_cast<uint32_t>(device_id_);
        tx_frame.can_dlc = 0x00;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void FXMotorDriver::clear_motor_error() {
    if (comm_type_ == CommType::CAN) {
        can_frame tx_frame{};
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(FX_MSG_STOP) << 24) | static_cast<uint32_t>(device_id_);
        tx_frame.can_dlc = 0x01;
        tx_frame.data[0] = 0x01;

        can_->transmit(tx_frame);
        {
            response_count_++;
        }
    }
}
