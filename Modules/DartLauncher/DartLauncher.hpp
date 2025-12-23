#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - motor_fric_front_left: '@&motor_fric_front_left'
  - motor_fric_front_right: '@&motor_fric_front_right'
  - motor_fric_back_left: '@&motor_fric_back_left'
  - motor_fric_back_right: '@&motor_fric_back_right'
  - task_stack_depth: 2048
  - fric1_setpoint_speed: 6300.0
  - fric2_setpoint_speed: 4200.0
  - fric_speed_pid_0:
      k: 1.0
      p: 0.002
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  - fric_speed_pid_1:
      k: 1.0
      p: 0.002
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  - fric_speed_pid_2:
      k: 1.0
      p: 0.002
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  - fric_speed_pid_3:
      k: 1.0
      p: 0.002
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
template_args:
  - MotorType: RMMotor
required_hardware:
  - dr16
  - can
depends:
  - qdu-feature/CMD
  - qdu_feature/RMMotor
=== END MANIFEST === */
// clang-format on

#include <cstdint>

#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "gpio.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "uart.hpp"

// static float* hook;

enum class DartEvent : uint8_t {
  SET_MODE_FRIC_START,
  SET_MODE_FRIC_STOP,
};

enum class DartMode : uint8_t {
  FRIC_START,
  FRIC_STOP,
};

template <typename MotorType>
class DartLauncher : public LibXR::Application {
 public:
  DartLauncher(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
               RMMotor *motor_fric_front_left_,
               RMMotor *motor_fric_front_right_, RMMotor *motor_fric_back_left_,
               RMMotor *motor_fric_back_right_, uint32_t task_stack_depth,
               float fric1_setpoint_speed, float fric2_setpoint_speed,
               LibXR::PID<float>::Param fric_speed_pid_0,
               LibXR::PID<float>::Param fric_speed_pid_1,
               LibXR::PID<float>::Param fric_speed_pid_2,
               LibXR::PID<float>::Param fric_speed_pid_3)
      : user_key_(hw.Find<LibXR::GPIO>("USER_KEY")),
        motor_fric_front_left_(motor_fric_front_left_),
        motor_fric_front_right_(motor_fric_front_right_),
        motor_fric_back_left_(motor_fric_back_left_),
        motor_fric_back_right_(motor_fric_back_right_),
        fric1_setpoint_speed_(fric1_setpoint_speed),
        fric2_setpoint_speed_(fric2_setpoint_speed),
        fric_speed_pid_{fric_speed_pid_0, fric_speed_pid_1, fric_speed_pid_2,
                        fric_speed_pid_3} {
    // Hardware initialization example:
    // auto dev = hw.template Find<LibXR::GPIO>("led");
    thread_.Create(this, ThreadFunction, "dartLauncherThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);
    // 设置按键中断
    if (user_key_ != nullptr) {
      // 注册按键回调函数
      auto user_key_callback = LibXR::GPIO::Callback::Create(
          [](bool in_isr, DartLauncher *self) {
            // if (!in_isr) return;

            // 切换模式
            self->mode_ = (self->mode_ == DartMode::FRIC_START)
                              ? DartMode::FRIC_STOP
                              : DartMode::FRIC_START;
          },
          this);

      user_key_->RegisterCallback(user_key_callback);

      // 配置GPIO为下降沿中断模式
      LibXR::GPIO::Configuration config;
      config.direction = LibXR::GPIO::Direction::FALL_INTERRUPT;
      config.pull = LibXR::GPIO::Pull::UP;
      user_key_->SetConfig(config);
      user_key_->EnableInterrupt();
    }

    auto cb = [](bool in_isr, void *arg, uint32_t event_id) {
      UNUSED(in_isr);
      static_cast<DartLauncher *>(arg)->EventHandler(event_id);
    };

    using GenericCallbackPtr = void (*)(bool, void *, uint32_t);
    using TargetCallbackPtr = void (*)(bool, DartLauncher *, uint32_t);

    GenericCallbackPtr generic_ptr = cb;

    auto specific_ptr = reinterpret_cast<TargetCallbackPtr>(generic_ptr);

    auto callback = LibXR::Callback<uint32_t>::Create(specific_ptr, this);
    dart_event_.Register(static_cast<uint32_t>(DartEvent::SET_MODE_FRIC_START),
                         callback);
    dart_event_.Register(static_cast<uint32_t>(DartEvent::SET_MODE_FRIC_STOP),
                         callback);
  }

  static void ThreadFunction(DartLauncher *dartLauncher) {
    while (true) {
      dartLauncher->Update();
      dartLauncher->Control();
      auto last_time = LibXR::Timebase::GetMilliseconds();
      dartLauncher->thread_.SleepUntil(last_time, 2);
    }
  }

  void Update() {
    auto now = LibXR::Timebase::GetMicroseconds();
    dt_ = (now - last_online_time_).ToSecondf();
    // hook = &dt_;
    last_online_time_ = now;

    motor_fric_front_right_->Update();
    motor_fric_front_left_->Update();
    motor_fric_back_left_->Update();
    motor_fric_back_right_->Update();

    fl_minus_fr_ = std::abs(motor_fric_front_left_->GetRPM()) -
                   std::abs(motor_fric_front_right_->GetRPM());
    bl_minus_br_ = std::abs(motor_fric_back_left_->GetRPM()) -
                   std::abs(motor_fric_back_right_->GetRPM());
  }

  void Control() {
    switch (mode_) {
      case DartMode::FRIC_STOP:
        fric_target_speed_[0] = 0;
        fric_target_speed_[1] = 0;
        fric_target_speed_[2] = 0;
        fric_target_speed_[3] = 0;
        motor_fric_back_left_->Relax();
        motor_fric_back_right_->Relax();
        motor_fric_front_left_->Relax();
        motor_fric_front_right_->Relax();
        break;
      case DartMode::FRIC_START:
        fric_target_speed_[0] = fric2_setpoint_speed_;
        fric_target_speed_[1] = fric2_setpoint_speed_;
        fric_target_speed_[2] = fric1_setpoint_speed_;
        fric_target_speed_[3] = fric1_setpoint_speed_;
        break;
    }

    // fric_target_speed_[0] = -fric2_setpoint_speed_;
    // fric_target_speed_[1] = fric1_setpoint_speed_;
    // fric_target_speed_[2] = -fric1_setpoint_speed_;
    // fric_target_speed_[3] = fric2_setpoint_speed_;

    fric_output_[0] = fric_speed_pid_[0].Calculate(
        fric_target_speed_[0], motor_fric_front_left_->GetRPM(), dt_);
    fric_output_[1] = fric_speed_pid_[1].Calculate(
        fric_target_speed_[1], motor_fric_front_right_->GetRPM(), dt_);
    fric_output_[2] = fric_speed_pid_[2].Calculate(
        fric_target_speed_[2], motor_fric_back_left_->GetRPM(), dt_);
    fric_output_[3] = fric_speed_pid_[3].Calculate(
        fric_target_speed_[3], motor_fric_back_right_->GetRPM(), dt_);

    motor_fric_front_left_->CurrentControl(fric_output_[0]);
    motor_fric_front_right_->CurrentControl(fric_output_[1]);
    motor_fric_back_left_->CurrentControl(fric_output_[2]);
    motor_fric_back_right_->CurrentControl(fric_output_[3]);
  }
  void HeatLimit() {}

  LibXR::Event &GetEvent() { return dart_event_; }

  void SetMode(uint32_t mode) {
    dart_event_.Active(mode);
  }
  void EventHandler(uint32_t event_id) {
    SetMode(static_cast<uint32_t>(static_cast<DartEvent>(event_id)));
  }

  void OnMonitor() override {}

 private:
  float dt_ = 0.0f;

  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  LibXR::MillisecondTimestamp last_change_angle_time_ = 0;

  LibXR::MillisecondTimestamp start_loading_time_ = 0;

  LibXR::GPIO *user_key_;

  RMMotor *motor_fric_front_left_;
  RMMotor *motor_fric_front_right_;
  RMMotor *motor_fric_back_left_;
  RMMotor *motor_fric_back_right_;

  float fric1_setpoint_speed_ = 0.0f;
  float fric2_setpoint_speed_ = 0.0f;
  float fric_target_speed_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  float fric_output_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  LibXR::PID<float> fric_speed_pid_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  DartMode mode_ = DartMode::FRIC_STOP;

  float fl_minus_fr_ = 0.0f;
  float bl_minus_br_ = 0.0f;

  bool fire_ = false;

  LibXR::MillisecondTimestamp start_fire_time_ = 0;
  LibXR::MillisecondTimestamp finish_fire_time_ = 0;
  float real_launch_delay_ = 0.0f;

  LibXR::Event dart_event_;

  LibXR::Thread thread_;
  LibXR::Mutex mutex_;
};
