#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - translation_motor: "@&translation_motor"
  - rotation_motor: "@&rotation_motor"
  - task_stack_depth: 2048
  - translation_speed_pid:
      k: 1.0
      p: 0.001
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  - translation_angle_pid:
      k: 1.0
      p: 2000.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 2000.0
      cycle: false
  - rotation_speed_pid:
      k: 1.0
      p: 0.001
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
template_args: []
required_hardware:
 - can
depends:
 - qdu-future/RMMotor
=== END MANIFEST === */
// clang-format on
#include <algorithm>
#include <cstdint>
#include "RMMotor.hpp"
#include "DartLauncher.hpp"
#include "app_framework.hpp"
// #include "stm32f4xx.h"
#include "cycle_value.hpp"
#include "event.hpp"
#include "gpio.hpp"
#include "libxr_def.hpp"
#include "thread.hpp"
#include "timebase.hpp"

enum class TargetEvent : uint8_t {
    SET_MODE_STOP,
    SET_MODE_TRANSLATION_MOTOR_RUN_SLOW,
    SET_MODE_TRANSLATION_MOTOR_RUN_FAST,
    SET_MODE_ROTATION_MOTOR_RUN_SLOW,
    SET_MODE_ROTATION_MOTOR_RUN_FAST,
    SET_MODE_HYBIRD
};

enum class TargetMode : uint8_t {
    STOP,
    TRANSLATION_MOTOR_RUN_SLOW,
    TRANSLATION_MOTOR_RUN_FAST,
    ROTATION_MOTOR_RUN_SLOW,
    ROTATION_MOTOR_RUN_FAST,
    HYBIRD

};
class Target : public LibXR::Application {
public:
  Target(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
        RMMotor *translation_motor, RMMotor *rotation_motor,
        uint32_t task_stack_depth,
        LibXR::PID<float>::Param translation_speed_pid,
        LibXR::PID<float>::Param translation_angle_pid,
        LibXR::PID<float>::Param rotation_speed_pid)
        : user_key_(hw.Find<LibXR::GPIO>("USER_KEY")),
          translation_motor_(translation_motor),
          rotation_motor_(rotation_motor),
          translation_speed_pid_(translation_speed_pid),
          translation_angle_pid_(translation_angle_pid),
          rotation_speed_pid_(rotation_speed_pid)
          {
            UNUSED(app);
    // Hardware initialization example:
    // auto dev = hw.template Find<LibXR::GPIO>("led");
    thread_.Create(this, ThreadFunction, "TargetThread", task_stack_depth,
                    LibXR::Thread::Priority::MEDIUM);
    // // 设置按键中断
    // if (user_key_ != nullptr) {
    //   // 注册按键回调函数
    //   auto user_key_callback = LibXR::GPIO::Callback::Create(
    //       [](bool in_isr, Target *self) {
    //         UNUSED(in_isr);
    //         uint8_t next_mode = (static_cast<uint8_t>(self->mode_) + 1) % 6;
    //         self->mode_ = static_cast<TargetMode>(next_mode);
    //       },
    //       this);

    //   user_key_->RegisterCallback(user_key_callback);

      auto user_key_callback = LibXR::GPIO::Callback::Create(
          [](bool in_isr, Target *target) {
            UNUSED(in_isr);
            uint8_t next_mode = (static_cast<uint8_t>(target->mode_) + 1) % 6;
            target->SetMode(next_mode);
          },
          this);
      user_key_->RegisterCallback(user_key_callback);

      // 配置GPIO为下降沿中断模式
      LibXR::GPIO::Configuration config;
      config.direction = LibXR::GPIO::Direction::FALL_INTERRUPT;
      config.pull = LibXR::GPIO::Pull::UP;
      user_key_->SetConfig(config);
      user_key_->EnableInterrupt();
//    }

}
 static void ThreadFunction(Target *target) {
  while (true) {
    target->Update();
    target->Control();
    auto last_time = LibXR::Timebase::GetMilliseconds();
    target->thread_.SleepUntil(last_time, 2);
  }
}

  void Update() {
    auto now = LibXR::Timebase::GetMicroseconds();
    dt_ = (now - last_online_time_).ToSecondf();

    last_online_time_ = now;
    const float LAST_TRANSLATION_MOTOR_ANGLE =
      LibXR::CycleValue<float>(this->translation_motor_->GetAngle());
    rotation_motor_->Update();
    translation_motor_->Update();
    translation_motor_->Update();
    const float DELTA_TRANSLATION_MOTOR_ANGLE =
      LibXR::CycleValue<float>(this->translation_motor_->GetAngle()) -
      LAST_TRANSLATION_MOTOR_ANGLE;

   this->translation_motor_angle_ += DELTA_TRANSLATION_MOTOR_ANGLE / GEAR_RATIO;
  }

  void Control() {
    switch (mode_) {
      case TargetMode::STOP:
        translation_motor_setpoint_speed_ = 0.0f;
        translation_motor_setpoint_angle_ = 0.0f;
        rotation_motor_setpoint_speed_ = 0.0f;
        rotation_motor_output_ = 0.0f;
        translation_motor_output_ = 0.0f;
        break;
      case TargetMode::HYBIRD:
      rotation_motor_setpoint_speed_ = 390.0f;
      case TargetMode::TRANSLATION_MOTOR_RUN_SLOW:
      case TargetMode::TRANSLATION_MOTOR_RUN_FAST:
      if(start_right_init_){
        translation_motor_setpoint_angle_ += M_2PI / 500.0f;
        cnt_++;
        if(cnt_ > 600) {
          if(std::abs(translation_motor_->GetCurrent()) > 5)
          {
            max_translation_motor_angle_ = translation_motor_angle_;
            cnt_ = 0;
            start_left_init_ = true;
            start_right_init_ = false;
          }
        }

      }
      else if(start_left_init_){
        translation_motor_setpoint_angle_ -=M_2PI / 500.0f;
        cnt_++;
          if(cnt_ > 1000){
            if(std::abs(translation_motor_->GetCurrent()) > 5){
              min_translation_motor_angle_ = translation_motor_angle_;
              translation_motor_setpoint_angle_ = max_translation_motor_angle_;
              start_left_init_ = false;
            }

          }
      }
        else if(!start_left_init_ && !start_right_init_){
          if (translation_motor_angle_ <= min_translation_motor_angle_ + 1.25f ) {
            translation_motor_setpoint_angle_ = max_translation_motor_angle_;
            translation_angle_pid_.SetOutLimit(500.0f);
          }
          if (translation_motor_angle_ >= max_translation_motor_angle_ - 1.25f) {
           translation_motor_setpoint_angle_ = min_translation_motor_angle_;
           translation_angle_pid_.SetOutLimit(500.0f);
          }
          if(translation_motor_angle_ >= min_translation_motor_angle_ + 3.0f && translation_motor_angle_ <= max_translation_motor_angle_ - 3.0f)
          {
            translation_angle_pid_.SetOutLimit(2000.0f);
          }




    }
    break;

      case TargetMode::ROTATION_MOTOR_RUN_SLOW:
        rotation_motor_setpoint_speed_ = 390.0f;
        break;
      case TargetMode::ROTATION_MOTOR_RUN_FAST:
        rotation_motor_setpoint_speed_ = 1950.0f;
        break;
        default:
          break;
    }



    rotation_motor_output_ = rotation_speed_pid_.Calculate(
        rotation_motor_setpoint_speed_, rotation_motor_->GetRPM(), dt_);

    translation_motor_setpoint_speed_ = translation_angle_pid_.Calculate(
        translation_motor_setpoint_angle_, translation_motor_angle_, dt_);
        // switch (mode_) {
        //   case TargetMode::TRANSLATION_MOTOR_RUN_SLOW:
        //   case TargetMode::HYBIRD:
        //     translation_motor_setpoint_speed_ = std::min(translation_motor_setpoint_speed_, 500.0f);
        //     break;
        //   case TargetMode::TRANSLATION_MOTOR_RUN_FAST:
        //     translation_motor_setpoint_speed_ = std::min(translation_motor_setpoint_speed_, 3000.0f);
        //     break;
        //   default:
        //     break;
        // }

    translation_motor_output_ = translation_speed_pid_.Calculate(
        translation_motor_setpoint_speed_, translation_motor_->GetRPM(), dt_);

    rotation_motor_->CurrentControl(rotation_motor_output_ );

    translation_motor_->CurrentControl(translation_motor_output_ );
  }
  LibXR::Event &GetEvent() { return target_event_; }

  void SetMode(uint32_t mode) {
    mode_ = static_cast<TargetMode>(mode);
  if(mode_ == TargetMode::ROTATION_MOTOR_RUN_FAST || mode_ == TargetMode::ROTATION_MOTOR_RUN_SLOW){
    translation_motor_setpoint_angle_ = (min_translation_motor_angle_ + max_translation_motor_angle_) / 2.0f;}
  if(mode_ == TargetMode::TRANSLATION_MOTOR_RUN_FAST || mode_ == TargetMode::TRANSLATION_MOTOR_RUN_SLOW || mode_ == TargetMode::HYBIRD){
    translation_motor_setpoint_angle_ = max_translation_motor_angle_;
  }
    rotation_motor_setpoint_speed_ = 0.0f;
    rotation_motor_output_ = 0.0f;
    translation_motor_output_ = 0.0f;
  }
  void EventHandler(uint32_t event_id) {
    SetMode(static_cast<uint32_t>(static_cast<TargetEvent>(event_id)));
  }
  void OnMonitor() override {}

 private:
  float dt_ = 0.0f;

  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  LibXR::GPIO *user_key_;

  //bool first_moving_ = true;

  bool start_right_init_ = true;
  bool start_left_init_ = false;

  uint32_t cnt_ = 0;

  float min_translation_motor_angle_ = 0.0f;
  float max_translation_motor_angle_ = 0.0f;

  RMMotor *translation_motor_;
  RMMotor *rotation_motor_;

  float translation_motor_angle_ = 0.0f;

  const float GEAR_RATIO = 19.0f;
  LibXR::PID<float> translation_speed_pid_;
  LibXR::PID<float> translation_angle_pid_;
  LibXR::PID<float> rotation_speed_pid_;

  float rotation_motor_setpoint_speed_ = 0.0f;

  float translation_motor_setpoint_speed_ = 0.0f;
  float translation_motor_setpoint_angle_ = 0.0f;

  float rotation_motor_output_ = 0.0f;

  float translation_motor_output_ = 0.0f;

  TargetMode mode_ = TargetMode::STOP;

  LibXR::Event target_event_;

  LibXR::Thread thread_;
};
