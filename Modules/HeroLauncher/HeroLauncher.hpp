#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - motor_fric_front_left: '@&motor_fric_front_left'
  - motor_fric_front_right: '@&motor_fric_front_right'
  - motor_fric_back_left: '@&motor_fric_back_left'
  - motor_fric_back_right: '@&motor_fric_back_right'
  - motor_trig_0: '@&motor_fric_0'
  - task_stack_depth: 2048
  - trig_speed_pid:
      k: 1.0
      p: 0.0012
      i: 0.001
      d: 0.0
      i_limit: 1.0
      out_limit: 1.0
      cycle: false
  - trig_angle_pid:
      k: 1.0
      p: 4000.0
      i: 3000.0
      d: 0.0
      i_limit: 0.0
      out_limit: 2000.0
      cycle: false
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
  - launcher_param:
      fric1_setpoint_speed: 3900.0
      fric2_setpoint_speed: 4300.0
      trig_gear_ratio: 19.0
      num_trig_tooth: 6.0
template_args:
  - MotorType: RMMotorContainer
required_hardware:
  - dr16
  - can
depends:
  - qdu-feature/CMD
  - qdu-feature/RMMotor
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cstdint>

#include "CMD.hpp"
#include "Launcher.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "pwm.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "uart.hpp"


enum class LauncherEvent : uint8_t {
  SET_MODE_RELAX,
  SET_MODE_START,
};

enum class TRIGMODE : uint8_t {
  SAFE = 0,
  SINGLE,
  CONTINUE,
  AIM,
};

enum class FRICMODE : uint8_t {
  SAFE = 0,
  READY,
};

typedef struct {
  float heat_limit;
  float heat_cooling;
  uint8_t level;
} RefereeData;

struct HeatControl {
  float heat;          /* 现在热量水平 */
  float last_heat;     /* 之前的热量水平 */
  float heat_limit;    /* 热量上限 */
  float speed_limit;   /* 弹丸初速是上限 */
  float cooling_rate;  /* 冷却速率 */
  float heat_increase; /* 每发热量增加值 */

  uint32_t available_shot; /* 热量范围内还可以发射的数量 */
};

  struct LauncherParam {
    /*一级摩擦轮转速*/
    float fric1_setpoint_speed;
    /*二级摩擦轮转速*/
    float fric2_setpoint_speed;
    /*拨弹盘电机减速比*/
    float trig_gear_ratio;
    /*拨齿数目*/
    float num_trig_tooth;
  };

template <typename MotorType>
class HeroLauncher : public LibXR::Application {
 public:
  HeroLauncher(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
               RMMotor *motor_fric_front_left_,
               RMMotor *motor_fric_front_right_,
               RMMotor *motor_fric_back_left_,
               RMMotor *motor_fric_back_right_,
               RMMotor *motor_trig_0,
               uint32_t task_stack_depth,
               LibXR::PID<float>::Param trig_speed_pid,
               LibXR::PID<float>::Param trig_angle_pid,
               LibXR::PID<float>::Param fric_speed_pid_0,
               LibXR::PID<float>::Param fric_speed_pid_1,
               LibXR::PID<float>::Param fric_speed_pid_2,
               LibXR::PID<float>::Param fric_speed_pid_3,
               LauncherParam launcher_param)
      : PARAM(launcher_param),
        motor_fric_front_left_(motor_fric_front_left_),
        motor_fric_front_right_(motor_fric_front_right_),
        motor_fric_back_left_(motor_fric_back_left_),
        motor_fric_back_right_(motor_fric_back_right_),
        motor_trig_0_(motor_trig_0),
        trig_speed_pid_(trig_speed_pid),
        trig_angle_pid_(trig_angle_pid),
        fric_speed_pid_{fric_speed_pid_0, fric_speed_pid_1, fric_speed_pid_2,
                        fric_speed_pid_3} {
    // Hardware initialization example:
    // auto dev = hw.template Find<LibXR::GPIO>("led");
    buzzer_ = hw.Find<LibXR::PWM>({"pwm_buzzer"});

    thread_.Create(this, ThreadFunction, "HeroLauncherThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);

    auto cb = [](bool in_isr, void *arg, uint32_t event_id) {
      UNUSED(in_isr);
      static_cast<HeroLauncher *>(arg)->EventHandler(event_id);
    };

    using GenericCallbackPtr = void (*)(bool, void *, uint32_t);
    using TargetCallbackPtr = void (*)(bool, HeroLauncher *, uint32_t);

    GenericCallbackPtr generic_ptr = cb;

    auto specific_ptr = reinterpret_cast<TargetCallbackPtr>(generic_ptr);

    auto callback = LibXR::Callback<uint32_t>::Create(specific_ptr, this);
    // launcher_event_.Register(
    //     static_cast<uint32_t>(LauncherEvent::SET_MODE_RELAX), callback);
    // launcher_event_.Register(
    //     static_cast<uint32_t>(LauncherEvent::SET_MODE_START), callback);
    launcher_event_.Register(
      static_cast<uint32_t>(FRICMODE::SAFE), callback);
    launcher_event_.Register(
      static_cast<uint32_t>(FRICMODE::READY), callback);
    launcher_event_.Register(
      static_cast<uint32_t>(TRIGMODE::SAFE), callback);
    launcher_event_.Register(
      static_cast<uint32_t>(TRIGMODE::SINGLE), callback);


  }

  static void ThreadFunction(HeroLauncher *HeroLauncher) {
    LibXR::Topic::ASyncSubscriber<CMD::LauncherCMD> launcher_cmd_tp(
      "launcher_cmd");
    launcher_cmd_tp.StartWaiting();
    while (true) {
      launcher_cmd_tp.StartWaiting();
      if (launcher_cmd_tp.Available()) {
        HeroLauncher->launcher_cmd_ = launcher_cmd_tp.GetData();
        launcher_cmd_tp.StartWaiting();
      }
      HeroLauncher->Update();
      HeroLauncher->HeatLimit();
      HeroLauncher->Control();

      auto last_time = LibXR::Timebase::GetMilliseconds();
      HeroLauncher->thread_.SleepUntil(last_time, 2);
    }
  }

  void Update() {
    auto now = LibXR::Timebase::GetMicroseconds();
    dt_ = (now - last_online_time_).ToSecondf();
    //hook = &dt_;
    last_online_time_ = now;

    const float LAST_TRIG_MOTOR_ANGLE =
        LibXR::CycleValue<float>(this->motor_trig_0_->GetAngle());
    motor_fric_front_left_->Update();
    motor_fric_front_right_->Update();
    motor_fric_back_left_->Update();
    motor_fric_back_right_->Update();
    motor_trig_0_->Update();
    const float DELTA_MOTOR_ANGLE =
        LibXR::CycleValue<float>(this->motor_trig_0_->GetAngle()) -
        LAST_TRIG_MOTOR_ANGLE;
    this->trig_angle_ += DELTA_MOTOR_ANGLE / PARAM.trig_gear_ratio;
    //thread_.Sleep(2.0f);

  }

  void Control() {

    current_back_left_ = motor_fric_front_left_->GetCurrent();

    fric_target_speed_[0] = PARAM.fric2_setpoint_speed;
    fric_target_speed_[1] = PARAM.fric2_setpoint_speed;
    fric_target_speed_[2] = PARAM.fric1_setpoint_speed;
    fric_target_speed_[3] = PARAM.fric1_setpoint_speed;

    if(reset_){
      first_loading_ = true;
      reset_ = false;}

      if (first_loading_) {
        if(start_loading_time_ == 0){start_loading_time_ = LibXR::Timebase::GetMilliseconds();}
        if (fire_flag_){
            trig_setpoint_angle_ -= M_2PI / 1002.0f;
            last_change_angle_time_ = LibXR::Timebase::GetMilliseconds();

        }

        if (LibXR::Timebase::GetMilliseconds() - start_loading_time_ > 500) {
          if(std::abs(motor_fric_back_left_->GetCurrent()) > 5) {  //发弹检测
          trig_zero_angle_= trig_angle_ ; //获取电机当前位置
          trig_setpoint_angle_ = trig_angle_ - M_2PI / 7.55f; //偏移量

          fire_flag_ = false ;
          first_loading_ = false;

          current_exti_++;
          fired_ ++;
        }
      }

    }
    else{

      if(fire_flag_){
        if(!enable_fire_){

          if (buzzer_ != nullptr) { //
            buzzer_->SetConfig({1500});  // 设置频率为1500Hz
            buzzer_->Enable(); //
            buzzer_->SetDutyCycle(0.5);  // 设置占空比为50%

            }
            if(heat_ctrl_.available_shot){

            buzzer_->Disable(); //

            trig_setpoint_angle_ -= M_2PI / 6.0f ;

            enable_fire_ = true;
            mark_launch_ = false;
            start_fire_time_ = LibXR::Timebase::GetMilliseconds();
            }
          }
      }
      if(!mark_launch_)
      {
        if(std::abs(motor_fric_back_left_->GetCurrent()) > 5) {//暂时作为遥控器使用
        fire_flag_ = false ;
        fired_++;
        mark_launch_ = true;
        enable_fire_ = false;
        finish_fire_time_ = LibXR::Timebase::GetMilliseconds();
        }
      }
    }
    real_launch_delay_ = (finish_fire_time_ - start_fire_time_ ).ToSecondf();

      // if (LibXR::Timebase::GetMilliseconds() - last_change_angle_time_ >
      //     10000) {
      //   trig_setpoint_angle_ -= M_2PI / 6.0f;
      //   last_change_angle_time_ = LibXR::Timebase::GetMilliseconds();

      fric_output_[0] = fric_speed_pid_[0].Calculate(
          fric_target_speed_[0] , motor_fric_front_left_->GetRPM(), dt_);
      fric_output_[1] = fric_speed_pid_[1].Calculate(
          fric_target_speed_[1] , motor_fric_front_right_->GetRPM(), dt_);
      fric_output_[2] = fric_speed_pid_[2].Calculate(
          fric_target_speed_[2], motor_fric_back_left_->GetRPM(), dt_);
      fric_output_[3] = fric_speed_pid_[3].Calculate(
          fric_target_speed_[3] , motor_fric_back_right_->GetRPM() , dt_);

      motor_fric_front_left_->CurrentControl( fric_output_[0] );
      motor_fric_front_right_->CurrentControl( fric_output_[1] );
      motor_fric_back_left_->CurrentControl( fric_output_[2]  );
      motor_fric_back_right_->CurrentControl( fric_output_[3]  );

    trig_setpoint_speed_ = trig_angle_pid_.Calculate(trig_setpoint_angle_,
                                                  trig_angle_, dt_);
    //trig_setpoint_speed_ =  LibXR::Timebase::GetMilliseconds() % 1000 > 600 ? -3000 : 0;
    trig_output_ = trig_speed_pid_.Calculate(trig_setpoint_speed_,
                                             motor_trig_0_->GetRPM(), dt_);

    motor_trig_0_->CurrentControl(trig_output_);
  }
  void HeatLimit() {
    referee_data_.level = 1 ; //for debug
    heat_ctrl_.heat_limit = 140 + (referee_data_.level  - 1 ) * 10;
    heat_ctrl_.cooling_rate = 12 + (referee_data_.level  - 1 ) * 2;
    heat_ctrl_.heat -=heat_ctrl_.cooling_rate / 500.0 ; //每个控制周期的冷却恢复
    if(fired_ > 0){
      heat_ctrl_.heat += heat_ctrl_.heat_increase * fired_ ;
      fired_ = 0;
    }
    heat_ctrl_.heat = std::clamp(heat_ctrl_.heat, 0.0f, heat_ctrl_.heat_limit);
    heat_ctrl_.available_shot = floorf((this->heat_ctrl_.heat_limit - this->heat_ctrl_.heat )/
                                        this->heat_ctrl_.heat_increase);

  }

  LibXR::Event &GetEvent() { return launcher_event_; }

  void SetMode(uint32_t mode) {
    launcher_event_.Active(mode);
  }
  void EventHandler(uint32_t event_id) {
    SetMode(static_cast<uint32_t>(static_cast<LauncherEvent>(event_id)));
  }


  void OnMonitor() override {}

 private:

  const LauncherParam PARAM;
  RefereeData referee_data_;

  HeatControl heat_ctrl_;

  bool first_loading_ = true;

  float dt_ = 0.0f;

  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  LibXR::MillisecondTimestamp last_change_angle_time_ = 0;

  LibXR::MillisecondTimestamp start_loading_time_ = 0;

  RMMotor *motor_fric_front_left_;
  RMMotor *motor_fric_front_right_;
  RMMotor *motor_fric_back_left_;
  RMMotor *motor_fric_back_right_;


  RMMotor *motor_trig_0_;

  LibXR::PWM *buzzer_ ; //



  float trig_setpoint_angle_ = 0.0f;
  float trig_setpoint_speed_ = 0.0f;

  float trig_zero_angle_ = 0.0f;
  float trig_angle_ = 0.0f;
  float trig_output_ = 0.0f;

  float fric_target_speed_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  float fric_output_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  LibXR::PID<float> trig_speed_pid_;
  LibXR::PID<float> trig_angle_pid_;
  LibXR::PID<float> fric_speed_pid_[4];

  CMD::LauncherCMD launcher_cmd_;

  float current_back_left_ = 0.0f;

  bool fire_flag_ = false;

  uint8_t fired_ = 0;

  bool reset_ = false;

  uint8_t current_exti_ = 0;

  bool enable_fire_ = false;
  bool mark_launch_ = false;

  LibXR::MillisecondTimestamp start_fire_time_ = 0;
  LibXR::MillisecondTimestamp finish_fire_time_ = 0;
  float real_launch_delay_ = 0.0f;



  LibXR::Thread thread_;
  LibXR::Semaphore semaphore_;
  LibXR::Mutex mutex_;
  LibXR::Event launcher_event_;
};
