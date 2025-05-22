#ifndef __DESKMATE_MOVEMENTS_H__
#define __DESKMATE_MOVEMENTS_H__

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "oscillator.h"

// 定义运动类型
#define FORWARD 1
#define BACKWARD -1
#define LEFT 1
#define RIGHT -1
#define UP 1
#define DOWN -1
#define BOTH 0
#define SMALL 5
#define MEDIUM 15
#define BIG 30


// 舵机（Servo）默认的角度变化速率限制，单位为度/秒（degree / sec）?
#define SERVO_LIMIT_DEFAULT 240

// 舵机索引
#define LEFT_ARM_ROLL   0
#define LEFT_ARM_PITCH  1
#define RIGHT_ARM_ROLL  2
#define RIGHT_ARM_PITCH 3
#define HEAD_PITCH      4
#define BODY_SWING      5
#define SERVO_COUNT 6

class Deskmate {
public:
    Deskmate();
    ~Deskmate();

    void Init();
    void AttachServos();
    void DetachServos();

    // 默认手部的trim为0
    void SetTrims(int left_arm_roll_trim, int left_arm_pitch_trim, int right_arm_roll_trim, int right_arm_pitch_trim, int head_pitch_trim, int body_swing_trim);
    void MoveServos(int time, int servo_target[]);
    void OscillateServos(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period, double phase_diff[SERVO_COUNT], float cycle);

    // 舵机复位
    void Home(bool arms_down = true);
    bool GetRestState();
    void SetRestState(bool state);
    void MoveSingle(int position, int servo_index);

    // 预设动作
    void Wave(float steps = 1, int period = 2000);
    // void Walk(float steps = 4, int period = 2000, int dir = FORWARD, int amount = 0);
    void Walk(float steps = 4, int period = 2000, int dir= FORWARD, int arm_amount = 30, int body_amount = 30);

    // 身体
    void Swing(float steps = 1, int period = 1000);

    // 头
    void HeadUp(int period = 1000, int dir = UP);
    // void HeadNod(int period = 1000, int count = 1);
    void HeadNod(float steps, int period, int head_amount, bool with_arms);
    void Negative(float steps, int period, int amount);

    void HandsUp(int period = 1000, int dir = 0);
    void HandsDown(int period = 1000, int dir = 0);
    void HandWave(int period = 1000, int dir = LEFT);
    void ShakeBoth(int period = 1000);

    // 舵机限位
    void EnableServoLimit(int speed_limit_degree_per_sec = SERVO_LIMIT_DEFAULT);
    void DisableServoLimit();


private:
    // 舵机相关数组，包含：振荡器、引脚、偏移量、增量
    Oscillator servo_[SERVO_COUNT];

    int servo_pins_[SERVO_COUNT];
    int servo_trim_[SERVO_COUNT];
    float increment_[SERVO_COUNT];

    unsigned long final_time_;
    unsigned long partial_time_;

    bool is_deskmate_reseting_;
    
    void Execute(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period, double phase_diff[SERVO_COUNT], float steps);
};

#endif