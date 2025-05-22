#include "deskmate_movements.h"
#include <algorithm>
#include "oscillator.h"
#include "config.h"

static const char *TAG = "DeskmateMovements";

#define HEAD_HOME_POSITON 90
#define BODY_HOME_POSITON 90
#define HANDS_HOME_POSITON 90

Deskmate::Deskmate() {
    is_deskmate_reseting_ = false;
    // 初始化舵机 为-1 都未连接
    for (int i = 0; i < SERVO_COUNT; i++) {
        servo_pins_[i] = -1;
        servo_trim_[i] = 0;
    }
}

Deskmate::~Deskmate() {
    DetachServos();
}

unsigned long IRAM_ATTR millis() {
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

void Deskmate::Init() {
    servo_pins_[HEAD_PITCH] = HEAD_PITCH_PIN;
    servo_pins_[BODY_SWING] = BODY_SWING_PIN;
    servo_pins_[LEFT_ARM_ROLL] = LEFT_ARM_ROLL_PIN;
    servo_pins_[LEFT_ARM_PITCH] = LEFT_ARM_PITCH_PIN;
    servo_pins_[RIGHT_ARM_ROLL] = RIGHT_ARM_ROLL_PIN;
    servo_pins_[RIGHT_ARM_PITCH] = RIGHT_ARM_PITCH_PIN;

    AttachServos();
    is_deskmate_reseting_ = false;
}

void Deskmate::AttachServos() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].Attach(servo_pins_[i]);
            ESP_LOGI(TAG, "Servo %d attached to pin %d", i, servo_pins_[i]);
        }
    }
}

void Deskmate::DetachServos() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].Detach();
            // ESP_LOGI(TAG, "Servo %d detached from pin %d", i, servo_pins_[i]);
        }
    }
}

// OSCILLATORS TRIMS ------------------------------------------//

void Deskmate::SetTrims(int left_arm_roll_trim, int left_arm_pitch_trim, int right_arm_roll_trim, int right_arm_pitch_trim, int head_pitch_trim, int body_swing_trim) {
    servo_trim_[LEFT_ARM_ROLL] = left_arm_roll_trim;
    servo_trim_[LEFT_ARM_PITCH] = left_arm_pitch_trim;
    servo_trim_[RIGHT_ARM_ROLL] = right_arm_roll_trim;
    servo_trim_[RIGHT_ARM_PITCH] = right_arm_pitch_trim;
    servo_trim_[HEAD_PITCH] = head_pitch_trim;
    servo_trim_[BODY_SWING] = body_swing_trim;
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].SetTrim(servo_trim_[i]);
        }
    }
}

// 基本运动函数
void Deskmate::MoveServos(int time, int servo_target[]) {
    if (GetRestState() == true) {
        SetRestState(false);
    }

    final_time_ = millis() + time;
    if(time > 10){
        for (int i = 0; i < SERVO_COUNT; i++)
        {
            if(servo_pins_[i] != -1) {
                increment_[i] = (servo_target[i] - servo_[i].GetPosition()) / (time / 10);
            }
        }

        for (int iteration = 1; millis() < final_time_; iteration++)
        {
            partial_time_ = millis() + 10;
            for (size_t i = 0; i < SERVO_COUNT; i++)
            {
                if(servo_pins_[i] != -1) {
                    servo_[i].SetPosition(servo_[i].GetPosition() + increment_[i]);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    } else {
        for (int i = 0; i < SERVO_COUNT; i++)
        {
            if(servo_pins_[i] != -1) {
                servo_[i].SetPosition(servo_target[i]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(time));
    }

    // final adjustment to the target position
    // 最后微调，确保所有舵机都到达目标位置。
    // 该循环用于修正由于浮点运算误差或步进丢失导致的微小偏差。
    // 可以进一步优化为只调整未到位的舵机，减少不必要的操作。
    for (int retry = 0; retry < 3; ++retry) { // 尝试次数减少为3次，提高效率
        bool all_reached = true;
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1 && servo_target[i] != servo_[i].GetPosition()) {
                servo_[i].SetPosition(servo_target[i]);
                all_reached = false;
            }
        }
        if (all_reached) break;
        vTaskDelay(pdMS_TO_TICKS(3)); // 更短的延迟，加快收敛速度
    }
  
}

/**
 * @brief 控制单个舵机移动到指定位置。
 *
 * 本函数将指定编号的舵机移动到给定角度。角度范围被限制在[0, 180]，超出范围时默认设置为90度。
 * 如果设备处于休眠状态，会先唤醒再移动舵机。
 *
 * @param position 目标舵机角度（0-180度）。
 * @param servo_index 要移动的舵机索引。
 */
void Deskmate::MoveSingle(int position, int servo_index) {
    if(position > 180) {
        position = 90;
    }
    if (position < 0) {
        position = 90;
    }
    if (GetRestState() == true) {
        SetRestState(false);
    }

    if (servo_index >= 0 && servo_index < SERVO_COUNT && servo_pins_[servo_index] != -1) {
        servo_[servo_index].SetPosition(position);
    }
}

void Deskmate::OscillateServos(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period, double phase_diff[SERVO_COUNT], float cycle = 1) {
    if (GetRestState() == true) {
        SetRestState(false);
    }
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].SetA(amplitude[i]);
            servo_[i].SetO(offset[i]);
            servo_[i].SetPh(phase_diff[i]);
            servo_[i].SetT(period);
        }
    }

    double ref = millis();
    double end_time = period * cycle + ref;

    while (millis() < end_time) {
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1) {
                servo_[i].Refresh();
            }
        }
        vTaskDelay(5);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}

void Deskmate::Execute(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period, double phase_diff[SERVO_COUNT], float steps = 1.0) {
    if(GetRestState() == true) {
        SetRestState(false);
    }

    int cycles = int(steps);

    // 执行全部周期 execute all cycles
    if (cycles >= 1) {
        for (int i = 0; i < cycles; i++) {
            OscillateServos(amplitude, offset, period, phase_diff);
        }
    }

    // 执行剩余周期 execute remaining cycles
    OscillateServos(amplitude, offset, period, phase_diff, (float)steps - cycles);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void Deskmate::Home(bool arms_down) {
    if(is_deskmate_reseting_ == false) {
        // 准备所有舵机的初始位置
        int homes[SERVO_COUNT];
        for(int i = 0; i < SERVO_COUNT; i++) {
            homes[i] = 90;
        }
        MoveServos(500, homes);
        is_deskmate_reseting_ = true;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}

void Deskmate::SetRestState(bool state) {
    is_deskmate_reseting_ = state;
}

bool Deskmate::GetRestState() {
    return is_deskmate_reseting_;
}

// 预定义动作

// 模拟行走：双臂摆动的同时身体也相应扭转
// 实现思路
// 双臂交替摆动：通过设置左右手臂的 roll 舵机（LEFT_ARM_ROLL、RIGHT_ARM_ROLL）以相反的相位进行周期性摆动，模拟行走时手臂的前后摆动。
// 身体扭转：身体的 swing 舵机（BODY_SWING）也做周期性左右摆动，配合手臂动作，增强“行走”感。
// 参数可调：通过参数 steps 控制动作周期数，period 控制每个周期的时长，dir 控制方向（正负），arm_amount 控制手臂摆动幅度。
void Deskmate::Walk(float steps, int period, int dir, int arm_amount, int body_amount) {
    // 舵机索引
    // #define LEFT_ARM_ROLL   0
    // #define LEFT_ARM_PITCH  1
    // #define RIGHT_ARM_ROLL  2
    // #define RIGHT_ARM_PITCH 3
    // #define HEAD_PITCH      4
    // #define BODY_SWING      5
    // #define SERVO_COUNT 6
    if (arm_amount <= 0){
        return;
    }
    // 行走振荡器参数
    int A[SERVO_COUNT] = {arm_amount, 0, arm_amount, 0, 0, body_amount};    // 振幅
    int O[SERVO_COUNT] = {0, 0, 0, 0, 0, 0};    // 偏移量
    // double phase_diff[SERVO_COUNT] = {0, 0, DEG2RAD(dir * -90), DEG2RAD(dir * -90), 0, 0}; // 相位差
    double phase_diff[SERVO_COUNT] = {0, 0, M_PI, 0, 0, 0};
    // 支持方向切换
    if (dir < 0) {
        phase_diff[0] = M_PI;
        phase_diff[2] = 0;
    }

    Execute(A, O, period, phase_diff, steps); // 执行行走动作
}

// 点头动作：只让头部 pitch 舵机周期性摆动，其余舵机不动
// Nod(2, 800, 30, true);  // 点头2次，周期800ms，幅度30，双臂配合
// Nod(2, 800, 30, false); // 点头2次，周期800ms，幅度30，双臂不动
void Deskmate::HeadNod(float steps, int period, int head_amount, bool with_arms) {
    if (head_amount <= 0) {
        return;
    }

    int A[SERVO_COUNT] = {0, 0, 0, 0, head_amount, 0}; // 默认只让头部 pitch 摆动
    double phase_diff[SERVO_COUNT] = {0, 0, 0, 0, 0, 0};

    if (with_arms) {
        // 让双臂roll自然配合点头动作，幅度和头部一致，相位相反
        A[LEFT_ARM_ROLL] = head_amount / 2;
        A[RIGHT_ARM_ROLL] = head_amount / 2;
        phase_diff[RIGHT_ARM_ROLL] = M_PI;
    }

    int O[SERVO_COUNT] = {0, 0, 0, 0, 0, 0};

    Execute(A, O, period, phase_diff, steps); // 执行动作
}

// 摇头动作：通过身体左右摆动和手臂动作来表达否定
// 动作说明：

// 身体摆动：

// 通过 BODY_SWING 舵机实现左右摆动作为主要动作
// 振幅由参数 amount 控制
// 手臂配合：

// 双臂的 roll 和 pitch 都参与运动，使动作更自然
// roll 左右反相，增强左右摆动感
// pitch 与 roll 相差 90 度，产生一定的圆弧感
// 手臂振幅设为身体振幅的一半，避免动作过大
// 头部固定：

// HEAD_PITCH 保持不动（振幅为0）
// 让观察者更容易注意到身体的左右摆动
// Shake(2, 1000, 30);  // 摇头2次，周期1000ms，幅度30
void Deskmate::Negative(float steps, int period, int amount) {
    if (amount <= 0) {
        return;
    }

    // 身体摆动幅度
    int body_amount = amount;
    // 手臂摆动幅度（相对身体小一些更自然）
    int arm_amount = amount / 2;

    // 振幅设置
    int A[SERVO_COUNT] = {
        arm_amount,  // LEFT_ARM_ROLL：左臂roll小幅度配合
        arm_amount,  // LEFT_ARM_PITCH：左臂pitch配合
        arm_amount,  // RIGHT_ARM_ROLL：右臂roll小幅度配合
        arm_amount,  // RIGHT_ARM_PITCH：右臂pitch配合
        0,          // HEAD_PITCH：头部不动
        body_amount // BODY_SWING：身体左右摆动
    };

    // 偏移量
    int O[SERVO_COUNT] = {0, 0, 0, 0, 0, 0};

    // 相位差设置
    double phase_diff[SERVO_COUNT] = {
        0,      // 左臂roll与身体同相
        M_PI/2, // 左臂pitch延迟90度
        M_PI,   // 右臂roll与左臂反相
        M_PI/2, // 右臂pitch延迟90度
        0,      // 头部不动
        0       // 身体基准相位
    };

    Execute(A, O, period, phase_diff, steps);
}

// 单臂握手 通常是左臂

// hug 拥抱

// 困惑

// 抖动

// HandsUp举手

// 投降

// Electronbot

void Deskmate::EnableServoLimit(int diff_limit) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].SetLimiter(diff_limit);
        }
    }
}

void Deskmate::DisableServoLimit() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].DisableLimiter();
        }
    }
}

