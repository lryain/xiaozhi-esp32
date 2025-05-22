#include "oscillator.h"

#include <driver/ledc.h>
#include <esp_timer.h>

#include <algorithm>
#include <cmath>

static const char *TAG = "Oscillator";
extern unsigned long IRAM_ATTR millis();

static ledc_channel_t next_free_channel = LEDC_CHANNEL_0;

Oscillator::Oscillator(int trim) {
    trim_ = trim;
    diff_limit_ = 0;
    is_attached_ = false;

    sampling_period_ = 30;          // 默认采样周期30ms
    period_ = 2000;                 // 默认周期2s
    num_samples_ = period_ / sampling_period_; // 采样数 2000/30 = 66
    inc_ = 2*M_PI / num_samples_;   // 每次相位增量 2*PI/66=0.095

    amplitude_ = 45;            // 默认振幅为45度
    phase_ = 0;                 // 初始相位为0度
    phase0_ = 0;                
    offset_ = 0;           
    stop_ = false;
    rev_ = false;
    
    pos_ = 90;                  // 默认角度为90度 回中
    previous_millis_ = 0;
}

Oscillator::~Oscillator() {
    Detach();
}

void Oscillator::Attach(int pin, bool rev) {
    if (is_attached_)
        return;
    pin_ = pin;
    rev_ = rev;

    // 开启一个定时器
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 50, // 50Hz
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    static int last_channel = 0;
    last_channel = (last_channel + 1) % 7 + 1; // 0-7
    ledc_channel_ = (ledc_channel_t)last_channel;

    ledc_channel_config_t ledc_channel = {
        .gpio_num = pin_,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = ledc_channel_,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
        .flags = {
            .output_invert = 0
        }
    };

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ledc_speed_mode_ = LEDC_LOW_SPEED_MODE;

    previous_servo_command_millis_ = millis();
    is_attached_ = true;
}

/**
 * @brief 设置振荡器的周期并更新相关参数。
 *
 * 本函数用于设置振荡器的周期，并根据当前采样周期重新计算每周期采样数和每次采样的相位增量。
 *
 * @param period 振荡器的周期，单位与sampling_period_一致（毫秒）。
 */
void Oscillator::SetT(unsigned int period) {
    period_ = period;
    num_samples_ = period_ / sampling_period_; // 采样数
    inc_ = 2*M_PI / num_samples_;   // 每次相位增量
}

void Oscillator::Detach() {
    if (!is_attached_)
        return;
    ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE, ledc_channel_, 0));
    is_attached_ = false;
}

void Oscillator::SetPosition(int position) {
    Write(position);
}

/**
 * @brief 刷新振荡器状态，如果到达新的采样点则输出下一个值。
 *
 * 本函数通过调用 NextSample() 判断是否到达新的采样点。
 * 如果振荡器未停止（stop_ 为 false），则根据当前幅度、相位和偏移量计算下一个位置，
 * 如需反转则取相反数，并通过 Write() 方法写入（加上90度偏移）。
 * 最后相位增加 inc_，为下一个周期做准备。
 */
void Oscillator::Refresh() {
    if (NextSample())
    {
        if(!stop_) {
            int pos = std::round(amplitude_ * std::sin(phase_ + phase0_) + offset_);
            if(rev_) {
                pos = - pos;
            }
            Write(pos + 90);
        }
        phase_ += inc_;
    }
    
}

/**
 * @brief 判断是否应该处理下一个采样点（基于采样周期）。
 *
 * 此函数检查自上次采样以来经过的时间是否大于等于设定的采样周期。
 * 如果满足条件，则更新时间戳并返回true，表示可以处理下一个采样点；
 * 否则返回false。
 *
 * @return 如果采样周期已到，返回true；否则返回false。
 */
bool Oscillator::NextSample() {
    current_millis_ = millis();

    if (current_millis_ - previous_millis_ >= sampling_period_) {
        previous_millis_ = current_millis_;
        return true;
    }

    return false;
}

/**
 * @brief 向振荡器写入新的位置，应用移动限制并更新舵机。
 *
 * 此函数将振荡器移动到指定位置，可根据配置的 diff_limit_ 限制移动速率。
 * 它确保位置在有效范围内，应用修正（trim）偏移，并通过 ESP32 的 LEDC PWM 驱动更新舵机。
 *
 * @param position 目标位置（角度，单位为度）。
 *
 * @note 如果振荡器未连接（未 attach），此函数不执行任何操作。
 * @note 应用修正后，位置会被限制在 0 到 180 度之间。
 * @note 舵机 PWM 的占空比通过 ESP-IDF 的 LEDC API 计算并设置。
 */
void Oscillator::Write(int position) {
    if(!is_attached_) {return;}

    // 限制舵机（或振荡器）每次移动的最大步进幅度，从而实现平滑过渡，防止位置变化过快导致机械冲击或抖动。
    // 首先，代码判断 diff_limit_ 是否大于0。如果大于0，说明启用了移动速率限制。此时，limit 变量表示当前位置允许变化的最大步长。
    // limit 的计算方式是：用当前时间与上次发送舵机命令的时间差（单位毫秒），乘以 diff_limit_（通常代表每秒最大允许变化的角度），再除以1000，将其转换为与时间间隔相对应的最大角度变化。
    // std::max(1, ...) 保证了即使时间间隔很短，步进也至少为1度，避免卡死。
    long currentMillis = millis();
    if(diff_limit_ > 0) {
        int limit = std::max(
            1, (((int)(current_millis_ - previous_servo_command_millis_)) * diff_limit_) / 1000);
        if (abs(position - pos_) > limit) {
            pos_ += position < pos_ ? -limit : limit;
        } else {
            pos_ = position;
        }
    } else {
        pos_ = position;
    }

    previous_servo_command_millis_ = currentMillis;
    int angle = pos_ + trim_;
    angle = std::min(std::max(angle, 0), 180);
    /**
     * @brief 计算给定角度0-180度映射到特定的PWM范围。
     *
     * 角度（通常在0到180之间） 
     * 13位PWM（0-8191）下的占空比值。
     */
    uint32_t duty = (uint32_t)(((angle / 180.0) * 2.0 + 0.5) * 8191 / 20.0);

    ESP_ERROR_CHECK(ledc_set_duty(ledc_speed_mode_, ledc_channel_, duty));
    ESP_ERROR_CHECK(ledc_update_duty(ledc_speed_mode_, ledc_channel_));
}

/**
 * @brief 实现了角度到脉宽的线性转换。将舵机角度转换为对应的PWM比较值（脉宽，单位为微秒），是舵机控制中常见的基础工具函数。
 *
 * 此函数将输入的角度（限制在 SERVO_MIN_DEGREE 和 SERVO_MAX_DEGREE 之间），
 * 映射为 SERVO_MIN_PULSEWIDTH_US 到 SERVO_MAX_PULSEWIDTH_US 之间的脉宽值。
 * 结果可用于通过PWM控制舵机。
 * 
 * 这种映射方式常用于舵机控制，将角度（如0<del>180度）转换为PWM脉宽（如500</del>2500微秒），以驱动舵机转到指定角度。
 *
 * @param angle 期望的舵机角度（单位：度）。
 * @return uint32_t 对应的PWM比较值（脉宽，单位：微秒）。
 */
uint32_t Oscillator::AngleToCompare(int angle) {
    // 限制角度范围
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}
