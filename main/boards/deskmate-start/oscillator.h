#ifndef __OSCILLATOR_H__
#define __OSCILLATOR_H__

#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#define M_PI 3.14159265358979323846

#ifndef DEG2RAD
#define DEG2RAD(g) ((g) * M_PI) / 180
#endif

#define SERVO_MIN_PULSEWIDTH_US 500         // 最小脉宽(微秒)
#define SERVO_MAX_PULSEWIDTH_US 2500        // 最大脉宽(微秒)
#define SERVO_MAX_DEGREE 90                 // 最大转动角度
#define SERVO_MIN_DEGREE -90                // 最小转动角度
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD 20000           // 20000 ticks, 20ms


class Oscillator {

public:
    Oscillator(int trim = 0) ;
    ~Oscillator();

    void Attach(int pin, bool rev =false);
    void Detach();

    void SetA(unsigned int amplitude) {amplitude_ = amplitude;};
    void SetO(int offset) {offset_ = offset;};
    void SetPh(double phase) {phase0_ = phase;};
    void SetT(unsigned int period);
    void SetTrim(int trim) {trim_ = trim;};
    void SetLimiter(int diff_limit) {diff_limit_ = diff_limit;};
    void DisableLimiter() {diff_limit_ = 0;};
    
    int GetTrim() {return trim_;};
    void SetPosition(int position);
    void Stop() { stop_ = true; };
    void Play() { stop_ = false; };
    void Reset() {phase_ = 0;};
    void Refresh();
    int GetPosition() {return pos_;};

private:
    bool NextSample();
    void Write(int position);
    uint32_t AngleToCompare(int angle);

private:
    bool is_attached_;

    // Oscillator parameters
    unsigned int amplitude_;    // 振幅(degrees)
    int offset_;                // 偏移(degrees)
    unsigned int period_;       // 周期(milliseconds)
    double phase0_;             // 初始相位(degrees)

    // Internal variables
    int pos_;                   // 当前角度(degrees)  
    int pin_;                   // 当前舵机引脚
    int trim_;                  // 校准偏移值(degrees)
    double phase_;              // 当前相位(degrees)
    double inc_;                // 相位增量(degrees)
    double num_samples_;          // 采样数
    double sampling_period_;       // 采样周期(milliseconds)

    long previous_millis_;          // 上次更新时间
    long current_millis_;           // 当前时间

    // Oscillator mode. If true, the servo is stopped
    bool stop_;

    // Reverse mode. If true, the servo is reversed
    bool rev_;

    int diff_limit_;                        // 角度差限制(degrees)?
    long previous_servo_command_millis_;    // 上次舵机命令时间

    ledc_channel_t ledc_channel_;           // LEDC通道
    ledc_mode_t ledc_speed_mode_;           // LEDC模式
};



#endif // __OSCILLATOR_H__