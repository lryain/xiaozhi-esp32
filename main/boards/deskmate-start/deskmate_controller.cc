// Deskmate controller

#include <esp_log.h>
#include <cstring>

#include "application.h"
#include "board.h"
#include "config.h"
#include "iot/thing.h"
#include "deskmate_movements.h"
#include "sdkconfig.h"

#define TAG "DeskmateController"

namespace iot {
struct DeskmateActionParams {
    int action_type;
    int steps;
    int speed;
    int direction;
    int amount;
};

class DeskmateController : public Thing {

private:
    Deskmate deskmate_;
    TaskHandle_t action_task_handle_ = nullptr;
    QueueHandle_t action_queue_ = nullptr;

    enum ActionType {
        ACTION_WALK,
        ACTION_NOD,
        ACTION_SHAKE_HEAD,
        ACTION_WAVE_HAND
    };

    // 限制数值在指定范围内
    static int Limit(int value, int min, int max) {
        if (value < min) {
            ESP_LOGW(TAG, "参数 %d 小于最小值 %d，设置为最小值", value, min);
            return min;
        }
        if (value > max) {
            ESP_LOGW(TAG, "参数 %d 大于最大值 %d，设置为最大值", value, max);
            return max;
        }
        return value;
    }

    static void ActionTask(void* arg) {
        DeskmateController* controller = static_cast<DeskmateController*>(arg);
        DeskmateActionParams action_params;
        controller->deskmate_.AttachServos();

        while (true) {
            if (xQueueReceive(controller->action_queue_, &action_params, pdMS_TO_TICKS(1000))== pdTRUE) {
                ESP_LOGI(TAG, "执行动作: %d, 步数: %d, 速度: %d, 方向: %d, 幅度: %d",
                            action_params.action_type, action_params.steps,
                            action_params.speed, action_params.direction,
                            action_params.amount);
                switch (action_params.action_type) {
                case ACTION_WALK:
                    controller->deskmate_.Walk(action_params.steps, action_params.speed,
                                                action_params.direction, action_params.amount);
                    break;
                case ACTION_NOD:
                    controller->deskmate_.HeadNod(action_params.steps, action_params.speed,
                                                action_params.direction, action_params.amount);
                    break;
                // case ACTION_SHAKE_HEAD:
                //     controller->deskmate_.ShakeBoth(action_params.steps, action_params.speed,
                //                                         action_params.direction, action_params.amount);
                //     break;
                // case ACTION_WAVE_HAND:
                //     controller->deskmate_.HandWave(action_params.steps, action_params.speed,
                //                                         action_params.direction, action_params.amount);
                //     break;
                }

                ESP_LOGI(TAG, "动作执行完成");
                controller->deskmate_.Home(true); // 动作完成后复位
            } else {
                if (uxQueueMessagesWaiting(controller->action_queue_) == 0) {
                    ESP_LOGI(TAG, "动作队列为空，等待新的动作参数...");
                    controller->deskmate_.Home(); // 动作完成后复位
                    vTaskDelay(pdMS_TO_TICKS(500));
                    controller->deskmate_.DetachServos();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    controller->action_task_handle_ = nullptr;
                    vTaskDelete(NULL);
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

public:
    DeskmateController():Thing("DeskmateController", "Deskmate Controller") {
        deskmate_.Init();
        deskmate_.Home(true);

        action_queue_ = xQueueCreate(10, sizeof(DeskmateActionParams));
        
        methods_.AddMethod("suspend", "清空动作队列，终端Deskmate动作", ParameterList(), 
            [this](const ParameterList& parameters) {
                ESP_LOGI(TAG, "停止机器人动作");
                if (action_task_handle_ != nullptr) {
                    vTaskDelete(action_task_handle_);
                    action_task_handle_ = nullptr;
                }
                xQueueReset(action_queue_);
                deskmate_.Home(true); // 中断动作时完全复位
            });

        methods_.AddMethod("AIControl", "AI把机器人待执行动作加入队列,动作需要时间", 
            ParameterList(
                {
                    Parameter("action_type",
                        "动作类型：0-行走，1-点头，2-摇头，3-挥手，4-复位", kValueTypeNumber, false),
                    Parameter("steps", "步数", kValueTypeNumber, false),
                    Parameter("speed", "速度", kValueTypeNumber, false),
                    Parameter("direction", "方向 (1=左, -1=右, 0=同时)", kValueTypeNumber, false),
                    Parameter("amount", "动作幅度(最小10)", kValueTypeNumber, true)}),
            [this](const ParameterList& parameters) {
                int action_type = parameters["action_type"].number();
                int steps = parameters["steps"].number();
                int speed = parameters["speed"].number();
                int direction = parameters["direction"].number();
                int amount = parameters["amount"].number();

                // 限制参数范围
                action_type = Limit(action_type, ACTION_WALK, ACTION_WAVE_HAND);
                steps = Limit(steps, 1, 100);
                speed = Limit(speed, 500, 3000);
                direction = Limit(direction, -1, 1);

                switch (action_type) {
                case ACTION_WALK:
                    amount = Limit(amount, 0, 100);
                    break;
                case ACTION_NOD:
                    amount = Limit(amount, 0, 30);
                    break;
                case ACTION_SHAKE_HEAD:
                    amount = Limit(amount, 10, 50);
                    break;
                case ACTION_WAVE_HAND:
                    amount = Limit(amount, 15, 40);
                    break;
                default:
                    amount = Limit(amount, 10, 50);
                }
                ESP_LOGI(TAG, "AI控制: 动作类型=%d, 步数=%d, 速度=%d, 方向=%d, 幅度=%d",
                        action_type, steps, speed, direction, amount);
                DeskmateActionParams action_params = {action_type, steps, speed, direction, amount};
                action_params.action_type = action_type;
                action_params.steps = steps;
                action_params.speed = speed;
                action_params.direction = direction;
                action_params.amount = amount;
                xQueueSend(action_queue_, &action_params, portMAX_DELAY);
                StartActionTaskIfNeeded();
            });
    }

    void StartActionTaskIfNeeded(){
        if(action_task_handle_ == nullptr) {
            xTaskCreate(ActionTask, "deskmate_action", 1024 * 3, this, 4, &action_task_handle_);
        }
    }
    ~DeskmateController(){
        if(action_task_handle_ != nullptr) {
            vTaskDelete(action_task_handle_);
        }
        vQueueDelete(action_queue_);
    }
};

} // namespace iot

DECLARE_THING(DeskmateController);
