#include "wifi_board.h"
#include "audio_codecs/es8311_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "iot/thing_manager.h"
#include "power_manager.h"
#include "sdkconfig.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_gc9a01.h>

#include <driver/gpio.h>
#include "esp_timer.h"
#include "led/circular_strip.h"

#define TAG "DeskmateStart"

bool button_released_ = false;
bool shutdown_ready_ = false;
esp_timer_handle_t shutdown_timer;

LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);

class CustomLcdDisplay : public SpiLcdDisplay {
public:
    CustomLcdDisplay(esp_lcd_panel_io_handle_t io_handle, 
                    esp_lcd_panel_handle_t panel_handle,
                    int width,
                    int height,
                    int offset_x,
                    int offset_y,
                    bool mirror_x,
                    bool mirror_y,
                    bool swap_xy) 
        : SpiLcdDisplay(io_handle, panel_handle, width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy,
                    {
                        .text_font = &font_puhui_20_4,
                        .icon_font = &font_awesome_20_4,
                        .emoji_font = font_emoji_64_init(),
                    }) {

        DisplayLockGuard lock(this);
        // 由于屏幕是圆的，所以状态栏需要增加左右内边距
        lv_obj_set_style_pad_left(status_bar_, LV_HOR_RES * 0.13, 0);
        lv_obj_set_style_pad_right(status_bar_, LV_HOR_RES * 0.13, 0);
    }
};

class DeskmateStart : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    Button key_button_;
    Display* display_;
    PowerManager* power_manager_;

    adc_oneshot_unit_handle_t adc1_handle;
    adc_cali_handle_t adc1_cali_handle;
    bool do_calibration = false;
    bool key_long_pressed = false;
    int64_t last_key_press_time = 0;
    static const int64_t LONG_PRESS_TIMEOUT_US = 5 * 1000000ULL;

    
    void InitializePowerManager() {
        power_manager_ =
            // new PowerManager(POWER_CHARGE_DETECT_PIN, POWER_ADC_UNIT, POWER_ADC_CHANNEL);
            new PowerManager(POWER_CHARGE_DETECT_PIN);
    }

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    
//     void InitializeADC() {
//         adc_oneshot_unit_init_cfg_t init_config1 = {
//             .unit_id = ADC_UNIT_1
//         };
//         ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

//         adc_oneshot_chan_cfg_t chan_config = {
//             .atten = ADC_ATTEN,
//             .bitwidth = ADC_WIDTH,
//         };
//         ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, VBAT_ADC_CHANNEL, &chan_config));

//         adc_cali_handle_t handle = NULL;
//         esp_err_t ret = ESP_FAIL;

// #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
//         adc_cali_curve_fitting_config_t cali_config = {
//             .unit_id = ADC_UNIT_1,
//             .atten = ADC_ATTEN,
//             .bitwidth = ADC_WIDTH,
//         };
//         ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
//         if (ret == ESP_OK) {
//             do_calibration = true;
//             adc1_cali_handle = handle;
//             ESP_LOGI(TAG, "ADC Curve Fitting calibration succeeded");
//         }
// #endif // ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
//     }

    // SPI初始化
    void InitializeSpi() {
        ESP_LOGI(TAG, "Initialize SPI bus");
        spi_bus_config_t buscfg = GC9A01_PANEL_BUS_SPI_CONFIG(DISPLAY_SPI_SCLK_PIN, DISPLAY_SPI_MOSI_PIN, 
                                    DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t));
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    // GC9A01初始化
    void InitializeGc9a01Display() {
        ESP_LOGI(TAG, "Init GC9A01 display");

        ESP_LOGI(TAG, "Install panel IO");
        esp_lcd_panel_io_handle_t io_handle = NULL;

        esp_lcd_panel_io_spi_config_t io_config = GC9A01_PANEL_IO_SPI_CONFIG(DISPLAY_SPI_CS_PIN, DISPLAY_SPI_DC_PIN, NULL, NULL);
        io_config.pclk_hz = DISPLAY_SPI_SCLK_HZ;
        io_config.cs_gpio_num = DISPLAY_SPI_CS_PIN;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &io_handle));
    
        ESP_LOGI(TAG, "Install GC9A01 panel driver");
        esp_lcd_panel_handle_t panel_handle = NULL;
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_SPI_RESET_PIN;    // Set to -1 if not use
        panel_config.rgb_endian = LCD_RGB_ENDIAN_BGR;           //LCD_RGB_ENDIAN_RGB;
        panel_config.bits_per_pixel = 16;                       // Implemented by LCD command `3Ah` (16/18)

        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true)); 

        display_ = new SpiLcdDisplay(io_handle, panel_handle,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                    {
                                        .text_font = &font_puhui_20_4,
                                        .icon_font = &font_awesome_20_4,
                                        .emoji_font = font_emoji_64_init(),
                                    });
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            // power_save_timer_->WakeUp();
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState(); 
        });

        key_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            app.ToggleChatState();
            key_long_pressed = false;
        });

        key_button_.OnLongPress([this]() {
            int64_t now = esp_timer_get_time();
            auto* led = static_cast<CircularStrip*>(this->GetLed());

            if (key_long_pressed) {
                if ((now - last_key_press_time) < LONG_PRESS_TIMEOUT_US) {
                    ESP_LOGW(TAG, "Key button long pressed the second time within 5s, shutting down...");
                    led->SetSingleColor(0, {0, 0, 0});
                    // gpio_set_level(DISPLAY_BACKLIGHT_PIN, 0);
                    GetBacklight()->SetBrightness(0);
                    gpio_hold_dis(MCU_VCC_CTL);
                    gpio_set_level(MCU_VCC_CTL, 0);

                } else {
                    last_key_press_time = now;
                    BlinkGreenFor5s();
                }
                key_long_pressed = true;
            } else {
                ESP_LOGW(TAG, "Key button first long press! Waiting second within 5s to shutdown...");
                last_key_press_time = now;
                key_long_pressed = true;

                BlinkGreenFor5s();
            }
        });

        // 在这里添加触摸任务
        
        // xTaskCreate(touch_task, "touch_task", 1024 * 5, NULL, 5, NULL);
    }

    void InitializePowerCtl() {
        InitializeGPIO();

        gpio_set_level(MCU_VCC_CTL, 1);
        gpio_hold_en(MCU_VCC_CTL);

        gpio_set_level(PERP_VCC_CTL, 1);
        gpio_hold_en(PERP_VCC_CTL);
    }

    
    void InitializeGPIO() {
        gpio_config_t io_pa = {
            .pin_bit_mask = (1ULL << AUDIO_CODEC_PA_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_pa);
        gpio_set_level(AUDIO_CODEC_PA_PIN, 0);

        gpio_config_t io_conf_1 = {
            .pin_bit_mask = (1ULL << MCU_VCC_CTL),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf_1);

        gpio_config_t io_conf_2 = {
            .pin_bit_mask = (1ULL << PERP_VCC_CTL),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf_2);
    }

    void InitializeIot() {
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Battery"));
        thing_manager.AddThing(iot::CreateThing("Screen"));   
        thing_manager.AddThing(iot::CreateThing("DeskmateController"));
    }

    void BlinkGreenFor5s() {
        auto* led = static_cast<CircularStrip*>(GetLed());
        if (!led) {
            return;
        }

        led->Blink({50, 25, 0}, 100);

        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                auto* self = static_cast<DeskmateStart*>(arg);
                auto* led = static_cast<CircularStrip*>(self->GetLed());
                if (led) {
                    led->SetSingleColor(0, {0, 0, 0});
                }
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "blinkGreenFor5s_timer"
        };

        esp_timer_handle_t blink_timer = nullptr;
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &blink_timer));
        ESP_ERROR_CHECK(esp_timer_start_once(blink_timer, LONG_PRESS_TIMEOUT_US));
    }

public:
    DeskmateStart() : boot_button_(BOOT_BUTTON_GPIO), key_button_(KEY_BUTTON_GPIO, true) {
        InitializePowerCtl();
        // InitializeADC();
        InitializeI2c();
        InitializeSpi();
        InitializeGc9a01Display();
        InitializeButtons();
        InitializeIot();
        GetBacklight()->RestoreBrightness();
        InitializePowerManager();
    }

    virtual Led* GetLed() override {
        static CircularStrip led(LED_PIN, 18);
        return &led;
    }

        virtual Display* GetDisplay() override {
        return display_;
    }
    
    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }

    virtual AudioCodec* GetAudioCodec() override {
         static Es8311AudioCodec audio_codec(i2c_bus_, I2C_NUM_0,
            AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE, AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_PA_PIN,
            AUDIO_CODEC_ES8311_ADDR, false);
        return &audio_codec;
    }

    // virtual bool GetBatteryLevel(int &level, bool &charging, bool &discharging) {
    //     if (!adc1_handle) {
    //         InitializeADC();
    //     }

    //     int raw_value = 0;
    //     int voltage = 0;

    //     ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, VBAT_ADC_CHANNEL, &raw_value));

    //     if (do_calibration) {
    //         ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, raw_value, &voltage));
    //         voltage = voltage * 3 / 2; // compensate for voltage divider
    //         ESP_LOGI(TAG, "Calibrated voltage: %d mV", voltage);
    //     } else {
    //         ESP_LOGI(TAG, "Raw ADC value: %d", raw_value);
    //         voltage = raw_value;
    //     }

    //     voltage = voltage < EMPTY_BATTERY_VOLTAGE ? EMPTY_BATTERY_VOLTAGE : voltage;
    //     voltage = voltage > FULL_BATTERY_VOLTAGE ? FULL_BATTERY_VOLTAGE : voltage;

    //     // 计算电量百分比
    //     level = (voltage - EMPTY_BATTERY_VOLTAGE) * 100 / (FULL_BATTERY_VOLTAGE - EMPTY_BATTERY_VOLTAGE);

    //     charging = gpio_get_level(MCU_VCC_CTL);
    //     ESP_LOGI(TAG, "Battery Level: %d%%, Charging: %s", level, charging ? "Yes" : "No");
    //     return true;
    // }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        charging = power_manager_->IsCharging();
        discharging = !charging;
        level = power_manager_->GetBatteryLevel();
        return true;
    }
};

DECLARE_BOARD(DeskmateStart);
