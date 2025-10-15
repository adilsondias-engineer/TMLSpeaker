/* TML Speaker
 *
 * TML Speaker is a project to test LVGL on ESP32 based projects.
 *
 * Author: Adilson Dias(API-Led Pty Ltd -> Tine Memories Laser(TML)
 * Date: 2023-06-5
 * Version: 1.0.8
 *
 * This project requires a LCD panel with the following specifications:
 * - Resolution: 128x64
 * - Interface: SPI
 * - Driver: SSD1306
 *
 * This project requires a Bluetooth module with the following specifications:
 * - Interface: UART
 * - Driver: BT05
 *
 * This project requires a Audio Jack with the following specifications:
 * - Interface: Jack
 * - Driver: Audio Jack
 *
 * A specific PCB was designed to fit the above requirements with audio amplifier and
 * 3 x 3W speakers(left, right and subwoofer). Potentiometers were added to adjust the volume
 * and the bass and treble.
 * CD4066BE were used to switch the audio source between the Jack and the Bluetooth.
 * PCM5102APWR was used as DAC to convert the digital audio signal from ESP32 to an analog audio signal.
 * 3x LM386N-3 was used to amplify the audio signal: left, right and subwoofer.
 *
 * The PCB design was done in EasyEDA and the Gerbers were generated with EasyEDA.
 * The components were soldered by Adilson Dias(API-Led Pty Ltd -> Tine Memories Laser(TML)

 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// #include <esp_log.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>
#include <inttypes.h>

#include "soc/soc_caps.h"
#include "esp_log.h"
/*#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_freertos_hooks.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"

#include <sys/time.h>
// #include <time.h>
// #include "sys/lock.h"

#include "esp_timer.h"
#include "esp_system.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"

#include "esp_bt.h"
#include "bt_app_core.h"
// #include "bt_app_av.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#include "lvgl.h"

#include "lvgl_helpers.h"

/*********************
 *      DEFINES
 *********************/
#define TAG "TML_SPEAKER"
#define LV_TICK_PERIOD_MS 1

/* log tags */
#define BT_AV_TAG "BT_AV"
#define BT_RC_TG_TAG "RC_TG"
#define BT_RC_CT_TAG "RC_CT"

/* device name */
#define LOCAL_DEVICE_NAME "TML_SPEAKER_v8"

/* event for stack up */
enum
{
    BT_APP_EVT_STACK_UP = 0,
};

/* AVRCP used transaction labels */
#define APP_RC_CT_TL_GET_CAPS (0)
#define APP_RC_CT_TL_GET_META_DATA (1)
#define APP_RC_CT_TL_RN_TRACK_CHANGE (2)
#define APP_RC_CT_TL_RN_PLAYBACK_CHANGE (3)
#define APP_RC_CT_TL_RN_PLAY_POS_CHANGE (4)

#define I2S_DATA_PIN 22 // default 25 //DAC 14
#define I2S_LRCK_PIN 25 // 15//25 //default 22 //DAC 15
#define I2S_BCK_PIN 26  // 14//26 //DAC 13

// #define I2S_BCK_PIN 22
// #define I2S_LRCK_PIN 26
// #define I2S_DATA_PIN 25

#define LED_GPIO_PIN GPIO_NUM_21   // NOT USED
#define BT_CONFIRM_PIN GPIO_NUM_12 // To confirm BT connection
// define SWITCH_GPIO_PIN GPIO_NUM_33 //To check if Audio is BT or jack..
#define DIRECT_OUT_PIN 14 // jack output
#define BT_OUT_PIN 33     // BT output

/* Application layer causes delay value */
#define APP_DELAY_VALUE 50 // 5ms

lv_obj_t *bt_logo = NULL;
lv_obj_t *audio_jack = NULL;
lv_obj_t *lblNoInput = NULL;

lv_obj_t *bt_lvlPlaying = NULL;
lv_obj_t *bt_lblDescription = NULL;

int btState = 0; // false default
int audioJackState = 0;
// QueueHandle_t interputQueue;

// adc_oneshot_unit_handle_t adc1_handle;

/*******************************
 * STATIC FUNCTION DECLARATIONS
 ******************************/

/* allocate new meta buffer */
static void bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param);
/* handler for new track is loaded */
static void bt_av_new_track(void);
/* handler for track status change */
// static void bt_av_playback_changed(void);
/* handler for track playing position change */
// static void bt_av_play_pos_changed(void);
/* notification event handler */
static void bt_av_notify_evt_handler(uint8_t event_id, esp_avrc_rn_param_t *event_parameter);
/* installation for i2s */
static void bt_i2s_driver_install(void);
/* uninstallation for i2s */
static void bt_i2s_driver_uninstall(void);
/* set volume by remote controller */
static void volume_set_by_controller(uint8_t volume);
/* set volume by local host */
// static void volume_set_by_local_host(uint8_t volume);
/* simulation volume change */
// static void volume_change_simulation(void *arg);
/* a2dp event handler */
static void bt_av_hdl_a2d_evt(uint16_t event, void *p_param);
/* avrc controller event handler */
static void bt_av_hdl_avrc_ct_evt(uint16_t event, void *p_param);
/* avrc target event handler */
static void bt_av_hdl_avrc_tg_evt(uint16_t event, void *p_param);

/*******************************
 * STATIC VARIABLE DEFINITIONS
 ******************************/

static uint32_t s_pkt_cnt = 0; /* count for audio packet */
static esp_a2d_audio_state_t s_audio_state = ESP_A2D_AUDIO_STATE_STOPPED;
/* audio stream datapath state */
static const char *s_a2d_conn_state_str[] = {"Disconnected", "Connecting", "Connected", "Disconnecting"};
/* connection state in string */
static const char *s_a2d_audio_state_str[] = {"Suspended", "Started"};
/* audio stream datapath state in string */
static esp_avrc_rn_evt_cap_mask_t s_avrc_peer_rn_cap;
/* AVRC target notification capability bit mask */
static _lock_t s_volume_lock;
// static TaskHandle_t s_vcs_task_hdl = NULL; /* handle for volume change simulation task */
static uint8_t s_volume = 0; /* local volume value */
static bool s_volume_notify; /* notify volume change or not */
i2s_chan_handle_t tx_chan = NULL;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_tick_task(void *arg);

static void guiTask(void *pvParameter);

// static void setSource(void);

/* GAP callback function */
static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
/* handler for bluetooth stack enabled events */
static void bt_av_hdl_stack_evt(uint16_t event, void *p_param);

static int adc_raw[2][10];
static int voltage[2][10];
// static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
// static void example_adc_calibration_deinit(adc_cali_handle_t handle);

/********************************
 * STATIC FUNCTION DEFINITIONS
 *******************************/

static void bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param)
{
    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(param);
    uint8_t *attr_text = (uint8_t *)malloc(rc->meta_rsp.attr_length + 1);

    memcpy(attr_text, rc->meta_rsp.attr_text, rc->meta_rsp.attr_length);
    attr_text[rc->meta_rsp.attr_length] = 0;
    rc->meta_rsp.attr_text = attr_text;
}

static void bt_av_new_track(void)
{
    /* request metadata */
    uint8_t attr_mask = ESP_AVRC_MD_ATTR_TITLE |
                        ESP_AVRC_MD_ATTR_ARTIST |
                        ESP_AVRC_MD_ATTR_ALBUM |
                        ESP_AVRC_MD_ATTR_GENRE;
    esp_avrc_ct_send_metadata_cmd(APP_RC_CT_TL_GET_META_DATA, attr_mask);

    /* register notification if peer support the event_id */
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,
                                           ESP_AVRC_RN_TRACK_CHANGE))
    {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_TRACK_CHANGE,
                                                   ESP_AVRC_RN_TRACK_CHANGE, 0);
    }
}

/*
static void bt_av_playback_changed(void)
{
     * register notification if peer support the event_id *
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,
                                           ESP_AVRC_RN_PLAY_STATUS_CHANGE))
    {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_PLAYBACK_CHANGE,
                                                   ESP_AVRC_RN_PLAY_STATUS_CHANGE, 0);
    }
}

static void bt_av_play_pos_changed(void)
{
     * register notification if peer support the event_id *
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,
                                           ESP_AVRC_RN_PLAY_POS_CHANGED))
    {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_PLAY_POS_CHANGE,
                                                   ESP_AVRC_RN_PLAY_POS_CHANGED, 10);
    }
}
*/
static void bt_av_notify_evt_handler(uint8_t event_id, esp_avrc_rn_param_t *event_parameter)
{
    switch (event_id)
    {
    /* when new track is loaded, this event comes */
    case ESP_AVRC_RN_TRACK_CHANGE:
        bt_av_new_track();
        break;
    /* when track status changed, this event comes */
    case ESP_AVRC_RN_PLAY_STATUS_CHANGE:
        ESP_LOGI(BT_AV_TAG, "Playback status changed: 0x%x", event_parameter->playback);
        // bt_av_playback_changed();
        break;
    /* when track playing position changed, this event comes */
    case ESP_AVRC_RN_PLAY_POS_CHANGED:
        ESP_LOGI(BT_AV_TAG, "Play position changed: %" PRIu32 "-ms", event_parameter->play_pos);
        // bt_av_play_pos_changed();
        break;
    /* others */
    default:
        ESP_LOGI(BT_AV_TAG, "unhandled event: %d", event_id);
        break;
    }
}

void bt_i2s_driver_install(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;

    /*   .sample_rate = 44100,
     .bits_per_sample = 16,
     .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,                           //2-channels
     .communication_format = I2S_COMM_FORMAT_I2S_MSB,
     .dma_buf_count = 6,
     .dma_buf_len = 60,
     .intr_alloc_flags = 0,                                                  //Default interrupt priority
     .tx_desc_auto_clear = true   */

    i2s_std_config_t std_cfg = {
        /* .clk_cfg  = {
             .sample_rate_hz = 44100,
             .clk_src = I2S_CLK_SRC_APLL, //I2S_CLK_SRC_DEFAULT
             .mclk_multiple = I2S_MCLK_MULTIPLE_384,
         },*/
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),                                                // BT can only handle 16BIT
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO), // I2S_COMM_FORMAT_I2S_MSB //I2S_STD_PCM_SLOT_DEFAULT_CONFIG
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_PIN,
            .ws = I2S_LRCK_PIN,
            .dout = I2S_DATA_PIN,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    /* enable I2S */
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, NULL));
    // ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
}

void bt_i2s_driver_uninstall(void)
{
    ESP_ERROR_CHECK(i2s_channel_disable(tx_chan));
    ESP_ERROR_CHECK(i2s_del_channel(tx_chan));
}

static void volume_set_by_controller(uint8_t volume)
{
    ESP_LOGI(BT_RC_TG_TAG, "Volume is set by remote controller to: %" PRIu32 "%%", (uint32_t)volume * 100 / 0x7f);
    /* set the volume in protection of lock */
    _lock_acquire(&s_volume_lock);
    s_volume = volume;
    _lock_release(&s_volume_lock);
}
/*
static void volume_set_by_local_host(uint8_t volume)
{
    ESP_LOGI(BT_RC_TG_TAG, "Volume is set locally to: %" PRIu32 "%%", (uint32_t)volume * 100 / 0x7f);
     * set the volume in protection of lock *
    _lock_acquire(&s_volume_lock);
    s_volume = volume;
    _lock_release(&s_volume_lock);

     * send notification response to remote AVRCP controller *
    if (s_volume_notify)
    {
        esp_avrc_rn_param_t rn_param;
        rn_param.volume = s_volume;
        esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_CHANGED, &rn_param);
        s_volume_notify = false;
    }
}
*/
static void bt_av_hdl_a2d_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_AV_TAG, "%s event: %d", __func__, event);

    esp_a2d_cb_param_t *a2d = NULL;

    switch (event)
    {
    /* when connection state changed, this event comes */
    case ESP_A2D_CONNECTION_STATE_EVT:
    {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        uint8_t *bda = a2d->conn_stat.remote_bda;
        ESP_LOGI(BT_AV_TAG, "A2DP connection state: %s, [%02x:%02x:%02x:%02x:%02x:%02x]",
                 s_a2d_conn_state_str[a2d->conn_stat.state], bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED)
        {
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            bt_i2s_driver_uninstall();
            bt_i2s_task_shut_down();

            if (lv_obj_is_valid(bt_logo))
            {
                // lv_obj_del(audio_jack);
                // lv_obj_clear_flag(bt_logo, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(bt_logo, LV_OBJ_FLAG_HIDDEN);
                ESP_LOGI(BT_AV_TAG, "BT Hidden!");
                btState = 0;
            }
            if (audioJackState == 1)
            {
                if (lv_obj_is_valid(audio_jack))
                {
                    // lv_obj_del(audio_jack);
                    lv_obj_clear_flag(audio_jack, LV_OBJ_FLAG_HIDDEN);
                    // lv_obj_add_flag(audio_jack, LV_OBJ_FLAG_HIDDEN);
                    ESP_LOGI(BT_AV_TAG, "Audio Jack Visible!");
                }
            }
            else
            {
                if (lv_obj_is_valid(lblNoInput))
                {
                    // lv_obj_del(audio_jack);
                    lv_obj_clear_flag(lblNoInput, LV_OBJ_FLAG_HIDDEN);
                    // lv_obj_add_flag(audio_jack, LV_OBJ_FLAG_HIDDEN);
                    ESP_LOGI(BT_AV_TAG, "No Input Visible!");
                }
            }
        }
        else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED)
        {
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            bt_i2s_task_start_up();

            if (lv_obj_is_valid(audio_jack))
            {
                // lv_obj_del(audio_jack);
                // lv_obj_clear_flag(audio_jack, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(audio_jack, LV_OBJ_FLAG_HIDDEN);
                ESP_LOGI(BT_AV_TAG, "Audio Jack Hidden!");
            }
            if (lv_obj_is_valid(lblNoInput))
            {
                // lv_obj_del(audio_jack);
                // lv_obj_clear_flag(lblNoInput, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(lblNoInput, LV_OBJ_FLAG_HIDDEN);
                ESP_LOGI(BT_AV_TAG, "No Input Hidden");
            }
            if (lv_obj_is_valid(bt_logo))
            {
                // lv_obj_del(audio_jack);
                lv_obj_clear_flag(bt_logo, LV_OBJ_FLAG_HIDDEN);
                // lv_obj_add_flag(bt_logo, LV_OBJ_FLAG_HIDDEN);
                ESP_LOGI(BT_AV_TAG, "BT Visible!");
                btState = 1;
            }
        }
        else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTING)
        {
            bt_i2s_driver_install();
        }
        break;
    }
    /* when audio stream transmission state changed, this event comes */
    case ESP_A2D_AUDIO_STATE_EVT:
    {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "A2DP audio state: %s", s_a2d_audio_state_str[a2d->audio_stat.state]);
        s_audio_state = a2d->audio_stat.state;
        if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state)
        {
            s_pkt_cnt = 0;
        }
        break;
    }
    /* when audio codec is configured, this event comes */
    case ESP_A2D_AUDIO_CFG_EVT:
    {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "A2DP audio stream configuration, codec type: %d", a2d->audio_cfg.mcc.type);
        /* for now only SBC stream is supported */
        if (a2d->audio_cfg.mcc.type == ESP_A2D_MCT_SBC)
        {
            int sample_rate = 16000;
            int ch_count = 2;
            char oct0 = a2d->audio_cfg.mcc.cie.sbc[0];
            if (oct0 & (0x01 << 6))
            {
                sample_rate = 32000;
            }
            else if (oct0 & (0x01 << 5))
            {
                sample_rate = 44100;
            }
            else if (oct0 & (0x01 << 4))
            {
                sample_rate = 48000;
            }

            if (oct0 & (0x01 << 3))
            {
                ch_count = 1;
            }

            /* i2s_channel_disable(tx_chan);
             i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate);
             i2s_std_slot_config_t slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_8BIT, ch_count);
             i2s_channel_reconfig_std_clock(tx_chan, &clk_cfg);
             i2s_channel_reconfig_std_slot(tx_chan, &slot_cfg);
             i2s_channel_enable(tx_chan);
 */
            ESP_LOGI(BT_AV_TAG, "Configure audio player: %x-%x-%x-%x",
                     a2d->audio_cfg.mcc.cie.sbc[0],
                     a2d->audio_cfg.mcc.cie.sbc[1],
                     a2d->audio_cfg.mcc.cie.sbc[2],
                     a2d->audio_cfg.mcc.cie.sbc[3]);
            ESP_LOGI(BT_AV_TAG, "Audio player configured, sample rate: %d", sample_rate);
        }
        break;
    }
    /* when a2dp init or deinit completed, this event comes */
    case ESP_A2D_PROF_STATE_EVT:
    {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        if (ESP_A2D_INIT_SUCCESS == a2d->a2d_prof_stat.init_state)
        {
            ESP_LOGI(BT_AV_TAG, "A2DP PROF STATE: Init Complete");
        }
        else
        {
            ESP_LOGI(BT_AV_TAG, "A2DP PROF STATE: Deinit Complete");
        }
        break;
    }
    /* When protocol service capabilities configured, this event comes */
    case ESP_A2D_SNK_PSC_CFG_EVT:
    {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "protocol service capabilities configured: 0x%x ", a2d->a2d_psc_cfg_stat.psc_mask);
        if (a2d->a2d_psc_cfg_stat.psc_mask & ESP_A2D_PSC_DELAY_RPT)
        {
            ESP_LOGI(BT_AV_TAG, "Peer device support delay reporting");
        }
        else
        {
            ESP_LOGI(BT_AV_TAG, "Peer device unsupport delay reporting");
        }
        break;
    }
    /* when set delay value completed, this event comes */
    case ESP_A2D_SNK_SET_DELAY_VALUE_EVT:
    {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        if (ESP_A2D_SET_INVALID_PARAMS == a2d->a2d_set_delay_value_stat.set_state)
        {
            ESP_LOGI(BT_AV_TAG, "Set delay report value: fail");
        }
        else
        {
            ESP_LOGI(BT_AV_TAG, "Set delay report value: success, delay_value: %u * 1/10 ms", a2d->a2d_set_delay_value_stat.delay_value);
        }
        break;
    }
    /* when get delay value completed, this event comes */
    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT:
    {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "Get delay report value: delay_value: %u * 1/10 ms", a2d->a2d_get_delay_value_stat.delay_value);
        /* Default delay value plus delay caused by application layer */
        esp_a2d_sink_set_delay_value(a2d->a2d_get_delay_value_stat.delay_value + APP_DELAY_VALUE);
        break;
    }
    /* others */
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

static void bt_av_hdl_avrc_ct_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_RC_CT_TAG, "%s event: %d", __func__, event);

    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(p_param);

    switch (event)
    {
    /* when connection state changed, this event comes */
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
    {
        uint8_t *bda = rc->conn_stat.remote_bda;
        ESP_LOGI(BT_RC_CT_TAG, "AVRC conn_state event: state %d, [%02x:%02x:%02x:%02x:%02x:%02x]",
                 rc->conn_stat.connected, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

        if (rc->conn_stat.connected)
        {
            /* get remote supported event_ids of peer AVRCP Target */
            esp_avrc_ct_send_get_rn_capabilities_cmd(APP_RC_CT_TL_GET_CAPS);
        }
        else
        {
            /* clear peer notification capability record */
            s_avrc_peer_rn_cap.bits = 0;
        }
        break;
    }
    /* when passthrough responsed, this event comes */
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
    {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC passthrough rsp: key_code 0x%x, key_state %d, rsp_code %d", rc->psth_rsp.key_code,
                 rc->psth_rsp.key_state, rc->psth_rsp.rsp_code);
        break;
    }
    /* when metadata responsed, this event comes */
    case ESP_AVRC_CT_METADATA_RSP_EVT:
    {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC metadata rsp: attribute id 0x%x, %s", rc->meta_rsp.attr_id, rc->meta_rsp.attr_text);
        if (bt_lvlPlaying != NULL)
        {

            if (lv_obj_is_valid(bt_lvlPlaying))
            {
                lv_label_set_text(bt_lvlPlaying, (const char *)rc->meta_rsp.attr_text);

                // lv_obj_add_flag(bt_lvlPlaying, LV_OBJ_FLAG_HIDDEN); // hide it
                lv_obj_clear_flag(bt_lvlPlaying, LV_OBJ_FLAG_HIDDEN);
                // ESP_LOGI(BT_AV_TAG, "Audio Jack Visible!");
            }
        }
        if (bt_lblDescription != NULL)
        {

            if (lv_obj_is_valid(bt_lblDescription))
            {
                //  lv_obj_del(audio_jack);
                // lv_obj_add_flag(audio_jack, LV_OBJ_FLAG_HIDDEN);
                // lv_obj_add_flag(bt_lblDescription, LV_OBJ_FLAG_HIDDEN); // hide it
                lv_label_set_text(bt_lblDescription, (const char *)rc->meta_rsp.attr_text);
                lv_obj_clear_flag(bt_lblDescription, LV_OBJ_FLAG_HIDDEN);
                // ESP_LOGI(BT_AV_TAG, "Audio Jack Visible!");
            }
        }
        free(rc->meta_rsp.attr_text);
        break;
    }
    /* when notified, this event comes */
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
    {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC event notification: %d", rc->change_ntf.event_id);
        bt_av_notify_evt_handler(rc->change_ntf.event_id, &rc->change_ntf.event_parameter);
        break;
    }
    /* when feature of remote device indicated, this event comes */
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT:
    {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC remote features %" PRIx32 ", TG features %x", rc->rmt_feats.feat_mask, rc->rmt_feats.tg_feat_flag);
        break;
    }
    /* when notification capability of peer device got, this event comes */
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT:
    {
        ESP_LOGI(BT_RC_CT_TAG, "remote rn_cap: count %d, bitmask 0x%x", rc->get_rn_caps_rsp.cap_count,
                 rc->get_rn_caps_rsp.evt_set.bits);
        s_avrc_peer_rn_cap.bits = rc->get_rn_caps_rsp.evt_set.bits;
        bt_av_new_track();
        // bt_av_playback_changed();
        // bt_av_play_pos_changed();
        break;
    }
    /* others */
    default:
        ESP_LOGE(BT_RC_CT_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

static void bt_av_hdl_avrc_tg_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_RC_TG_TAG, "%s event: %d", __func__, event);

    esp_avrc_tg_cb_param_t *rc = (esp_avrc_tg_cb_param_t *)(p_param);

    switch (event)
    {
    /* when connection state changed, this event comes */
    case ESP_AVRC_TG_CONNECTION_STATE_EVT:
    {
        uint8_t *bda = rc->conn_stat.remote_bda;
        ESP_LOGI(BT_RC_TG_TAG, "AVRC conn_state evt: state %d, [%02x:%02x:%02x:%02x:%02x:%02x]",
                 rc->conn_stat.connected, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        if (rc->conn_stat.connected)
        {
            /* create task to simulate volume change */
            //  xTaskCreate(volume_change_simulation, "vcsTask", 2048, NULL, 5, &s_vcs_task_hdl);
        }
        else
        {
            //  vTaskDelete(s_vcs_task_hdl);
            ESP_LOGI(BT_RC_TG_TAG, "Stop volume change simulation");
        }
        break;
    }
    /* when passthrough commanded, this event comes */
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT:
    {
        ESP_LOGI(BT_RC_TG_TAG, "AVRC passthrough cmd: key_code 0x%x, key_state %d", rc->psth_cmd.key_code, rc->psth_cmd.key_state);
        break;
    }
    /* when absolute volume command from remote device set, this event comes */
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
    {
        ESP_LOGI(BT_RC_TG_TAG, "AVRC set absolute volume: %d%%", (int)rc->set_abs_vol.volume * 100 / 0x7f);
        volume_set_by_controller(rc->set_abs_vol.volume);
        break;
    }
    /* when notification registered, this event comes */
    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT:
    {
        ESP_LOGI(BT_RC_TG_TAG, "AVRC register event notification: %d, param: 0x%" PRIx32, rc->reg_ntf.event_id, rc->reg_ntf.event_parameter);
        if (rc->reg_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE)
        {
            s_volume_notify = true;
            esp_avrc_rn_param_t rn_param;
            rn_param.volume = s_volume;
            esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_INTERIM, &rn_param);
        }
        break;
    }
    /* when feature of remote device indicated, this event comes */
    case ESP_AVRC_TG_REMOTE_FEATURES_EVT:
    {
        ESP_LOGI(BT_RC_TG_TAG, "AVRC remote features: %" PRIx32 ", CT features: %x", rc->rmt_feats.feat_mask, rc->rmt_feats.ct_feat_flag);
        break;
    }
    /* others */
    default:
        ESP_LOGE(BT_RC_TG_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

/********************************
 * EXTERNAL FUNCTION DEFINITIONS
 *******************************/

void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event)
    {
    case ESP_A2D_CONNECTION_STATE_EVT:
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_PROF_STATE_EVT:
    case ESP_A2D_SNK_PSC_CFG_EVT:
    case ESP_A2D_SNK_SET_DELAY_VALUE_EVT:
    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT:
    {
        bt_app_work_dispatch(bt_av_hdl_a2d_evt, event, param, sizeof(esp_a2d_cb_param_t), NULL);
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "Invalid A2DP event: %d", event);
        break;
    }
}

void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len)
{
    write_ringbuf(data, len);

    /* log the number every 100 packets */
    if (++s_pkt_cnt % 100 == 0)
    {
        ESP_LOGI(BT_AV_TAG, "Audio packet count: %" PRIu32, s_pkt_cnt);
    }
}

void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    switch (event)
    {
    case ESP_AVRC_CT_METADATA_RSP_EVT:
        bt_app_alloc_meta_buffer(param);
        /* fall through */
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT:
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT:
    {
        bt_app_work_dispatch(bt_av_hdl_avrc_ct_evt, event, param, sizeof(esp_avrc_ct_cb_param_t), NULL);
        break;
    }
    default:
        ESP_LOGE(BT_RC_CT_TAG, "Invalid AVRC event: %d", event);
        break;
    }
}

void bt_app_rc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param)
{
    switch (event)
    {
    case ESP_AVRC_TG_CONNECTION_STATE_EVT:
    case ESP_AVRC_TG_REMOTE_FEATURES_EVT:
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT:
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT:
    case ESP_AVRC_TG_SET_PLAYER_APP_VALUE_EVT:
        bt_app_work_dispatch(bt_av_hdl_avrc_tg_evt, event, param, sizeof(esp_avrc_tg_cb_param_t), NULL);
        break;
    default:
        ESP_LOGE(BT_RC_TG_TAG, "Invalid AVRC event: %d", event);
        break;
    }
}

/*******************************
 * STATIC FUNCTION DEFINITIONS
 ******************************/

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    uint8_t *bda = NULL;

    switch (event)
    {
    /* when authentication completed, this event comes */
    case ESP_BT_GAP_AUTH_CMPL_EVT:
    {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(BT_AV_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(BT_AV_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        }
        else
        {
            ESP_LOGE(BT_AV_TAG, "authentication failed, status: %d", param->auth_cmpl.stat);
        }
        ESP_LOGI(BT_AV_TAG, "link key type of current link is: %d", param->auth_cmpl.lk_type);
        break;
    }
    case ESP_BT_GAP_ENC_CHG_EVT:
    {
        char *str_enc[3] = {"OFF", "E0", "AES"};
        bda = (uint8_t *)param->enc_chg.bda;
        ESP_LOGI(BT_AV_TAG, "Encryption mode to [%02x:%02x:%02x:%02x:%02x:%02x] changed to %s",
                 bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], str_enc[param->enc_chg.enc_mode]);
        break;
    }

        // if (CONFIG_EXAMPLE_A2DP_SINK_SSP_ENABLED == true)
    /* when Security Simple Pairing user confirmation requested, this event comes */
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %" PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    /* when Security Simple Pairing passkey notified, this event comes */
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey: %" PRIu32, param->key_notif.passkey);
        break;
    /* when Security Simple Pairing passkey requested, this event comes */
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
        // #endif

    /* when GAP mode changed, this event comes */
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode: %d", param->mode_chg.mode);
        break;
    /* when ACL connection completed, this event comes */
    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        bda = (uint8_t *)param->acl_conn_cmpl_stat.bda;
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT Connected to [%02x:%02x:%02x:%02x:%02x:%02x], status: 0x%x",
                 bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], param->acl_conn_cmpl_stat.stat);

        gpio_set_level(BT_OUT_PIN, 1);     // BT enabled
        gpio_set_level(DIRECT_OUT_PIN, 0); // Jack  disabled

        ESP_LOGI(BT_AV_TAG, "... Source changed to BT ...");

        break;
    /* when ACL disconnection completed, this event comes */
    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        bda = (uint8_t *)param->acl_disconn_cmpl_stat.bda;
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_ACL_DISC_CMPL_STAT_EVT Disconnected from [%02x:%02x:%02x:%02x:%02x:%02x], reason: 0x%x",
                 bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], param->acl_disconn_cmpl_stat.reason);

        gpio_set_level(BT_OUT_PIN, 0);     // BT Disabled
        gpio_set_level(DIRECT_OUT_PIN, 1); // Jack  enabled

        ESP_LOGI(BT_AV_TAG, "... Source changed to Jack ...");

        break;
    /* others */
    default:
    {
        ESP_LOGI(BT_AV_TAG, "event: %d", event);
        break;
    }
    }
}

static void bt_av_hdl_stack_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_AV_TAG, "%s event: %d", __func__, event);

    switch (event)
    {
    /* when do the stack up, this event comes */
    case BT_APP_EVT_STACK_UP:
    {
        // esp_bt_dev_set_device_name(LOCAL_DEVICE_NAME);
        esp_bt_gap_set_device_name(LOCAL_DEVICE_NAME);
        esp_bt_gap_register_callback(bt_app_gap_cb);

        assert(esp_avrc_ct_init() == ESP_OK);
        esp_avrc_ct_register_callback(bt_app_rc_ct_cb);
        assert(esp_avrc_tg_init() == ESP_OK);
        esp_avrc_tg_register_callback(bt_app_rc_tg_cb);

        esp_avrc_rn_evt_cap_mask_t evt_set = {0};
        esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
        assert(esp_avrc_tg_set_rn_evt_cap(&evt_set) == ESP_OK);

        assert(esp_a2d_sink_init() == ESP_OK);
        esp_a2d_register_callback(&bt_app_a2d_cb);
        esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);

        /* Get the default value of the delay value */
        esp_a2d_sink_get_delay_value();

        /* set discoverable and connectable mode, wait to be connected */
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;
    }
    /* others */
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

void bt_app_main(void)
{
    /* initialize NVS — it is used to store PHY calibration data */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /*
     * This example only uses the functions of Classical Bluetooth.
     * So release the controller memory for Bluetooth Low Energy.
     */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((err = esp_bt_controller_init(&bt_cfg)) != ESP_OK)
    {
        ESP_LOGE(BT_AV_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(err));
        return;
    }
    if ((err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK)
    {
        ESP_LOGE(BT_AV_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(err));
        return;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    // if (CONFIG_EXAMPLE_A2DP_SINK_SSP_ENABLED == false)
    //    bluedroid_cfg.ssp_en = false;
    // #endif
    if ((err = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK)
    {
        ESP_LOGE(BT_AV_TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(err));
        return;
    }

    if ((err = esp_bluedroid_enable()) != ESP_OK)
    {
        ESP_LOGE(BT_AV_TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(err));
        return;
    }

    // if (CONFIG_EXAMPLE_A2DP_SINK_SSP_ENABLED == true)
    /* set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
    // endif

    /* set default parameters for Legacy Pairing (use fixed pin code 1234) */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code;
    pin_code[0] = '1';
    pin_code[1] = '2';
    pin_code[2] = '3';
    pin_code[3] = '4';
    esp_bt_gap_set_pin(pin_type, 4, pin_code);

    bt_app_task_start_up();
    /* bluetooth device name, connection mode and profile set up */
    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);
}

// static void IRAM_ATTR gpio_interrupt_handler(void *args)
//{
// int pinNumber = (int)args;
// int data[2];

/*

 if (audioJackState == 0)
 {
     audioJackState = 1;
 }
 else
 {
     audioJackState = 0;
 }


 data[0] = gpio_get_level(SWITCH_GPIO_PIN);
 data[1] = audioJackState;

 xQueueSendFromISR(interputQueue, data, NULL);
 */
//}
/*
void LED_Control_Task(void *params)
{
    int currenState, count, val = 0;

    //  int data[2];
    while (true)
    {
        currenState = gpio_get_level(SWITCH_GPIO_PIN);
        // val = adc1_get_raw(ADC1_CHANNEL_0);

        // printf("ADC1_CHANNEL_0 %d value is %d\n", ADC1_CHANNEL_0, val);

        // xQueueReceive(interputQueue, &data, portMAX_DELAY);
        if (currenState == 0 && audioJackState == 0) // gpio_get_level(SWITCH_GPIO_PIN) == 1 &&
        {
            printf("GPIO %d was pressed %d times. The value is %d and state is %d\n", SWITCH_GPIO_PIN, count++, currenState, audioJackState);
            // if (gpio_get_level(SWITCH_GPIO_PIN) == 1 && state == 0)
            // {
            gpio_set_level(LED_GPIO_PIN, 1);

            if (audio_jack != NULL)
            {

                if (lv_obj_is_valid(audio_jack))
                {
                    //  lv_obj_del(audio_jack);
                    // lv_obj_add_flag(audio_jack, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_clear_flag(audio_jack, LV_OBJ_FLAG_HIDDEN);
                    ESP_LOGI(BT_AV_TAG, "Audio Jack Visible!");
                }
                else
                {
                    ESP_LOGI(BT_AV_TAG, "Audio Jack not alive, can't make visible!");
                }
                if (btState == 0)
                {
                    if (lv_obj_is_valid(lblNoInput))
                    {
                        // lv_obj_del(audio_jack);
                        // lv_obj_clear_flag(lblNoInput, LV_OBJ_FLAG_HIDDEN);
                        lv_obj_add_flag(lblNoInput, LV_OBJ_FLAG_HIDDEN);
                        ESP_LOGI(BT_AV_TAG, "No Input Hidden!");
                    }
                }
            }

            // ESP_LOGI(BT_AV_TAG, "Audio Jack Hidden!");
            count = 0;
            audioJackState = 1;
            //  }
        }
        else if (currenState == 1 && audioJackState == 1) // gpio_get_level(SWITCH_GPIO_PIN) == 1 &&
        {
            printf("GPIO %d was pressed %d times. The value is %d and state is %d\n", SWITCH_GPIO_PIN, count++, currenState, audioJackState);

            gpio_set_level(LED_GPIO_PIN, 0);

            // to be safe remove before creating again
            if (lv_obj_is_valid(audio_jack))
            {
                // lv_obj_del(audio_jack);
                // lv_obj_clear_flag(audio_jack, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(audio_jack, LV_OBJ_FLAG_HIDDEN);
                ESP_LOGI(BT_AV_TAG, "Audio Jack Hidden!");
            }
            else
            {
                ESP_LOGI(BT_AV_TAG, "Audio Jack not alive, before hidding!");
            }
            if (btState == 0)
            {
                if (lv_obj_is_valid(lblNoInput))
                {
                    // lv_obj_del(audio_jack);
                    lv_obj_clear_flag(lblNoInput, LV_OBJ_FLAG_HIDDEN);
                    // lv_obj_add_flag(audio_jack, LV_OBJ_FLAG_HIDDEN);
                    ESP_LOGI(BT_AV_TAG, "No Input Visible!");
                }
            }
            count = 0;
            audioJackState = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));

    } // for
}
*/
void BT_Confirm_Task(void *params)
{
    int currenState = 0;
    int count = 0;

    //  int data[2];
    while (true)
    {
        currenState = gpio_get_level(BT_CONFIRM_PIN);
        //  printf("GPIO %d was pressed %d times. The value is %d\n", BT_CONFIRM_PIN, count, currenState);
        if (currenState == 1)
        {
            count++;
            printf("GPIO %d was pressed %d times. The value is %d\n", BT_CONFIRM_PIN, count, currenState);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));

    } // for
}
/**********************
 *   APPLICATION MAIN
 **********************/
void app_main()
{

    /*
     gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO_PIN),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&io_conf);
*/

    // esp_rom_gpio_pad_select_gpio(LED_GPIO_PIN);
    // gpio_set_direction(LED_GPIO_PIN, GPIO_MODE_OUTPUT);

    esp_rom_gpio_pad_select_gpio(BT_CONFIRM_PIN);
    gpio_set_direction(BT_CONFIRM_PIN, GPIO_MODE_INPUT);

    esp_rom_gpio_pad_select_gpio(BT_OUT_PIN);
    gpio_set_direction(BT_OUT_PIN, GPIO_MODE_OUTPUT);

    esp_rom_gpio_pad_select_gpio(DIRECT_OUT_PIN);
    gpio_set_direction(DIRECT_OUT_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(BT_OUT_PIN, 0);     // BT Disabled
    gpio_set_level(DIRECT_OUT_PIN, 1); // Jack  enabled

    /*  esp_rom_gpio_pad_select_gpio(SWITCH_GPIO_PIN);
      gpio_set_direction(SWITCH_GPIO_PIN, GPIO_MODE_INPUT);
      gpio_pulldown_en(SWITCH_GPIO_PIN);
      gpio_pullup_dis(SWITCH_GPIO_PIN);
      gpio_set_intr_type(SWITCH_GPIO_PIN, GPIO_INTR_POSEDGE);
  */
    // interputQueue = xQueueCreate(1, sizeof(int));
    // xTaskCreate(LED_Control_Task, "LED_Control_Task", 2048, NULL, 1, NULL);

    xTaskCreatePinnedToCore(BT_Confirm_Task, "BT_Confirm_Task", 2048, NULL, 1, NULL, 1);
    // gpio_install_isr_service(0);
    // gpio_isr_handler_add(SWITCH_GPIO_PIN, gpio_interrupt_handler, (void *)SWITCH_GPIO_PIN);

    // star BT
    bt_app_main();

    /* If you want to use a task to create the graphic, you NEED to create a Pinned task
     * Otherwise there can be problem such as memory corruption and so on.
     * NOTE: When not using Wi-Fi nor Bluetooth you can pin the guiTask to core 0 */
    xTaskCreatePinnedToCore(guiTask, "gui", 4096 * 2, NULL, 0, NULL, 1);
    /*
        while (1) {
            gpio_set_level(LED_GPIO_PIN, 1); // Turn ON
            vTaskDelay(1000 / portTICK_RATE_MS);
            gpio_set_level(LED_GPIO_PIN, 0); // Turn OFF
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
        */
    // time_t now;
    // char strftime_buf[64];
    // struct tm timeinfo;

    // Set timezone to Australia East Standard Time
    /* setenv("TZ", "AEST", 1);
     tzset();

     struct tm stm;
     struct timeval stv;
     time_t date;

     stm.tm_mday = 16;
     stm.tm_mon  = 7-1;
     stm.tm_year = 2024-1900;
     stm.tm_hour = 17;
     stm.tm_min  = 50;
     stm.tm_sec  = 00;
     date = mktime(&stm);

     stv.tv_sec = date;
     stv.tv_usec = 0;
     settimeofday(&stv, NULL);

     time(&now);
     localtime_r(&now, &timeinfo);
     strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
     ESP_LOGI(TAG, "The current date/time in Melbourne is: %s", strftime_buf); */

    // adc1_config_width(ADC_WIDTH_BIT_12);
    // adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);

    //-------------ADC1 Init---------------//
    // adc_oneshot_unit_handle_t adc1_handle;
    // adc_oneshot_unit_init_cfg_t init_config1 = {
    //     .unit_id = ADC_UNIT_1,
    // };
    // ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    // adc_oneshot_chan_cfg_t config = {
    //     .bitwidth = ADC_BITWIDTH_DEFAULT,
    //     .atten = ADC_ATTEN_DB_12,
    // };
    // ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));
    //  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN1, &config));

    //-------------ADC1 Calibration Init---------------//
    // adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    // adc_cali_handle_t adc1_cali_chan1_handle = NULL;
    // bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_0, ADC_ATTEN_DB_12, &adc1_cali_chan0_handle);
    //  bool do_calibration1_chan1 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN1, EXAMPLE_ADC_ATTEN, &adc1_cali_chan1_handle);
    /*while (1)
    {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_raw[0][0]));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_0, adc_raw[0][0]);
        if (do_calibration1_chan0)
        {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, ADC_CHANNEL_0, voltage[0][0]);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }*/
    // Tear Down
    // ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    // if (do_calibration1_chan0)
    //{
    //    example_adc_calibration_deinit(adc1_cali_chan0_handle);
    //}
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}
*/
/* Creates a semaphore to handle concurrent call to lvgl stuff
 * If you wish to call *any* lvgl function from other threads/tasks
 * you should lock on the very same semaphore! */
SemaphoreHandle_t xGuiSemaphore;

static void lv_tick_task(void *arg)
{
    (void)arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}

static void guiTask(void *pvParameter)
{

    (void)pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();

    lv_init();

    /* Initialize SPI or I2C bus used by the drivers */
    lvgl_driver_init();

    lv_color_t *buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);

    /* Use double buffered when not working with monochrome displays */
    lv_color_t *buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2 != NULL);

    static lv_disp_draw_buf_t disp_buf;

    uint32_t size_in_px = DISP_BUF_SIZE;

    /* Initialize the working buffer depending on the selected display.
     * NOTE: buf2 == NULL when using monochrome displays. */
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, size_in_px);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;

#if defined CONFIG_DISPLAY_ORIENTATION_PORTRAIT || defined CONFIG_DISPLAY_ORIENTATION_PORTRAIT_INVERTED
    disp_drv.rotated = 1;
#endif
    // need to set resolution for LVGL 8x
    disp_drv.hor_res = CONFIG_LV_HOR_RES_MAX;
    disp_drv.ver_res = CONFIG_LV_VER_RES_MAX;

    disp_drv.draw_buf = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"};
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    /* Create the demo application */
    // create_demo_application();
    lv_obj_t *lblSource = lv_label_create(lv_scr_act());
    lv_label_set_text(lblSource, "Source: ");
    lv_obj_set_style_text_font(lblSource, &lv_font_montserrat_18, 0);
    lv_obj_align(lblSource, LV_ALIGN_TOP_LEFT, 0, 0); // h, v - 0,0 center

    /* Cable */
    audio_jack = lv_img_create(lv_scr_act());
    LV_IMG_DECLARE(img_audio_jack3);
    lv_img_set_src(audio_jack, &img_audio_jack3);
    lv_obj_align(audio_jack, LV_ALIGN_TOP_LEFT, 30, 20);
    lv_obj_add_flag(audio_jack, LV_OBJ_FLAG_HIDDEN); // hide it
    // setSource();

    /*  BT source*/
    bt_logo = lv_img_create(lv_scr_act());
    LV_IMG_DECLARE(img_bt_logo);
    lv_img_set_src(bt_logo, &img_bt_logo);
    lv_obj_align(bt_logo, LV_ALIGN_TOP_LEFT, 10, 12);
    lv_obj_add_flag(bt_logo, LV_OBJ_FLAG_HIDDEN); // hide it

    lblNoInput = lv_label_create(lv_scr_act());
    lv_label_set_text(lblNoInput, " No Input ");
    lv_obj_set_style_text_font(lblNoInput, &lv_font_montserrat_18, 0);
    lv_obj_align(lblNoInput, LV_ALIGN_TOP_LEFT, 10, 20); // h, v - 0,0 center

    bt_lvlPlaying = lv_label_create(lv_scr_act());
    lv_label_set_text(bt_lvlPlaying, "Playing blah blah");
    lv_obj_set_style_text_font(bt_lvlPlaying, &lv_font_montserrat_16, 0);
    lv_obj_align(bt_lvlPlaying, LV_ALIGN_TOP_LEFT, 0, 45);
    lv_obj_add_flag(bt_lvlPlaying, LV_OBJ_FLAG_HIDDEN); // hide it

    bt_lblDescription = lv_label_create(lv_scr_act());
    lv_label_set_text(bt_lblDescription, "Description");
    lv_obj_set_style_text_font(bt_lblDescription, &lv_font_montserrat_16, 0);
    lv_obj_align(bt_lblDescription, LV_ALIGN_TOP_LEFT, 0, 70);
    lv_obj_add_flag(bt_lblDescription, LV_OBJ_FLAG_HIDDEN); // hide it

    ESP_LOGI(TAG, "... App Started ...");

    while (1)
    {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
        {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
        }
    }

    /* A task should NEVER return */
    free(buf1);
    free(buf2);
    vTaskDelete(NULL);
}

/*static void setSource(void)
{

    lv_obj_t *audio_jack = lv_img_create(lv_scr_act());
    LV_IMG_DECLARE(img_audio_jack3);
    lv_img_set_src(audio_jack, &img_audio_jack3);
    lv_obj_align(audio_jack, LV_ALIGN_TOP_LEFT, 10, 60);

        lv_obj_t *lvlPlaying = lv_label_create( lv_scr_act() );
        lv_label_set_text( lvlPlaying, "Playing blah blah" );
        lv_obj_align( lvlPlaying, LV_ALIGN_TOP_LEFT, 0, 20  );

        lv_obj_t *lblDescription = lv_label_create( lv_scr_act() );
        lv_label_set_text( lblDescription, "Description" );
        lv_obj_align( lblDescription, LV_ALIGN_TOP_LEFT,  0, 40 );

}*/