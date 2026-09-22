#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "esp_log.h"
#include "debug_protocol.h"
#include "control_params.h"
#include "udp_logger.h"


static const char *TAG = "debug_protol";

extern TaskHandle_t ota_https_task_handle;

debug_cmd_t tcp_cmd_data_v;

// extern const int CONNECTED_BIT;
// extern const int ESPTOUCH_DONE_BIT;
extern const int RUN_OTA_BIT;



// 枚举字符串查找表
const char* tcp_cmd_names[] = {
    "DEBUG_CMD_NONE",
    "DEBUG_CMD_VOFA",
    
    /* 速度环 */
    "DEBUG_CMD_SPEED_KP",
    "DEBUG_CMD_SPEED_KI",
    "DEBUG_CMD_SPEED_KD",
    
    /* 位置环 */
    "DEBUG_CMD_POS_KP",
    "DEBUG_CMD_POS_KI",
    "DEBUG_CMD_POS_KD",
    
    /* IQ 电流环 */
    "DEBUG_CMD_IQ_KP",
    "DEBUG_CMD_IQ_KI",
    "DEBUG_CMD_IQ_KD",
    
    /* ID 电流环 */
    "DEBUG_CMD_ID_KP",
    "DEBUG_CMD_ID_KI",
    "DEBUG_CMD_ID_KD",
    
    /* SMO */
    "DEBUG_CMD_SMO_KS",
    "DEBUG_CMD_SMO_LPF",
    
    /* PLL */
    "DEBUG_CMD_PLL_KP",
    "DEBUG_CMD_PLL_KI",
    
    /* 电机 */
    "DEBUG_CMD_SPEED_REF",
    "DEBUG_CMD_POS_REF",
};


static const debug_cmd_map_t debug_cmd_table[] =
{
    {"sped_Kp", DEBUG_CMD_SPEED_KP},
    {"sped_Ki", DEBUG_CMD_SPEED_KI},
    {"sped_Kd", DEBUG_CMD_SPEED_KD},

    {"pos_Kp", DEBUG_CMD_POS_KP},
    {"pos_Ki", DEBUG_CMD_POS_KI},
    {"pos_Kd", DEBUG_CMD_POS_KD},

    {"iq_Kp", DEBUG_CMD_IQ_KP},
    {"iq_Ki", DEBUG_CMD_IQ_KI},
    {"iq_Kd", DEBUG_CMD_IQ_KD},

    {"id_Kp", DEBUG_CMD_ID_KP},
    {"id_Ki", DEBUG_CMD_ID_KI},
    {"id_Kd", DEBUG_CMD_ID_KD},

    {"smo_ks", DEBUG_CMD_SMO_KS},
    {"smo_lpf", DEBUG_CMD_SMO_LPF},

    {"pll_Kp", DEBUG_CMD_PLL_KP},
    {"pll_Ki", DEBUG_CMD_PLL_KI},

    {"speed_ref", DEBUG_CMD_SPEED_REF},
    {"pos_ref", DEBUG_CMD_POS_REF},
    {"run_ota", DEBUG_CMD_RUN_OTA},
};



// 获取枚举名称的函数
inline const char* debug_cmd_to_string(debug_cmd_id_t index)
{
    if (index >= DEBUG_CMD_NONE && index < DEBUG_CMD_MAX)
    {
        return tcp_cmd_names[index];
    }
    return "UNKNOWN_CMD";
}


static debug_cmd_id_t find_debug_cmd(const char *name)
{
    for (int i = 0;
         i < sizeof(debug_cmd_table) / sizeof(debug_cmd_table[0]);
         i++)
    {
        if (strcmp(name, debug_cmd_table[i].name) == 0)
        {
            return debug_cmd_table[i].id;
        }
    }

    return DEBUG_CMD_NONE;
}


const char *get_debug_cmd_para_name(debug_cmd_id_t id)
{
    for (int i = 0;
         i < sizeof(debug_cmd_table) / sizeof(debug_cmd_table[0]);
         i++)
    {
        if (debug_cmd_table[i].id == id)
        {
            return debug_cmd_table[i].name;
        }
    }

    if (id == DEBUG_CMD_VOFA)
    {
        return "vofa";
    }

    return "unknown";
}




/**
 * @brief debug协议命令解析
 * 
 * @param parm_cmd 命令数据
 * @param cmd 存储解析的命令数据结构体
 */
void debug_cmd_proces(char *parm_cmd,debug_cmd_t *cmd)
{
    if (cmd == NULL)
    {
        return;
    }

    // 判断接收到的字符串是否为 "vofa"
    if (strcmp((char*)parm_cmd, "vofa") == 0)
    {
        cmd->id = DEBUG_CMD_VOFA;
        cmd->value = 0.0f;

        strcpy(cmd->name, "vofa");

        ESP_LOGI(TAG,"vofa66");

        return;
    }

    char para_name[32] = {0}; 
    float para_value = 0.0f;

    if (sscanf((char*)parm_cmd, "%31[^:]:%f", para_name, &para_value) == 2) 
    {   
        // ESP_LOGI(
        //     TAG,
        //     "11parse_line: cmdname:%s,name:%s,value:%.3f", 
        //     parm_cmd,
        //     para_name,
        //     para_value
        // );

        cmd->id = find_debug_cmd(para_name);
        if (cmd->id == DEBUG_CMD_NONE)
        {
            return;
        }
        cmd->value = para_value;
        
        strncpy(cmd->name, get_debug_cmd_para_name(cmd->id), strlen(get_debug_cmd_para_name(cmd->id)) ); 

        // ESP_LOGI(TAG,"parse_line: name:%s(%s),value:%.3f,id:%s", 
        //     cmd->name,
        //     get_debug_cmd_para_name(cmd->id),
        //     cmd->value,
        //     debug_cmd_to_string(cmd->id)
        // );
        
    }

    ESP_LOGI(
        TAG,
        "CMD[id:%d]: %s = %.2f",
        cmd->id,
        cmd->name,
        cmd->value
    );


    switch (cmd->id)
    {
        /*
         * VOFA 握手
         */
        case DEBUG_CMD_VOFA:
            ESP_LOGI(
                TAG,
                "VOFA protocol connected"
            );
            break;

        /*
         * 速度环
         */
        case DEBUG_CMD_SPEED_KP:

            g_ctrl.speed_pid.kp = cmd->value;
            ESP_LOGI(
                TAG,
                "Set_speed_Kp = %.4f(%.4f)",
                cmd->value,
                g_ctrl.speed_pid.kp
            );

            UDP_LOGI(
                "Set_speed_Kp: %.3f, %.3f\r\n",
                cmd->value,
                g_ctrl.speed_pid.kp
            );
            /*
             * TODO:
             *
             * speed_pid.kp = cmd->value;
             */
            break;

        case DEBUG_CMD_SPEED_KI:

            g_ctrl.speed_pid.ki = cmd->value;
            UDP_LOGI(
                "Set_speed_Ki: %.3f, %.3f\r\n",
                cmd->value,
                g_ctrl.speed_pid.ki
            );

            /*
             * TODO:
             *
             * speed_pid.ki = cmd->value;
             */
            break;

        case DEBUG_CMD_SPEED_KD:
            ESP_LOGI(
                TAG,
                "Set speed Kd = %.6f",
                cmd->value
            );

            /*
             * TODO:
             *
             * speed_pid.kd = cmd->value;
             */
            break;


        /*
         * 位置环
         */
        case DEBUG_CMD_POS_KP:
            ESP_LOGI(
                TAG,
                "Set pos Kp = %.6f",
                cmd->value
            );

            /*
             * TODO:
             *
             * position_pid.kp = cmd->value;
             */
            break;


        case DEBUG_CMD_POS_KI:
            ESP_LOGI(
                TAG,
                "Set pos Ki = %.6f",
                cmd->value
            );

            /*
             * TODO:
             *
             * position_pid.ki = cmd->value;
             */
            break;


        case DEBUG_CMD_POS_KD:
            ESP_LOGI(
                TAG,
                "Set pos Kd = %.6f",
                cmd->value
            );

            /*
             * TODO:
             *
             * position_pid.kd = cmd->value;
             */
            break;


        /*
         * IQ
         */
        case DEBUG_CMD_IQ_KP:
            ESP_LOGI(
                TAG,
                "Set iq Kp = %.6f",
                cmd->value
            );

            /*
             * TODO:
             *
             * iq_pid.kp = cmd->value;
             */
            break;


        case DEBUG_CMD_IQ_KI:
            ESP_LOGI(
                TAG,
                "Set iq Ki = %.6f",
                cmd->value
            );

            /*
             * TODO:
             *
             * iq_pid.ki = cmd->value;
             */
            break;


        case DEBUG_CMD_IQ_KD:
            ESP_LOGI(
                TAG,
                "Set iq Kd = %.6f",
                cmd->value
            );

            /*
             * TODO:
             *
             * iq_pid.kd = cmd->value;
             */
            break;

        /*foc park.id kp*/
        case DEBUG_CMD_ID_KP:
            ESP_LOGI(
                TAG,
                "Set id Kp = %.6f",
                cmd->value
            );

            /*
             * TODO:
             *
             * id_pid.kp = cmd->value;
             */
            break;

        /*foc park.id ki*/
        case DEBUG_CMD_ID_KI:
            ESP_LOGI(
                TAG,
                "Set id Ki = %.6f",
                cmd->value
            );

            /*
             * TODO:
             *
             * id_pid.ki = cmd->value;
             */
            break;

        /*foc park.id kd*/
        case DEBUG_CMD_ID_KD:
            ESP_LOGI(
                TAG,
                "Set id Kd = %.6f",
                cmd->value
            );

            /*
             * TODO:
             *
             * id_pid.kd = cmd->value;
             */
            break;


        /*
         * foc SMO
         */
        case DEBUG_CMD_SMO_KS:
            ESP_LOGI(
                TAG,
                "Set smo Ks = %.6f",
                cmd->value
            );

            /*
             * TODO:
             *
             * smo.k_smo = cmd->value;
             */
            break;


        case DEBUG_CMD_SMO_LPF:
            ESP_LOGI(
                TAG,
                "Set smo Lpf = %.6f",
                cmd->value
            );

            /*
             * TODO:
             *
             * smo.lpf_alpha = cmd->value;
             */
            break;

        /*PLL*/
        case DEBUG_CMD_PLL_KP:
            ESP_LOGI(
                TAG,
                "Set pll Kp = %.6f",
                cmd->value
            );

            /*
             * TODO:
             *
             * pll.kp = cmd->value;
             */
            break;


        case DEBUG_CMD_PLL_KI:
            ESP_LOGI(
                TAG,
                "Set pll Ki = %.6f",
                cmd->value
            );

            /*
             * TODO:
             *
             * pll.ki = cmd->value;
             */
            break;


        /*
         * 速度目标
         */
        case DEBUG_CMD_SPEED_REF:
            ESP_LOGI(
                TAG,
                "Set speed ref = %.6f",
                cmd->value
            );

            /*
             * TODO:
             *
             * speed_target = cmd->value;
             */
            break;


        /*
         * 位置目标
         */
        case DEBUG_CMD_POS_REF:
            ESP_LOGI(
                TAG,
                "Set pos ref = %.6f",
                cmd->value
            );

            /*
             * TODO:
             *
             * position_target = cmd->value;
             */
            break;
            
        case DEBUG_CMD_RUN_OTA:
            ESP_LOGI(
                TAG,
                "run_http_ota_server%.2f",
                cmd->value
            );
            // if (ota_https_task_handle != NULL) {
            //     xTaskNotifyGive(ota_https_task_handle);
            // }
            if (g_wifi_event_group!=NULL)
            {
                /*
                 * 设置开始OTA升级标志
                 */
                xEventGroupSetBits(
                    g_wifi_event_group,
                    RUN_OTA_BIT
                );

            }
            

            /*
             * TODO:
             *
             * position_target = cmd->value;
             */
            break;

        default:
            ESP_LOGW(
                TAG,
                "Unknown command"
            );
            break;
    }
}