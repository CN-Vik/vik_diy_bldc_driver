#ifndef DEBUG_PROTOCOL_H
#define DEBUG_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define DEBUG_PROTOCOL_LINE_MAX     128


typedef enum
{
    DEBUG_CMD_NONE = 0,

    DEBUG_CMD_VOFA,

    /* 速度环 */
    DEBUG_CMD_SPEED_KP,
    DEBUG_CMD_SPEED_KI,
    DEBUG_CMD_SPEED_KD,

    /* 位置环 */
    DEBUG_CMD_POS_KP,
    DEBUG_CMD_POS_KI,
    DEBUG_CMD_POS_KD,

    /* IQ 电流环 */
    DEBUG_CMD_IQ_KP,
    DEBUG_CMD_IQ_KI,
    DEBUG_CMD_IQ_KD,

    /* ID 电流环 */
    DEBUG_CMD_ID_KP,
    DEBUG_CMD_ID_KI,
    DEBUG_CMD_ID_KD,

    /* SMO */
    DEBUG_CMD_SMO_KS,
    DEBUG_CMD_SMO_LPF,

    /* PLL */
    DEBUG_CMD_PLL_KP,
    DEBUG_CMD_PLL_KI,

    /* 电机 */
    DEBUG_CMD_SPEED_REF,
    DEBUG_CMD_POS_REF,


    DEBUG_CMD_RUN_OTA,

    DEBUG_CMD_MAX

} debug_cmd_id_t;


typedef struct
{
    debug_cmd_id_t id;

    char name[32];
    float value;

} debug_cmd_t;


typedef struct
{
    const char *name;
    debug_cmd_id_t id;

} debug_cmd_map_t;




extern debug_cmd_t tcp_cmd_data_v;


inline const char* debug_cmd_to_string(debug_cmd_id_t index);

void debug_cmd_proces(char *parm_cmd,debug_cmd_t *cmd);





#endif