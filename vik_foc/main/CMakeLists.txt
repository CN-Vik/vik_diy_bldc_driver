# ============================================================
# 头文件目录变量管理
# ============================================================

set(INC_MAIN                  ".")
set(INC_VIK_FOC               "vik_foc")
set(INC_APP_RTOS              "app_rtos")
set(INC_MOTOR_CURRENT         "motor_current")
set(INC_AS5600                "as5600")
set(INC_BSP                   "BSP")
set(INC_MOTOR_POWER           "motor_power")
set(INC_SIX_STEP_CTRL_MOTOR   "six_step_ctrl_motor")

# CMake 自动生成的头文件目录
set(INC_GENERATED
    "${CMAKE_BINARY_DIR}/generated"
)

# ============================================================
# 合并所有头文件目录
# ============================================================

set(COMPONENT_PRIV_INC_DIRS
    ${INC_MAIN}
    ${INC_VIK_FOC}
    ${INC_APP_RTOS}
    ${INC_MOTOR_CURRENT}
    ${INC_AS5600}
    ${INC_MOTOR_POWER}
    ${INC_SIX_STEP_CTRL_MOTOR}
    ${INC_BSP}
    ${INC_GENERATED}
)

# ============================================================
# 源文件
# ============================================================

# main/*.c
file(GLOB MAIN_SOURCES
    "*.c"
)

# app_rtos/*.c
file(GLOB APP_RTOS_SOURCES
    "app_rtos/*.c"
)

# motor_power/*.c
file(GLOB MOTOR_POWER_SOURCES
    "motor_power/*.c"
)

# motor_current/*.c
file(GLOB MOTOR_CURRENT_SOURCES
    "motor_current/*.c"
)

# vik_foc/*.c
file(GLOB VIK_FOC_SOURCES
    "vik_foc/*.c"
)

# as5600/*.c
file(GLOB AS5600_SOURCES
    "as5600/*.c"
)

# six_step_ctrl_motor/*.c
file(GLOB SIX_STEP_CTRL_MOTOR_SOURCES
    "six_step_ctrl_motor/*.c"
)

# ============================================================
# 合并所有源文件
# ============================================================

set(COMPONENT_SRCS
    ${MAIN_SOURCES}
    ${APP_RTOS_SOURCES}
    ${MOTOR_CURRENT_SOURCES}
    ${VIK_FOC_SOURCES}
    ${AS5600_SOURCES}
    ${MOTOR_POWER_SOURCES}
    ${SIX_STEP_CTRL_MOTOR_SOURCES}
)

# ============================================================
# 注册组件
# ============================================================

idf_component_register(
    SRCS
        ${COMPONENT_SRCS}

    PRIV_INCLUDE_DIRS
        ${COMPONENT_PRIV_INC_DIRS}

    PRIV_REQUIRES
        esp_driver_mcpwm
        esp_driver_gpio
        esp_driver_gptimer
        esp_timer
        esp_driver_i2c
        esp_adc
        nvs_flash
        wifi_network
        esp_app_format
)