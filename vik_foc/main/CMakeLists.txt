# ====================== 头文件目录变量管理 ======================
# 逐个模块定义头文件目录
set(INC_MAIN          ".")
set(INC_VIK_FOC       "vik_foc")
set(INC_APP_RTOS      "app_rtos")
set(INC_MOTOR_CURRENT "motor_current")
set(INC_AS5600        "as5600")
set(INC_BSP        "BSP")
set(INC_MOTOR_POWER   "motor_power")
set(INC_SIX_STEP_CTRL_MOTOR   "six_step_ctrl_motor")


# 合并所有头文件
# 统一汇总私有头文件路径（本组件内使用）
set(COMPONENT_PRIV_INC_DIRS
    ${INC_MAIN}
    ${INC_VIK_FOC}
    ${INC_APP_RTOS}
    ${INC_MOTOR_CURRENT}
    ${INC_AS5600}
    ${INC_MOTOR_POWER}
    ${INC_BSP}
    ${INC_SIX_STEP_CTRL_MOTOR}

)


# ====================== 源文件变量管理 ======================
# 自动获取 main/ 下所有 .c 文件
file(GLOB MAIN_SOURCES "*.c")

# 自动获取 app_rtos/ 下所有 .c 文件
file(GLOB APP_RTOS_SOURCES "app_rtos/*.c")

# 自动获取 motor_power/ 下所有 .c 文件
file(GLOB MOTOR_POWER_SOURCES "motor_power/*.c")


# 自动获取 motor_current/ 下所有 .c 文件
file(GLOB MOTOR_CUREENT_SOURCES "motor_current/*.c")

# 自动获取 vik_foc/ 下所有 .c 文件
file(GLOB VIK_FOC_SOURCES "vik_foc/*.c")

file(GLOB AS5600_SOURCES "as5600/*.c")

file(GLOB SIX_STEP_CTRL_MOTOR_SOURCES "six_step_ctrl_motor/*.c")

# 合并所有源文件
set(COMPONENT_SRCS
    ${MAIN_SOURCES}
    ${APP_RTOS_SOURCES}
    ${MOTOR_CUREENT_SOURCES}
    ${VIK_FOC_SOURCES}
    ${AS5600_SOURCES}
    ${MOTOR_POWER_SOURCES}
    ${SIX_STEP_CTRL_MOTOR_SOURCES}
)


# 注册组件（正确版本）
idf_component_register(

    # 源文件集合
    SRCS ${COMPONENT_SRCS}
    
    # 头文件集合
    PRIV_INCLUDE_DIRS ${COMPONENT_PRIV_INC_DIRS}
    
    PRIV_REQUIRES esp_driver_mcpwm 
                  esp_driver_gpio 
                  esp_driver_gptimer
                  esp_timer
                  esp_driver_i2c  
                  esp_adc 
                  nvs_flash
                   
)