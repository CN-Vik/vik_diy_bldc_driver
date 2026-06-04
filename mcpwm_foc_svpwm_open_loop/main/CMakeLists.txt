# 自动获取 main/ 下所有 .c 文件
file(GLOB MAIN_SOURCES "*.c")

# 自动获取 svpwm/ 下所有 .c 文件
file(GLOB SVPWM_SOURCES "svpwm/*.c")

# 自动获取 vik_foc/ 下所有 .c 文件
file(GLOB VIK_FOC_SOURCES "vik_foc/*.c")

file(GLOB AS5600_SOURCES "as5600/*.c")

# 合并所有源文件
set(COMPONENT_SRCS
    ${MAIN_SOURCES}
    ${SVPWM_SOURCES}
    ${VIK_FOC_SOURCES}
    ${AS5600_SOURCES}
)

# 注册组件（正确版本）
idf_component_register(
    SRCS ${COMPONENT_SRCS}
    INCLUDE_DIRS "." "svpwm" "vik_foc"
    PRIV_REQUIRES esp_driver_mcpwm 
                  esp_driver_gpio 
                  esp_driver_gptimer
                  esp_timer
                  esp_driver_i2c 
                   
)