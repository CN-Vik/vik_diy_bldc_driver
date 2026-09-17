file(GLOB SOURCES 
    ${CMAKE_CURRENT_LIST_DIR}/*.c 
)

idf_component_register(

    SRCS ${SOURCES}

    INCLUDE_DIRS "."

    EMBED_TXTFILES
        ${CMAKE_CURRENT_LIST_DIR}/server_certs/ca_cert.pem

    REQUIRES
        nvs_flash
        esp_wifi
        esp_event
        esp_netif
        esp_http_client
        esp_https_ota
        app_update
)