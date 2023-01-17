idf_component_register(SRCS "pppos_client.c"
                        INCLUDE_DIRS "include"
                        REQUIRES driver lwip esp_netif)