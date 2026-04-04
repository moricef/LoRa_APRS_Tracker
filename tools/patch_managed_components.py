"""Pre-build script: patch managed components that have bugs.

esp_insights CMakeLists.txt embeds server certs unconditionally,
even when CONFIG_ESP_INSIGHTS_ENABLED=n. This wraps the cert
embedding in an if(CONFIG_ESP_INSIGHTS_ENABLED) guard.
"""
import os

Import("env")


def patch_esp_insights():
    project_dir = env.subst("$PROJECT_DIR")
    cmake_path = os.path.join(
        project_dir, "managed_components", "espressif__esp_insights", "CMakeLists.txt"
    )

    if not os.path.isfile(cmake_path):
        return

    with open(cmake_path, "r") as f:
        content = f.read()

    # Already patched?
    if "if(CONFIG_ESP_INSIGHTS_ENABLED)" in content and "target_add_binary_data" in content:
        lines = content.splitlines()
        for i, line in enumerate(lines):
            if "target_add_binary_data" in line:
                for j in range(max(0, i - 3), i):
                    if "if(CONFIG_ESP_INSIGHTS_ENABLED)" in lines[j]:
                        return
                break

    old = """if(CONFIG_ESP_INSIGHTS_TRANSPORT_MQTT)
    target_add_binary_data(${COMPONENT_TARGET} "server_certs/mqtt_server.crt" TEXT)
    target_sources(${COMPONENT_LIB} PRIVATE "src/transport/esp_insights_mqtt.c")
else()
    target_add_binary_data(${COMPONENT_TARGET} "server_certs/https_server.crt" TEXT)
    idf_component_get_property(http_client_lib esp_http_client COMPONENT_LIB)
    target_link_libraries(${COMPONENT_LIB} PRIVATE ${http_client_lib})
    target_sources(${COMPONENT_LIB} PRIVATE "src/transport/esp_insights_https.c")
endif()"""

    new = """if(CONFIG_ESP_INSIGHTS_ENABLED)
if(CONFIG_ESP_INSIGHTS_TRANSPORT_MQTT)
    target_add_binary_data(${COMPONENT_TARGET} "server_certs/mqtt_server.crt" TEXT)
    target_sources(${COMPONENT_LIB} PRIVATE "src/transport/esp_insights_mqtt.c")
else()
    target_add_binary_data(${COMPONENT_TARGET} "server_certs/https_server.crt" TEXT)
    idf_component_get_property(http_client_lib esp_http_client COMPONENT_LIB)
    target_link_libraries(${COMPONENT_LIB} PRIVATE ${http_client_lib})
    target_sources(${COMPONENT_LIB} PRIVATE "src/transport/esp_insights_https.c")
endif()
endif()"""

    if old in content:
        content = content.replace(old, new)
        with open(cmake_path, "w") as f:
            f.write(content)
        print("  [patch] esp_insights CMakeLists.txt: wrapped cert embedding in CONFIG_ESP_INSIGHTS_ENABLED guard")


patch_esp_insights()
