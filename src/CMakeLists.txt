# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(SRCS ${app_sources})

# Embed web files
target_add_binary_data(${COMPONENT_TARGET} "${CMAKE_SOURCE_DIR}/data_embed/index.html.gz" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "${CMAKE_SOURCE_DIR}/data_embed/style.css.gz" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "${CMAKE_SOURCE_DIR}/data_embed/script.js.gz" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "${CMAKE_SOURCE_DIR}/data_embed/bootstrap.css.gz" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "${CMAKE_SOURCE_DIR}/data_embed/bootstrap.js.gz" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "${CMAKE_SOURCE_DIR}/data_embed/favicon.png.gz" TEXT)
