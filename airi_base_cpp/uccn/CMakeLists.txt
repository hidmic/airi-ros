cmake_minimum_required(VERSION 3.5)
project(airi-uccn)

add_subdirectory(uccn)

add_library(${PROJECT_NAME}
  src/drive_state_typesupport.c
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Werror)

target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(${PROJECT_NAME} uccn mpack)
