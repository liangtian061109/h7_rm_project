cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME rm_project)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    # 一般不用手填：下面的 AutoSources 插件会自动收集 APP / BSP / modules 下的 .c
)

# -----------------------------------------------------------------------------
# 自动收集用户代码（插件）：
#   递归扫描 APP / BSP / modules 下所有 .c 加入编译，所有 .h 所在目录加入 include。
#   新增/删除 .c/.h 后不用改本文件（CONFIGURE_DEPENDS 会让 Ninja 下次构建时重跑 cmake）。
# -----------------------------------------------------------------------------
include(${CMAKE_SOURCE_DIR}/cmake/AutoSources.cmake)
auto_add_sources(${CMAKE_PROJECT_NAME} APP BSP modules)


# -----------------------------------------------------------------------------
# 用工程自有链接脚本替换 CubeMX 生成脚本（插件）：
#   STM32H723XG_Project.ld = 生成脚本 + .dma_buffer 段（RAM_D1，供 DMA 访问）
#   生成脚本 STM32H723XG_FLASH.ld 每次 CubeMX 生成代码都会被覆盖，不能直接改。
# -----------------------------------------------------------------------------
include(${CMAKE_SOURCE_DIR}/cmake/LinkerScript.cmake)
override_linker_script(${CMAKE_PROJECT_NAME}
                       "${CMAKE_SOURCE_DIR}/STM32H723XG_Project.ld")

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
