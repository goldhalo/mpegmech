# =============================================================================
#  mpegmech —— PS/TS 容器封装与解封装库（C++17，零外部依赖）
#
#  构建：
#      cmake -S . -B build -G "Visual Studio 16 2019" -A x64
#      cmake --build build --config Release
#      ./build/bin/mpegmech_ut
#
#  交叉编译（板端 aarch64）：
#      cmake -S . -B build-arm -DCMAKE_TOOLCHAIN_FILE=<toolchain>.cmake
#      cmake --build build-arm -j2
#
#  依赖：仅 C++17 标准库。
# =============================================================================

cmake_minimum_required(VERSION 3.16)

project(mpegmech
        VERSION 0.1.0
        DESCRIPTION "MPEG PS/TS mux and demux with arbitrary-chunk input"
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# 源码是 UTF-8（含中文注释）。cl 默认按系统代码页（GBK）读源码，
# UTF-8 里某些汉字的末字节正好是 0x5C（反斜杠），会被当成续行符把下一行吞掉，
# 报出一堆莫名其妙的语法错误。全局加 /utf-8，库和测试都受益。
if(MSVC)
    add_compile_options(/utf-8)
    # fopen / getenv 在 MSVC 上默认触发 C4996（建议换 fopen_s / _dupenv_s）。
    # 测试与示例大量用到，换掉只会让代码更啰嗦，跨平台也更麻烦。
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

set(MPEGMECH_OUTPUT_DIR "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY         "${MPEGMECH_OUTPUT_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${MPEGMECH_OUTPUT_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${MPEGMECH_OUTPUT_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY         "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY         "${CMAKE_BINARY_DIR}/lib")

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(MPEGMECH_TOP_LEVEL ON)
else()
    set(MPEGMECH_TOP_LEVEL OFF)
endif()

# ---------------------------------------------------------------------------
# 1. 选项
# ---------------------------------------------------------------------------
option(MPEGMECH_BUILD_TESTS      "构建单元测试" ON)
option(MPEGMECH_BUILD_EXAMPLES   "构建示例程序" OFF)
option(MPEGMECH_WARNINGS_AS_ERRORS "把编译告警当错误（CI 用）" OFF)

# 位置无关代码。
#
# 本库只出静态库，但静态库常常被链进 .so。缺 -fPIC 时，编静态库、跑单测都
# 正常，**只在最终链 .so 那一步**报：
#     relocation R_X86_64_PC32 against symbol `...' can not be used when
#     making a shared object; recompile with -fPIC
# 纯静态场景可 -DMPEGMECH_PIC=OFF。
option(MPEGMECH_PIC "编译为位置无关代码（静态库要链进 .so 时必须开）" ON)

# ---------------------------------------------------------------------------
# 2. 库目标
# ---------------------------------------------------------------------------
file(GLOB_RECURSE MPEGMECH_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB_RECURSE MPEGMECH_HEADERS CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/include/mpegmech/*.hpp")

add_library(mpegmech ${MPEGMECH_SOURCES} ${MPEGMECH_HEADERS})

set_target_properties(mpegmech PROPERTIES
    POSITION_INDEPENDENT_CODE ${MPEGMECH_PIC})

target_include_directories(mpegmech
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(mpegmech PUBLIC cxx_std_17)

# 导出宏：静态库不需要，动态库在 Windows 上需要 dllimport/dllexport。
# v1 只构建静态库形态（板端静态链更省心），保留变量便于后续切换。
target_compile_definitions(mpegmech PRIVATE MPEGMECH_BUILDING)

if(MSVC)
    target_compile_options(mpegmech PRIVATE /W4 /permissive-)
    if(MPEGMECH_WARNINGS_AS_ERRORS)
        target_compile_options(mpegmech PRIVATE /WX)
    endif()
else()
    target_compile_options(mpegmech PRIVATE -Wall -Wextra -Wpedantic)
    if(MPEGMECH_WARNINGS_AS_ERRORS)
        target_compile_options(mpegmech PRIVATE -Werror)
    endif()
endif()

# ---------------------------------------------------------------------------
# 2.5 示例 / 工具
# ---------------------------------------------------------------------------
if(MPEGMECH_BUILD_EXAMPLES)
    file(GLOB MPEGMECH_EXAMPLE_SOURCES CONFIGURE_DEPENDS
         "${CMAKE_CURRENT_SOURCE_DIR}/example/*.cpp")

    foreach(ex_src ${MPEGMECH_EXAMPLE_SOURCES})
        get_filename_component(ex_name ${ex_src} NAME_WE)
        add_executable(${ex_name} ${ex_src})
        target_link_libraries(${ex_name} PRIVATE mpegmech)
    endforeach()
endif()

# ---------------------------------------------------------------------------
# 3. 测试
# ---------------------------------------------------------------------------
if(MPEGMECH_BUILD_TESTS)
    enable_testing()
    file(GLOB MPEGMECH_TEST_SOURCES CONFIGURE_DEPENDS
         "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp")

    # 每个测试文件都有自己的 main()（独立可执行），不合并成一个。
    # 合并会触发 "multiple definition of main"。
    foreach(test_src ${MPEGMECH_TEST_SOURCES})
        get_filename_component(test_name ${test_src} NAME_WE)
        add_executable(${test_name} ${test_src})
        target_link_libraries(${test_name} PRIVATE mpegmech)
        # 少数测试要手工拼容器（构造异常/多节目流），用得到 src 里的内部头
        target_include_directories(${test_name} PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/src")
        add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()

    if(FALSE)
        target_include_directories(mpegmech_ut PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/test")
        add_test(NAME mpegmech_ut COMMAND mpegmech_ut)
    endif()
endif()

# ---------------------------------------------------------------------------
# 4. 安装
# ---------------------------------------------------------------------------
include(GNUInstallDirs)

install(TARGETS mpegmech
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY include/mpegmech
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.hpp")
