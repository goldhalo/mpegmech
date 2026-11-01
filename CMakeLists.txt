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

# ---------------------------------------------------------------------------
# 2. 库目标
# ---------------------------------------------------------------------------
file(GLOB_RECURSE MPEGMECH_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB_RECURSE MPEGMECH_HEADERS CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/include/mpegmech/*.hpp")

add_library(mpegmech ${MPEGMECH_SOURCES} ${MPEGMECH_HEADERS})

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
