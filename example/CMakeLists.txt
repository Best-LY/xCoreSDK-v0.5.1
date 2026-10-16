#-------------------------------------------------------------------------------
# Auto-scanned example executables setup

project(xCoreSDK-examples CXX)

add_compile_definitions(
    # for M_PI in cmath
    $<$<CXX_COMPILER_ID:MSVC>:_USE_MATH_DEFINES>
    # if linking the shared library, add a macro for MSVC dllimport attribute
    $<$<BOOL:${XCORE_LINK_SHARED_LIBS}>:XCORESDK_DLL>)

set(ORIGINAL_CASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/original_cases")
set(FUNCTION_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/function_config")
set(TASK_LIBRARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/task_library")
set(TEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Test")

set(EXAMPLE_HEADERS
    ${FUNCTION_CONFIG_DIR}/print_helper.hpp
    ${FUNCTION_CONFIG_DIR}/robot_vision_config.hpp
    ${FUNCTION_CONFIG_DIR}/visual_servo_config.hpp
)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

if(DEFINED ENV{HOME} AND EXISTS "$ENV{HOME}/.local/xcore-vision")
    list(PREPEND CMAKE_PREFIX_PATH "$ENV{HOME}/.local/xcore-vision")
endif()

find_package(OpenCV QUIET COMPONENTS core imgproc imgcodecs calib3d highgui videoio)
find_package(realsense2 QUIET)

set(BUILT_EXAMPLE_TARGETS)

function(xcore_add_scanned_example source_file)
    get_filename_component(example_name "${source_file}" NAME_WE)

    string(REGEX REPLACE "[^A-Za-z0-9_]+" "_" target_name "${example_name}")
    string(REGEX REPLACE "_+$" "" target_name "${target_name}")
    string(REGEX REPLACE "^_+" "" target_name "${target_name}")

    if(target_name STREQUAL "")
        file(RELATIVE_PATH relative_source "${CMAKE_CURRENT_SOURCE_DIR}" "${source_file}")
        string(MD5 source_hash "${relative_source}")
        string(SUBSTRING "${source_hash}" 0 8 source_hash_short)
        set(target_name "example_${source_hash_short}")
    endif()

    if(TARGET ${target_name})
        message(FATAL_ERROR "Duplicate executable cpp name: ${example_name}. Please rename one of the files.")
    endif()

    file(READ "${source_file}" source_content)

    if(NOT source_content MATCHES "(^|[\r\n])[ \t]*(int|auto)[ \t\r\n]+main[ \t\r\n]*\\(")
        message(STATUS "Skipping ${target_name}: no main() entry point")
        return()
    endif()

    set(needs_model OFF)
    if(source_content MATCHES "XMATEMODEL_LIB_SUPPORTED|FollowPosition|TorqueType|xMateModel[ \t]*<")
        set(needs_model ON)
    endif()

    if(needs_model AND NOT XCORE_USE_XMATE_MODEL)
        message(STATUS "Skipping ${target_name}: set XCORE_USE_XMATE_MODEL=ON to build it")
        return()
    endif()

    set(needs_vision OFF)
    if(source_content MATCHES "opencv2/|librealsense2/")
        set(needs_vision ON)
    endif()

    if(needs_vision AND NOT (OpenCV_FOUND AND realsense2_FOUND))
        message(STATUS "Skipping ${target_name}: OpenCV or librealsense2 not found")
        return()
    endif()

    add_executable(${target_name} ${source_file} ${EXAMPLE_HEADERS})
    set_target_properties(${target_name} PROPERTIES OUTPUT_NAME "${example_name}")
    target_include_directories(${target_name} PUBLIC
        ${SHARED_INCLUDEDIR}
        ${FUNCTION_CONFIG_DIR}
        ${TASK_LIBRARY_DIR}
    )
    target_link_libraries(${target_name}
        Rokae::Rokae
        eigen
        Threads::Threads
    )

    if(needs_model)
        target_compile_definitions(${target_name} PUBLIC XMATEMODEL_LIB_SUPPORTED)
    endif()

    if(needs_vision)
        target_include_directories(${target_name} PUBLIC ${OpenCV_INCLUDE_DIRS})
        target_link_libraries(${target_name}
            ${OpenCV_LIBS}
            realsense2::realsense2
        )
    endif()

    list(APPEND BUILT_EXAMPLE_TARGETS ${target_name})
    set(BUILT_EXAMPLE_TARGETS "${BUILT_EXAMPLE_TARGETS}" PARENT_SCOPE)
endfunction()

file(GLOB_RECURSE SCANNED_EXAMPLE_SOURCES CONFIGURE_DEPENDS
    "${ORIGINAL_CASE_DIR}/*.cpp"
    "${FUNCTION_CONFIG_DIR}/*.cpp"
    "${TASK_LIBRARY_DIR}/*.cpp"
    "${TEST_DIR}/*.cpp"
)

list(SORT SCANNED_EXAMPLE_SOURCES)

foreach(source_file ${SCANNED_EXAMPLE_SOURCES})
    xcore_add_scanned_example(${source_file})
endforeach()

#-------------------------------------------------------------------------------
# Installation
if(BUILT_EXAMPLE_TARGETS)
    install(TARGETS ${BUILT_EXAMPLE_TARGETS}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        OPTIONAL
    )
endif()
