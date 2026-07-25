cmake_minimum_required(VERSION 3.21)

project(
    LEparagliding
    VERSION 3.28.0
    DESCRIPTION "Qt C++ port of LEparagliding 3.28"
    LANGUAGES C CXX
)

set(CMAKE_C_STANDARD 90)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(WIN32 AND "${CMAKE_PREFIX_PATH}" STREQUAL "")
    file(GLOB qt_candidates LIST_DIRECTORIES true "C:/Qt/6.*/msvc2022_64")
    list(SORT qt_candidates COMPARE NATURAL ORDER DESCENDING)
    foreach(qt_candidate IN LISTS qt_candidates)
        if(EXISTS "${qt_candidate}/lib/cmake/Qt6/Qt6Config.cmake")
            set(CMAKE_PREFIX_PATH "${qt_candidate}" CACHE PATH "Qt installation" FORCE)
            message(STATUS "Auto-detected Qt at ${CMAKE_PREFIX_PATH}")
            break()
        endif()
    endforeach()
endif()

find_package(Qt6 6.5 REQUIRED COMPONENTS Widgets)

file(GLOB F2C_RUNTIME_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/libf2c/*.c"
)

add_library(f2c_runtime STATIC ${F2C_RUNTIME_SOURCES})
target_include_directories(f2c_runtime PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/libf2c"
)
target_compile_definitions(f2c_runtime PRIVATE
    SIGNED_ZEROS
    WANT_LEAD_0
)
if(WIN32)
    target_compile_definitions(f2c_runtime PRIVATE
        USE_CLOCK
        MSDOS
        NO_ONEXIT
        NO_My_ctype
        NO_ISATTY
        _CRT_SECURE_NO_WARNINGS
    )
    target_compile_options(f2c_runtime PRIVATE /W0)
else()
    target_compile_options(f2c_runtime PRIVATE -w)
endif()

add_executable(leparagliding-engine
    src/engine/engine_paths.cpp
    src/engine/engine_paths.h
    src/engine/input_migration.cpp
    src/engine/input_migration.h
    src/engine/main.cpp
    src/legacy/leparagliding_core.cpp
)
target_include_directories(leparagliding-engine PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/engine"
)
target_link_libraries(leparagliding-engine PRIVATE f2c_runtime)
if(MSVC)
    set_source_files_properties(src/legacy/leparagliding_core.cpp PROPERTIES
        COMPILE_OPTIONS "/bigobj;/W0;/Zc:strictStrings-"
    )
endif()

add_executable(reference-report-compare
    tests/reference_report_compare.cpp
)

qt_add_executable(LEparagliding WIN32
    src/gui/design_document.cpp
    src/gui/design_document.h
    src/gui/main.cpp
    src/gui/mainwindow.cpp
    src/gui/mainwindow.h
    src/gui/paraglider_view.cpp
    src/gui/paraglider_view.h
    src/gui/section_help.cpp
    src/gui/section_help.h
)
target_link_libraries(LEparagliding PRIVATE Qt6::Widgets)
add_dependencies(LEparagliding leparagliding-engine)

set_target_properties(
    LEparagliding
    leparagliding-engine
    reference-report-compare
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
foreach(configuration Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER "${configuration}" configuration_upper)
    set_target_properties(
        LEparagliding
        leparagliding-engine
        reference-report-compare
        PROPERTIES
        "RUNTIME_OUTPUT_DIRECTORY_${configuration_upper}" "${CMAKE_BINARY_DIR}/bin/${configuration}"
    )
endforeach()

if(WIN32)
    get_target_property(qmake_executable Qt6::qmake IMPORTED_LOCATION)
    get_filename_component(qt_bin_directory "${qmake_executable}" DIRECTORY)
    find_program(WINDEPLOYQT_EXECUTABLE windeployqt
        HINTS "${qt_bin_directory}"
        NO_CACHE
    )
    if(WINDEPLOYQT_EXECUTABLE)
        add_custom_command(TARGET LEparagliding POST_BUILD
            COMMAND "${WINDEPLOYQT_EXECUTABLE}"
                --no-translations
                --no-system-d3d-compiler
                "$<TARGET_FILE:LEparagliding>"
            COMMENT "Deploying Qt runtime"
        )
    endif()
endif()

enable_testing()
add_test(NAME engine_help COMMAND leparagliding-engine --help)
add_test(NAME engine_rejects_missing_input
    COMMAND leparagliding-engine
        "${CMAKE_BINARY_DIR}/does-not-exist.txt"
        "${CMAKE_BINARY_DIR}/missing-input-output"
)
set_tests_properties(engine_rejects_missing_input PROPERTIES WILL_FAIL TRUE)
add_test(NAME gui_smoke COMMAND LEparagliding --smoke-test)
set_tests_properties(gui_smoke PROPERTIES
    TIMEOUT 20
)
add_test(NAME studio_model_smoke
    COMMAND LEparagliding
        --studio-self-test
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/3.28/leparagliding.txt"
        "${CMAKE_CURRENT_SOURCE_DIR}/Data/fortranResult/lep-3d.dxf"
)
set_tests_properties(studio_model_smoke PROPERTIES
    TIMEOUT 20
)
add_test(NAME gui_headless_help COMMAND LEparagliding --headless --help)
set_tests_properties(gui_headless_help PROPERTIES
    TIMEOUT 20
)

set(reference_input
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/3.28/leparagliding.txt")
set(reference_directory
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/reference/3.28")
if(WIN32
   AND EXISTS "${reference_input}"
   AND EXISTS "${reference_directory}/lep-out.txt"
   AND EXISTS "${reference_directory}/lines.txt"
   AND EXISTS "${reference_directory}/run-log.txt")
    add_test(NAME engine_matches_fortran_reference
        COMMAND "${CMAKE_COMMAND}"
            "-DENGINE=$<TARGET_FILE:leparagliding-engine>"
            "-DREPORT_COMPARATOR=$<TARGET_FILE:reference-report-compare>"
            "-DINPUT=${reference_input}"
            "-DOUTPUT_DIR=${CMAKE_BINARY_DIR}/reference-test-output"
            "-DREFERENCE_DIR=${reference_directory}"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CompareReferenceRun.cmake"
    )
    set_tests_properties(engine_matches_fortran_reference PROPERTIES
        TIMEOUT 90
    )
endif()

install(TARGETS LEparagliding leparagliding-engine
    RUNTIME DESTINATION .
)
