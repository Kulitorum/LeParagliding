cmake_minimum_required(VERSION 3.21)

# Ship binaries that run several macOS releases back rather than only on the
# build host: without an explicit deployment target the minimum OS defaults to
# the CI runner's version (macOS 15), so the app refuses to launch on older
# systems. Must be set before project(). The CI release build additionally
# passes -DCMAKE_OSX_ARCHITECTURES="arm64;x86_64" for a universal binary.
if(APPLE AND NOT DEFINED CMAKE_OSX_DEPLOYMENT_TARGET)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "13.0" CACHE STRING
        "Minimum macOS version")
endif()

project(
    LEparagliding
    VERSION 0.4.1
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

find_package(Qt6 6.5 REQUIRED COMPONENTS Widgets OpenGL OpenGLWidgets Xml)

# Open CASCADE Technology is the authoritative 3D kernel for model creation,
# STEP serialization, tessellation, and viewport rendering. The Windows
# default mirrors C:\CODE\cobod-slicer and can be overridden for another OCCT
# build with -DLEP_OCCT_ROOT=<path>.
set(LEP_OCCT_ROOT "" CACHE PATH "Open CASCADE build or installation root")
if(WIN32)
    if(NOT LEP_OCCT_ROOT)
        foreach(occt_candidate
                "C:/OpenCASCADE-8.0/build2"
                "C:/OpenCASCADE-8.0/build"
                "C:/OpenCASCADE-7.9.0-vc14-64/opencascade-7.9.0")
            if(EXISTS "${occt_candidate}/inc/Standard.hxx")
                set(LEP_OCCT_ROOT "${occt_candidate}" CACHE PATH
                    "Open CASCADE build or installation root" FORCE)
                break()
            endif()
        endforeach()
    endif()
    if(EXISTS "${LEP_OCCT_ROOT}/inc/Standard.hxx")
        # Classic OCCT development layout (local builds under C:/OpenCASCADE).
        set(LEP_OCCT_INCLUDE_DIR "${LEP_OCCT_ROOT}/inc")
        set(LEP_OCCT_LIBRARY_DIR "${LEP_OCCT_ROOT}/win64/vc14/lib")
        set(LEP_OCCT_BINARY_DIR "${LEP_OCCT_ROOT}/win64/vc14/bin")
    else()
        # OCCT installed with INSTALL_DIR_LAYOUT=Unix (the CI build).
        set(LEP_OCCT_INCLUDE_DIR "${LEP_OCCT_ROOT}/include/opencascade")
        set(LEP_OCCT_LIBRARY_DIR "${LEP_OCCT_ROOT}/lib")
        set(LEP_OCCT_BINARY_DIR "${LEP_OCCT_ROOT}/bin")
    endif()
else()
    find_package(OpenCASCADE REQUIRED)
    set(LEP_OCCT_INCLUDE_DIR "${OpenCASCADE_INCLUDE_DIR}")
    set(LEP_OCCT_LIBRARY_DIR "${OpenCASCADE_LIBRARY_DIR}")
    set(LEP_OCCT_BINARY_DIR "${OpenCASCADE_BINARY_DIR}")
endif()

if(NOT EXISTS "${LEP_OCCT_INCLUDE_DIR}/Standard.hxx")
    message(FATAL_ERROR
        "Open CASCADE headers were not found. Set LEP_OCCT_ROOT to an OCCT "
        "build/install root (current value: '${LEP_OCCT_ROOT}').")
endif()

set(LEP_OCCT_TOOLKITS
    TKernel
    TKMath
    TKG2d
    TKG3d
    TKGeomBase
    TKBRep
    TKGeomAlgo
    TKTopAlgo
    TKMesh
    TKService
    TKV3d
    TKOpenGl
    TKDE
    TKXSBase
    TKDESTEP
    TKCDF
    TKLCAF
    TKCAF
    TKXCAF
    TKBinL
    TKBin
    TKBinXCAF
)

add_library(lep_occt INTERFACE)
target_include_directories(lep_occt SYSTEM BEFORE INTERFACE
    "${LEP_OCCT_INCLUDE_DIR}"
)
target_compile_definitions(lep_occt INTERFACE
    OCCT_NO_DEBUG
    OCCT_NO_DEPRECATED
)
foreach(occt_toolkit IN LISTS LEP_OCCT_TOOLKITS)
    if(WIN32)
        set(occt_library
            "${LEP_OCCT_LIBRARY_DIR}/${occt_toolkit}.lib")
    else()
        # find_library skips the search when the result variable is already
        # set, so clear it or every toolkit after the first reuses TKernel.
        unset(occt_library)
        find_library(occt_library
            NAMES "${occt_toolkit}"
            HINTS "${LEP_OCCT_LIBRARY_DIR}"
            REQUIRED
            NO_CACHE
        )
    endif()
    if(NOT EXISTS "${occt_library}")
        message(FATAL_ERROR
            "Required Open CASCADE toolkit not found: ${occt_library}")
    endif()
    target_link_libraries(lep_occt INTERFACE "${occt_library}")
endforeach()
message(STATUS "Using Open CASCADE at ${LEP_OCCT_ROOT}")

if(NOT WIN32)
    # OCCT usually lives in a custom prefix (see README); bake its library
    # directory into the build-tree rpath so binaries and tests run directly.
    list(APPEND CMAKE_BUILD_RPATH "${LEP_OCCT_LIBRARY_DIR}")
endif()

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
    if(UNIX)
        target_link_libraries(f2c_runtime PUBLIC m)
    endif()
endif()

# Vendored SoftWingLab XPBD physics core (src/softwing, see its README.md
# for provenance): dependency-free soft-body/membrane/pneumatics/suspension
# simulation used by the Playground tab's toy live-wing model.
add_library(softwing_core STATIC
    src/softwing/canopy.cpp
    src/softwing/cell_volume.cpp
    src/softwing/contact.cpp
    src/softwing/membrane.cpp
    src/softwing/parallel.cpp
    src/softwing/pneumatics.cpp
    src/softwing/soft_body.cpp
    src/softwing/suspension.cpp
)
target_include_directories(softwing_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/softwing/include"
)
if(MSVC)
    target_compile_options(softwing_core PRIVATE /W0 /bigobj)
else()
    target_compile_options(softwing_core PRIVATE -w)
endif()

# The Playground's wing: mesh parsing, refinement, SoftBody assembly and the
# per-frame step, with no widget attached. A library rather than GUI sources
# so the headless solver benchmark drives the identical simulation.
add_library(playground_pressure STATIC
    src/gui/playground_pressure_solve.cpp
    src/gui/playground_pressure_solve.h
)
target_include_directories(playground_pressure PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui)

add_library(playground_cell_air STATIC
    src/gui/playground_cell_air.cpp
    src/gui/playground_cell_air.h
)
target_link_libraries(playground_cell_air PUBLIC softwing_core)
target_include_directories(playground_cell_air PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui)

add_library(playground_sim STATIC
    src/gui/playground_sim.cpp
    src/gui/playground_sim.h
    src/gui/playground_metrics.cpp
    src/gui/playground_metrics.h
)
add_library(playground_contact STATIC
    src/gui/playground_contact.cpp
    src/gui/playground_contact.h
)
target_link_libraries(playground_contact PUBLIC softwing_core)
target_include_directories(playground_contact PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/gui"
)
target_link_libraries(playground_sim PUBLIC
    playground_cell_air
    playground_pressure
    Qt6::Core
    softwing_core
    playground_contact
)
target_include_directories(playground_sim PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/gui"
)

# Vendored XFLR5 v6.62 (third_party/xflr5, see PROVENANCE.md): the XFoil
# solver library plus the complete xflr5v6 application compiled as a static
# library. The GUI hosts XFLR5's MainFrame in a tab; XFLR5's own
# QApplication bootstrap (globals/main.cpp, globals/xflr5app.*) is excluded.
add_library(xfoil STATIC
    third_party/xflr5/XFoil-lib/xfoil.cpp
)
target_include_directories(xfoil PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/xflr5/XFoil-lib"
)
target_compile_definitions(xfoil PUBLIC XFOILLIB_STATIC)
target_link_libraries(xfoil PUBLIC Qt6::Core)

# Explicit source list generated from the upstream qmake manifests: the
# v6.62 tree contains orphaned duplicate files that qmake never compiled,
# so a glob would sweep in code that does not build (see the header of the
# generated file for how to regenerate).
set(LEP_XFLR5_V6_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/xflr5/xflr5v6")
include(cmake/xflr5_sources.cmake)

add_library(xflr5core STATIC ${XFLR5_CORE_SOURCES})
set_target_properties(xflr5core PROPERTIES AUTOMOC ON)
target_include_directories(xflr5core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/xflr5/xflr5v6"
)
target_link_libraries(xflr5core PUBLIC
    xfoil
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    Qt6::Xml
)
if(WIN32)
    target_link_libraries(xflr5core PUBLIC opengl32 user32)
endif()
if(MSVC)
    target_compile_options(xfoil PRIVATE /W0 /bigobj)
    target_compile_options(xflr5core PRIVATE /W0 /bigobj)
    foreach(xflr5_target xfoil xflr5core)
        target_compile_definitions(${xflr5_target} PRIVATE
            _CRT_SECURE_NO_WARNINGS
            _USE_MATH_DEFINES
            NOMINMAX
        )
    endforeach()
else()
    target_compile_options(xfoil PRIVATE -w)
    target_compile_options(xflr5core PRIVATE -w)
endif()

# XFLR5's Qt resources (icons, dark stylesheet, GL shaders, script/text
# templates) must be compiled into the executable: rcc registration relies
# on static initializers that the linker may drop from a static library.
set(XFLR5_RESOURCES
    third_party/xflr5/xflr5v6/resources/images.qrc
    third_party/xflr5/xflr5v6/resources/qss.qrc
    third_party/xflr5/xflr5v6/xfl3d/resources/shaders.qrc
    third_party/xflr5/xflr5v6/xfl3d/resources/text_files.qrc
    third_party/xflr5/xflr5v6/xflscript/script.qrc
    third_party/xflr5/xflr5v6/xflwidgets/textfiles.qrc
)

add_executable(leparagliding-engine
    src/engine/engine_paths.cpp
    src/engine/engine_paths.h
    src/engine/input_migration.cpp
    src/engine/input_migration.h
    src/engine/main.cpp
    src/flatparts/flat_capture.cpp
    src/flatparts/flat_capture.h
    src/legacy/leparagliding_core.cpp
    src/model/nurbs_model.cpp
    src/model/nurbs_model.h
)
target_include_directories(leparagliding-engine PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/engine"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/flatparts"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/model"
)
target_link_libraries(leparagliding-engine PRIVATE f2c_runtime lep_occt)
if(MSVC)
    set_source_files_properties(src/legacy/leparagliding_core.cpp PROPERTIES
        COMPILE_OPTIONS "/bigobj;/W0;/Zc:strictStrings-"
    )
else()
    # The translated core is C89 wrapped in extern "C" guards; compiling it
    # as C keeps GCC/Clang from rejecting f2c's literal-to-char* idioms.
    set_source_files_properties(src/legacy/leparagliding_core.cpp PROPERTIES
        LANGUAGE C
        COMPILE_OPTIONS "-w"
    )
endif()

# Flat-part model, nester and sheet writers, shared by the Print tab and the
# nesting bench.
add_library(flatparts STATIC
    src/flatparts/flat_parts.cpp
    src/flatparts/flat_parts.h
    src/flatparts/nesting.cpp
    src/flatparts/nesting.h
    src/flatparts/sheet_export.cpp
    src/flatparts/sheet_export.h
)
target_link_libraries(flatparts PUBLIC Qt6::Gui)
target_include_directories(flatparts PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/flatparts"
)

add_executable(nesting-bench tests/nesting_bench.cpp)
target_link_libraries(nesting-bench PRIVATE flatparts)

add_executable(reference-report-compare
    tests/reference_report_compare.cpp
)

add_executable(preprocessor-reference-compare
    tests/preprocessor_reference_compare.cpp
    src/gui/geometry_preprocessor.cpp
)
target_include_directories(preprocessor-reference-compare PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/gui"
)

add_executable(section1-curves-test
    tests/section1_curves_test.cpp
    src/gui/section1_curves.cpp
)
target_include_directories(section1-curves-test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/gui"
)

add_executable(spline-fit-test
    tests/spline_fit_test.cpp
    src/gui/spline_fit.cpp
)
target_include_directories(spline-fit-test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/gui"
)

add_executable(section-grid-test
    tests/section_grid_test.cpp
    src/gui/section_grid.cpp
)
target_include_directories(section-grid-test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/gui"
)

add_executable(airfoil-file-test
    tests/airfoil_file_test.cpp
    src/gui/airfoil_file.cpp
    src/gui/spline_fit.cpp
)
target_include_directories(airfoil-file-test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/gui"
)

add_executable(playground-metrics-test
    tests/playground_metrics_test.cpp
)
target_link_libraries(playground-metrics-test PRIVATE playground_sim)

add_executable(playground-pressure-solve-test
    tests/playground_pressure_solve_test.cpp
)
target_link_libraries(playground-pressure-solve-test PRIVATE playground_pressure)

add_executable(playground-cell-air-test
    tests/playground_cell_air_test.cpp
)
target_link_libraries(playground-cell-air-test PRIVATE playground_cell_air)

add_executable(softwing-material-test
    tests/softwing_material_test.cpp
)
target_link_libraries(softwing-material-test PRIVATE softwing_core)

add_executable(softwing-cell-volume-test
    tests/softwing_cell_volume_test.cpp
)
target_link_libraries(softwing-cell-volume-test PRIVATE softwing_core)

add_executable(playground-cells-test
    tests/playground_cells_test.cpp
)
target_link_libraries(playground-cells-test PRIVATE playground_sim)

add_executable(playground-material-test
    tests/playground_material_test.cpp
)
target_link_libraries(playground-material-test PRIVATE playground_sim)

add_executable(playground-contact-test
    tests/playground_contact_test.cpp
)
target_link_libraries(playground-contact-test PRIVATE playground_contact)

add_executable(playground-contact-integration-test
    tests/playground_contact_integration_test.cpp
)
target_link_libraries(playground-contact-integration-test PRIVATE playground_sim)

add_executable(flatparts-export-test
    tests/flatparts_export_test.cpp
)
target_link_libraries(flatparts-export-test PRIVATE flatparts Qt6::Gui)

qt_add_executable(LEparagliding WIN32
    src/gui/airfoil_file.cpp
    src/gui/airfoil_file.h
    src/gui/airfoil_panel.cpp
    src/gui/airfoil_panel.h
    src/gui/flat_parts_view.cpp
    src/gui/flat_parts_view.h
    src/gui/design_document.cpp
    src/gui/design_document.h
    src/gui/geometry_preprocessor.cpp
    src/gui/geometry_preprocessor.h
    src/gui/geometry_preprocessor_dialog.cpp
    src/gui/geometry_preprocessor_dialog.h
    src/gui/grid_curve_panel.cpp
    src/gui/grid_curve_panel.h
    src/gui/holes_panel.cpp
    src/gui/holes_panel.h
    src/gui/main.cpp
    src/gui/mainwindow.cpp
    src/gui/mainwindow.h
    src/gui/curve_editor.cpp
    src/gui/curve_editor.h
    src/gui/panel_undo.h
    src/gui/paraglider_view.cpp
    src/gui/paraglider_view.h
    src/gui/playground_analysis.cpp
    src/gui/playground_analysis.h
    src/gui/playground_page.cpp
    src/gui/playground_page.h
    src/gui/nest_worker.cpp
    src/gui/nest_worker.h
    src/gui/print_page.cpp
    src/gui/print_page.h
    src/gui/preset_catalog.cpp
    src/gui/preset_catalog.h
    src/gui/section1_curve_panel.cpp
    src/gui/section1_curve_panel.h
    src/gui/section1_curves.cpp
    src/gui/section1_curves.h
    src/gui/section_grid.cpp
    src/gui/section_grid.h
    src/gui/section_grid_panel.cpp
    src/gui/section_grid_panel.h
    src/gui/section_help.cpp
    src/gui/section_help.h
    src/gui/section_specs.cpp
    src/gui/section_specs.h
    src/gui/spline_fit.cpp
    src/gui/spline_fit.h
    resources/manual/manual.qrc
    ${XFLR5_RESOURCES}
)
set_target_properties(LEparagliding PROPERTIES AUTORCC ON AUTOMOC ON)
target_link_libraries(LEparagliding PRIVATE
    Qt6::Widgets Qt6::OpenGLWidgets lep_occt xflr5core softwing_core
    playground_sim flatparts)
add_dependencies(LEparagliding leparagliding-engine)

# Headless timing harness for the Playground's XPBD solve. Developer tool,
# not shipped: it is excluded from the installer and the release archives.
add_executable(softwing-bench
    tools/softwing_bench.cpp
    tools/softwing_gpu.cpp
    tools/softwing_gpu.h
)
target_link_libraries(softwing-bench PRIVATE playground_sim Qt6::OpenGL)

# Wing presets from laboratoridenvol.com: shipped with the executable so the
# Presets menu (and the installer wildcard) picks them up. On macOS they must
# land in Contents/Resources rather than next to the binary in Contents/MacOS:
# data files under MacOS/ make `codesign --deep` treat the folder as a nested
# code bundle and reject the app ("bundle format unrecognized, invalid, or
# unsuitable").
if(APPLE)
    set(lep_presets_deploy_dir
        "$<TARGET_BUNDLE_CONTENT_DIR:LEparagliding>/Resources/presets")
else()
    set(lep_presets_deploy_dir "$<TARGET_FILE_DIR:LEparagliding>/presets")
endif()
add_custom_command(TARGET LEparagliding POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/resources/presets"
        "${lep_presets_deploy_dir}"
    COMMENT "Deploying wing presets"
)

set_target_properties(
    LEparagliding
    leparagliding-engine
    reference-report-compare
    preprocessor-reference-compare
    section1-curves-test
    spline-fit-test
    section-grid-test
    airfoil-file-test
    playground-metrics-test
    playground-pressure-solve-test
    playground-cell-air-test
    softwing-material-test
    softwing-cell-volume-test
    playground-cells-test
    playground-material-test
    playground-contact-test
    playground-contact-integration-test
    flatparts-export-test
    nesting-bench
    softwing-bench
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
foreach(configuration Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER "${configuration}" configuration_upper)
    set_target_properties(
        LEparagliding
        leparagliding-engine
        reference-report-compare
        preprocessor-reference-compare
        section1-curves-test
        spline-fit-test
        section-grid-test
        airfoil-file-test
        playground-metrics-test
        playground-pressure-solve-test
        playground-cell-air-test
        softwing-material-test
        softwing-cell-volume-test
        playground-cells-test
        playground-material-test
        playground-contact-test
        playground-contact-integration-test
        flatparts-export-test
        nesting-bench
        softwing-bench
        PROPERTIES
        "RUNTIME_OUTPUT_DIRECTORY_${configuration_upper}" "${CMAKE_BINARY_DIR}/bin/${configuration}"
    )
endforeach()

if(APPLE)
    # Bundled so macdeployqt can produce a self-contained, DMG-able app.
    set_target_properties(LEparagliding PROPERTIES
        MACOSX_BUNDLE TRUE
        MACOSX_BUNDLE_BUNDLE_NAME "LEparagliding"
        MACOSX_BUNDLE_GUI_IDENTIFIER "com.kulitorum.leparagliding"
        MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}"
        MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}"
    )
endif()

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

    # Runtime dependencies are broader than link-time imports: STEP and the
    # OpenGL viewer load several OCCT support toolkits transitively.
    set(LEP_OCCT_RUNTIME_TOOLKITS
        ${LEP_OCCT_TOOLKITS}
        TKBO
        TKCAF
        TKCDF
        TKHLR
        TKLCAF
        TKPrim
        TKShHealing
        TKVCAF
        TKXCAF
    )
    list(REMOVE_DUPLICATES LEP_OCCT_RUNTIME_TOOLKITS)
    set(lep_occt_runtime_files)
    foreach(occt_toolkit IN LISTS LEP_OCCT_RUNTIME_TOOLKITS)
        list(APPEND lep_occt_runtime_files
            "${LEP_OCCT_BINARY_DIR}/${occt_toolkit}.dll")
    endforeach()
    # Superset of the third-party runtimes across supported OCCT builds
    # (local build2, CI source build, official prebuilt); each is copied
    # only if the OCCT bin directory actually provides it.
    foreach(occt_dependency
            freetype.dll
            brotlicommon.dll
            brotlidec.dll
            bz2.dll
            libpng16.dll
            zlib1.dll
            FreeImage.dll
            avcodec-57.dll
            avformat-57.dll
            avutil-55.dll
            swscale-4.dll
            openvr_api.dll
            tbb12.dll
            tbbmalloc.dll
            jemalloc.dll)
        if(EXISTS "${LEP_OCCT_BINARY_DIR}/${occt_dependency}")
            list(APPEND lep_occt_runtime_files
                "${LEP_OCCT_BINARY_DIR}/${occt_dependency}")
        endif()
    endforeach()
    foreach(runtime_target LEparagliding leparagliding-engine)
        add_custom_command(TARGET ${runtime_target} POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                ${lep_occt_runtime_files}
                "$<TARGET_FILE_DIR:${runtime_target}>"
            COMMENT "Deploying Open CASCADE runtime for ${runtime_target}"
            COMMAND_EXPAND_LISTS
        )
    endforeach()
endif()

enable_testing()
add_test(NAME engine_help COMMAND leparagliding-engine --help)
add_test(NAME section1_curves COMMAND section1-curves-test)
add_test(NAME spline_fit COMMAND spline-fit-test)
add_test(NAME section_grid COMMAND section-grid-test)
add_test(NAME airfoil_file COMMAND airfoil-file-test
    "${CMAKE_CURRENT_SOURCE_DIR}/Data/gnua.txt")
add_test(NAME playground_metrics COMMAND playground-metrics-test)
add_test(NAME playground_pressure_solve COMMAND playground-pressure-solve-test)
add_test(NAME playground_cell_air COMMAND playground-cell-air-test)
add_test(NAME softwing_material COMMAND softwing-material-test)
add_test(NAME softwing_cell_volume COMMAND softwing-cell-volume-test)
add_test(NAME playground_cells COMMAND playground-cells-test)
add_test(NAME playground_material COMMAND playground-material-test)
add_test(NAME playground_contact COMMAND playground-contact-test)
add_test(NAME playground_contact_integration
    COMMAND playground-contact-integration-test)
add_test(NAME flatparts_export COMMAND flatparts-export-test)
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
)
set_tests_properties(studio_model_smoke PROPERTIES
    TIMEOUT 180
)
add_test(NAME gui_headless_help COMMAND LEparagliding --headless --help)
# One representative wing preset must calculate cleanly end to end; catches
# regressions in the packaged resources/presets tree.
add_test(NAME preset_gnua7_calculates
    COMMAND leparagliding-engine
        "${CMAKE_CURRENT_SOURCE_DIR}/resources/presets/gnuA7/24/leparagliding.txt"
        "${CMAKE_BINARY_DIR}/preset-test-output"
)
set_tests_properties(preset_gnua7_calculates PROPERTIES
    TIMEOUT 120
)
# Every shipped preset must pass the Studio editor's stricter validation
# (unique section headers, no blank records) — the engine alone tolerates
# upstream quirks that would leave the GUI preview silently unavailable.
add_test(NAME presets_validate_in_studio
    COMMAND LEparagliding
        --validate-presets
        "${CMAKE_CURRENT_SOURCE_DIR}/resources/presets"
)
set_tests_properties(presets_validate_in_studio PROPERTIES
    TIMEOUT 60
)
add_test(NAME engine_tolerates_blank_lines
    COMMAND leparagliding-engine
        --resource-dir "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/3.28"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/3.28/leparagliding-blank-lines.txt"
        "${CMAKE_BINARY_DIR}/blank-lines-test-output"
)
set_tests_properties(engine_tolerates_blank_lines PROPERTIES
    TIMEOUT 90
)
foreach(preprocessor_case
        basic
        vault1-cells1-odd
        vault2-cells2-even
        vault2-cells4)
    add_test(NAME "preprocessor_matches_fortran_${preprocessor_case}"
        COMMAND preprocessor-reference-compare
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/pre-1.6/${preprocessor_case}/pre-data.txt"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/pre-1.6/${preprocessor_case}/geometry-out.txt"
    )
endforeach()
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
    BUNDLE DESTINATION .
)
install(DIRECTORY resources/presets
    DESTINATION .
)
if(WIN32)
    install(FILES ${lep_occt_runtime_files}
        DESTINATION .
    )
endif()

# The Inno Setup script includes installer/version.iss, regenerated on every
# configure so the installer filename embeds the HEAD commit. Commit before
# building a release installer or the hash will be stale.
execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    OUTPUT_VARIABLE LEP_GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT LEP_GIT_HASH)
    set(LEP_GIT_HASH "unknown")
endif()
set(LEP_INSTALLER_VERSION "${PROJECT_VERSION}-${LEP_GIT_HASH}")
configure_file(
    installer/version.iss.in
    "${CMAKE_CURRENT_SOURCE_DIR}/installer/version.iss"
    @ONLY
)
