cmake_minimum_required(VERSION 3.21)

project(
    LEparagliding
    VERSION 0.1.0
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
    src/legacy/leparagliding_core.cpp
    src/model/nurbs_model.cpp
    src/model/nurbs_model.h
)
target_include_directories(leparagliding-engine PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/engine"
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

qt_add_executable(LEparagliding WIN32
    src/gui/design_document.cpp
    src/gui/design_document.h
    src/gui/geometry_preprocessor.cpp
    src/gui/geometry_preprocessor.h
    src/gui/geometry_preprocessor_dialog.cpp
    src/gui/geometry_preprocessor_dialog.h
    src/gui/main.cpp
    src/gui/mainwindow.cpp
    src/gui/mainwindow.h
    src/gui/paraglider_view.cpp
    src/gui/paraglider_view.h
    src/gui/preset_catalog.cpp
    src/gui/preset_catalog.h
    src/gui/section_help.cpp
    src/gui/section_help.h
    ${XFLR5_RESOURCES}
)
set_target_properties(LEparagliding PROPERTIES AUTORCC ON)
target_link_libraries(LEparagliding PRIVATE Qt6::Widgets lep_occt xflr5core)
add_dependencies(LEparagliding leparagliding-engine)

# Wing presets from laboratoridenvol.com: shipped next to the executable so
# the Presets menu (and the installer wildcard) picks them up.
add_custom_command(TARGET LEparagliding POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/resources/presets"
        "$<TARGET_FILE_DIR:LEparagliding>/presets"
    COMMENT "Deploying wing presets"
)

set_target_properties(
    LEparagliding
    leparagliding-engine
    reference-report-compare
    preprocessor-reference-compare
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
