if(NOT DEFINED ENGINE OR NOT DEFINED INPUT OR NOT DEFINED OUTPUT_DIR)
    message(FATAL_ERROR "ENGINE, INPUT, and OUTPUT_DIR are required")
endif()

file(MAKE_DIRECTORY "${OUTPUT_DIR}")
execute_process(
    COMMAND "${ENGINE}" "${INPUT}" "${OUTPUT_DIR}"
    RESULT_VARIABLE engine_result
    OUTPUT_VARIABLE engine_output
    ERROR_VARIABLE engine_error
    TIMEOUT 110
)
if(NOT engine_result EQUAL 0)
    message(FATAL_ERROR
        "Odd-cell preset calculation failed (${engine_result}):\n"
        "${engine_output}\n${engine_error}")
endif()

set(step_path "${OUTPUT_DIR}/lep-3d.step")
if(NOT EXISTS "${step_path}")
    message(FATAL_ERROR "Odd-cell preset did not produce ${step_path}")
endif()

# gnuA7 has an odd cell count, so Panel 1 spans the centreline and is already
# the complete physical centre cell. Every exported Panel 1 leaf (skin and
# optional construction-curve regions) must therefore live in a Center group;
# a larger panel count is the old coincident Left + Right double export.
file(READ "${step_path}" step_text)
string(REGEX MATCHALL "PRODUCT[(]'Panel 1'" panel_matches "${step_text}")
string(REGEX MATCHALL "PRODUCT[(]'Center'" center_matches "${step_text}")
list(LENGTH panel_matches panel_count)
list(LENGTH center_matches center_count)
if(panel_count EQUAL 0 OR NOT panel_count EQUAL center_count)
    message(FATAL_ERROR
        "Odd-cell STEP center assembly is invalid: found ${panel_count} "
        "Panel 1 products but ${center_count} Center products")
endif()

message(STATUS
    "Odd-cell STEP exports ${panel_count} center-panel products exactly once")
