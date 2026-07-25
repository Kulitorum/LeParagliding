if(NOT DEFINED ENGINE OR NOT DEFINED REPORT_COMPARATOR
   OR NOT DEFINED INPUT OR NOT DEFINED OUTPUT_DIR
   OR NOT DEFINED REFERENCE_DIR)
    message(FATAL_ERROR
        "ENGINE, REPORT_COMPARATOR, INPUT, OUTPUT_DIR, and REFERENCE_DIR are required")
endif()

file(REMOVE_RECURSE "${OUTPUT_DIR}")
file(MAKE_DIRECTORY "${OUTPUT_DIR}")

set(history_input_directory "${OUTPUT_DIR}/history-input")
file(MAKE_DIRECTORY "${history_input_directory}")
file(READ "${INPUT}" input_contents)
file(WRITE "${history_input_directory}/leparagliding.txt"
    "${input_contents}"
    "\n* >>> LEPARAGLIDING STUDIO HISTORY V1 >>>\n"
    "* parity-test-history-record\n"
    "* <<< LEPARAGLIDING STUDIO HISTORY V1 <<<\n")
get_filename_component(input_directory "${INPUT}" DIRECTORY)

execute_process(
    COMMAND "${ENGINE}"
        --resource-dir "${input_directory}"
        "${history_input_directory}/leparagliding.txt"
        "${OUTPUT_DIR}"
    RESULT_VARIABLE engine_result
    OUTPUT_VARIABLE engine_stdout
    ERROR_VARIABLE engine_stderr
    TIMEOUT 60
)

if(NOT engine_result EQUAL 0)
    message(FATAL_ERROR
        "Engine exited with ${engine_result}\n"
        "stdout:\n${engine_stdout}\n"
        "stderr:\n${engine_stderr}")
endif()

if(NOT engine_stdout MATCHES "excluded embedded Studio version history")
    message(FATAL_ERROR
        "Engine did not report stripping the embedded history trailer\n"
        "stdout:\n${engine_stdout}")
endif()

foreach(output_name lep-out.txt lines.txt run-log.txt)
    execute_process(
        COMMAND "${REPORT_COMPARATOR}"
            "${REFERENCE_DIR}/${output_name}"
            "${OUTPUT_DIR}/${output_name}"
        RESULT_VARIABLE report_compare_result
        OUTPUT_VARIABLE report_compare_stdout
        ERROR_VARIABLE report_compare_stderr
    )
    if(NOT report_compare_result EQUAL 0)
        message(FATAL_ERROR
            "${output_name} differs from the Fortran reference\n"
            "${report_compare_stdout}${report_compare_stderr}")
    endif()
endforeach()

set(expected_leparagliding_dxf
    "5FA99147AB71E822F96248AAEC89960A19E1994D73C44BF587A60607C66DEBD3")
set(expected_lep_3d_dxf
    "179CCD5EB1C0B87936381F80D8F69A1B74C69FD38EB2D2279EE5B013DAFA0DE1")

foreach(output_name leparagliding.dxf lep-3d.dxf)
    if(output_name STREQUAL "leparagliding.dxf")
        set(expected_hash "${expected_leparagliding_dxf}")
    else()
        set(expected_hash "${expected_lep_3d_dxf}")
    endif()

    file(SHA256 "${OUTPUT_DIR}/${output_name}" actual_hash)
    string(TOUPPER "${actual_hash}" actual_hash)
    if(NOT actual_hash STREQUAL expected_hash)
        message(FATAL_ERROR
            "${output_name} differs from the native Fortran 3.28 reference\n"
            "expected ${expected_hash}\n"
            "actual   ${actual_hash}")
    endif()
endforeach()
