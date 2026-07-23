if(NOT DEFINED ENGINE OR NOT DEFINED INPUT OR NOT DEFINED OUTPUT_DIR
   OR NOT DEFINED REFERENCE_DIR)
    message(FATAL_ERROR
        "ENGINE, INPUT, OUTPUT_DIR, and REFERENCE_DIR are required")
endif()

file(REMOVE_RECURSE "${OUTPUT_DIR}")
file(MAKE_DIRECTORY "${OUTPUT_DIR}")

execute_process(
    COMMAND "${ENGINE}" "${INPUT}" "${OUTPUT_DIR}"
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

foreach(output_name
        leparagliding.dxf
        lep-3d.dxf
        lep-out.txt
        lines.txt)
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E compare_files
            "${OUTPUT_DIR}/${output_name}"
            "${REFERENCE_DIR}/${output_name}"
        RESULT_VARIABLE compare_result
    )
    if(NOT compare_result EQUAL 0)
        message(FATAL_ERROR
            "${output_name} differs from the Fortran reference")
    endif()
endforeach()
