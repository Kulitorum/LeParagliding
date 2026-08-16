if(NOT DEFINED ENGINE OR NOT DEFINED INPUT OR NOT DEFINED OUTPUT_DIR)
    message(FATAL_ERROR "ENGINE, INPUT, and OUTPUT_DIR are required")
endif()

get_filename_component(input_directory "${INPUT}" DIRECTORY)
execute_process(
    COMMAND "${ENGINE}"
        --resource-dir "${input_directory}"
        "${INPUT}"
        "${OUTPUT_DIR}"
    RESULT_VARIABLE engine_result
    OUTPUT_VARIABLE engine_output
    ERROR_VARIABLE engine_error
)
if(NOT engine_result EQUAL 0)
    message(FATAL_ERROR
        "SoftWingStudio export failed in the LEparagliding engine "
        "(${engine_result}).\n${engine_output}\n${engine_error}")
endif()

foreach(required_file lep-3d.step lep-sim.json lep-out.txt lines.txt run-log.txt)
    set(required_path "${OUTPUT_DIR}/${required_file}")
    if(NOT EXISTS "${required_path}")
        message(FATAL_ERROR "Engine did not create ${required_file}")
    endif()
    file(SIZE "${required_path}" required_size)
    if(required_size LESS 16)
        message(FATAL_ERROR "Engine created an empty ${required_file}")
    endif()
endforeach()

file(READ "${OUTPUT_DIR}/lep-sim.json" simulation)
string(JSON node_count LENGTH "${simulation}" nodes)
string(JSON quad_count LENGTH "${simulation}" quads)
string(JSON rib_loop_count LENGTH "${simulation}" ribLoops)
string(JSON line_count LENGTH "${simulation}" lines)
if(node_count LESS 20 OR quad_count LESS 10 OR rib_loop_count LESS 5
   OR line_count LESS 4)
    message(FATAL_ERROR
        "Generated simulation is incomplete: ${node_count} nodes, "
        "${quad_count} quads, ${rib_loop_count} rib loops, "
        "${line_count} lines")
endif()

string(FIND "${simulation}" "\"intrados\"" intrados_index)
if(intrados_index EQUAL -1)
    message(FATAL_ERROR
        "Generated simulation does not contain the exported lower surface")
endif()

# The fixture's native VH bridge has a deliberately broad upper landing. Its
# captured rib contour arches above both endpoints; a legacy straight a-to-b
# seam cannot do that.
string(JSON seam_start_z GET "${simulation}" straps 1 a 0 2)
string(JSON seam_arch_z GET "${simulation}" straps 1 a 4 2)
string(JSON seam_end_z GET "${simulation}" straps 1 a 5 2)
if(NOT seam_arch_z GREATER seam_start_z
   OR NOT seam_arch_z GREATER seam_end_z)
    message(FATAL_ERROR
        "Native VH-rib landing was flattened instead of following the rib contour")
endif()

file(READ "${OUTPUT_DIR}/lep-3d.step" step_model)
string(FIND "${step_model}" "PRODUCT('Intrados'" step_intrados_index)
string(FIND "${step_model}" "PRODUCT('VH-rib 1'" step_vh_rib_index)
if(step_intrados_index EQUAL -1 OR step_vh_rib_index EQUAL -1)
    message(FATAL_ERROR
        "Generated STEP is missing the lower skin or native Section 12 VH-rib")
endif()
