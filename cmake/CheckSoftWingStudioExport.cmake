if(NOT DEFINED ENGINE
   OR NOT DEFINED SOLID_CHECKER
   OR NOT DEFINED INPUT
   OR NOT DEFINED OUTPUT_DIR)
    message(FATAL_ERROR
        "ENGINE, SOLID_CHECKER, INPUT, and OUTPUT_DIR are required")
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

string(REGEX MATCH
    "Rib/skin profiles: ([0-9]+) shared NURBS boundaries, maximum deviation ([^ ]+) mm"
    rib_profile_summary "${engine_output}")
if(NOT rib_profile_summary)
    message(FATAL_ERROR
        "Engine did not report exact shared rib/skin profile boundaries")
endif()
set(shared_rib_boundary_count "${CMAKE_MATCH_1}")
set(maximum_rib_skin_deviation "${CMAKE_MATCH_2}")
if(shared_rib_boundary_count LESS 6
   OR maximum_rib_skin_deviation GREATER 0.000001)
    message(FATAL_ERROR
        "Rib profiles do not reuse the skin NURBS exactly: "
        "${shared_rib_boundary_count} shared boundaries, "
        "${maximum_rib_skin_deviation} mm deviation")
endif()

if(NOT engine_output MATCHES
   "CFD solid: [0-9]+ exterior faces, 2 generated wingtip caps, 0 trailing-edge closures, 3 centreline closures, [0-9]+ shared edges, 0 free edges")
    message(FATAL_ERROR
        "SoftWingStudio CFD solid did not apply only its required bounded "
        "closures:\n${engine_output}\n${engine_error}")
endif()

string(REGEX MATCH
    "Mini-rib shaping: ([0-9]+) constrained ribs, maximum skin pull ([0-9.eE+-]+) mm"
    minirib_profile_summary "${engine_output}")
if(NOT minirib_profile_summary)
    message(FATAL_ERROR
        "Engine did not report constrained mini-rib shaping")
endif()
set(constrained_minirib_count "${CMAKE_MATCH_1}")
set(maximum_minirib_skin_pull "${CMAKE_MATCH_2}")
if(constrained_minirib_count LESS 2)
    message(FATAL_ERROR
        "Mini-ribs did not enter the prescribed ballooning model: "
        "${constrained_minirib_count} ribs, ${maximum_minirib_skin_pull} mm pull")
endif()

foreach(required_file
        lep-3d.step
        lep-solid.step
        lep-sim.json
        lep-out.txt
        lines.txt
        run-log.txt)
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

# New simulation meshes identify mini-ribs and emit both boundaries from the
# welded mid-cell skin rows. Every sample must therefore be an actual skin node
# (including the collapsed trailing-edge tip), never a free-standing profile
# that the Playground can only attach through its broad compatibility search.
string(JSON strap_count LENGTH "${simulation}" straps)
math(EXPR last_strap "${strap_count} - 1")
string(JSON skin_nodes GET "${simulation}" nodes)
set(simulation_minirib_count 0)
foreach(strap_index RANGE 0 ${last_strap})
    string(JSON is_minirib ERROR_VARIABLE minirib_error
        GET "${simulation}" straps ${strap_index} minirib)
    if(minirib_error OR NOT is_minirib)
        continue()
    endif()
    math(EXPR simulation_minirib_count "${simulation_minirib_count} + 1")
    string(JSON a_count LENGTH "${simulation}" straps ${strap_index} a)
    string(JSON b_count LENGTH "${simulation}" straps ${strap_index} b)
    if(a_count LESS 2 OR NOT a_count EQUAL b_count)
        message(FATAL_ERROR
            "Simulation mini-rib ${strap_index} has mismatched boundaries")
    endif()
    math(EXPR last_sample "${a_count} - 1")
    foreach(side a b)
        foreach(sample_index RANGE 0 ${last_sample})
            string(JSON point GET "${simulation}"
                straps ${strap_index} ${side} ${sample_index})
            string(FIND "${skin_nodes}" "${point}" found_skin_node)
            if(found_skin_node EQUAL -1)
                message(FATAL_ERROR
                    "Simulation mini-rib ${strap_index} ${side} sample "
                    "${sample_index} is not a welded skin node")
            endif()
        endforeach()
    endforeach()
endforeach()

execute_process(
    COMMAND "${SOLID_CHECKER}" "${OUTPUT_DIR}/lep-solid.step"
    RESULT_VARIABLE solid_result
    OUTPUT_VARIABLE solid_output
    ERROR_VARIABLE solid_error
    TIMEOUT 30
)
if(NOT solid_result EQUAL 0)
    message(FATAL_ERROR
        "SoftWingStudio CFD STEP validation failed (${solid_result}):\n"
        "${solid_output}\n${solid_error}")
endif()
file(READ "${OUTPUT_DIR}/lep-solid.step" solid_model)
foreach(forbidden_product
        "Ribs"
        "Mini-ribs"
        "Diagonals"
        "Lines"
        "Brake lines"
        "Extrados curves"
        "Vent curves"
        "Intrados curves")
    if(solid_model MATCHES "PRODUCT[(]'${forbidden_product}'")
        message(FATAL_ERROR
            "CFD STEP contains internal product '${forbidden_product}'")
    endif()
endforeach()
if(simulation_minirib_count LESS 2)
    message(FATAL_ERROR
        "Generated simulation omitted attached mini-ribs")
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
string(FIND "${step_model}" "PRODUCT('Mini-rib 1'" step_minirib_index)
if(step_intrados_index EQUAL -1 OR step_vh_rib_index EQUAL -1
   OR step_minirib_index EQUAL -1)
    message(FATAL_ERROR
        "Generated STEP is missing the lower skin, native Section 12 VH-rib, "
        "or attached mini-rib")
endif()
