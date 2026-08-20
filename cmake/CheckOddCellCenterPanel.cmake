if(NOT DEFINED ENGINE
   OR NOT DEFINED SOLID_CHECKER
   OR NOT DEFINED INPUT
   OR NOT DEFINED OUTPUT_DIR)
    message(FATAL_ERROR
        "ENGINE, SOLID_CHECKER, INPUT, and OUTPUT_DIR are required")
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
string(REGEX MATCHALL "PRODUCT[(]'Rib 0'" rib_zero_matches "${step_text}")
string(REGEX MATCHALL "PRODUCT[(]'Rib 1'" rib_one_matches "${step_text}")
list(LENGTH panel_matches panel_count)
list(LENGTH center_matches center_count)
list(LENGTH rib_zero_matches rib_zero_count)
list(LENGTH rib_one_matches rib_one_count)
if(panel_count EQUAL 0 OR NOT panel_count EQUAL center_count)
    message(FATAL_ERROR
        "Odd-cell STEP center assembly is invalid: found ${panel_count} "
        "Panel 1 products but ${center_count} Center products")
endif()
if(NOT engine_output MATCHES
   "CFD solid: [0-9]+ exterior faces, 2 generated wingtip caps, [0-9]+ trailing-edge closures, [0-9]+ centreline closures, [0-9]+ shared edges, 0 free edges")
    message(FATAL_ERROR
        "Odd-cell CFD export did not conditionally close both open wingtips:\n"
        "${engine_output}\n${engine_error}")
endif()

set(solid_path "${OUTPUT_DIR}/lep-solid.step")
if(NOT EXISTS "${solid_path}")
    message(FATAL_ERROR "Odd-cell preset did not produce ${solid_path}")
endif()
execute_process(
    COMMAND "${SOLID_CHECKER}" "${solid_path}"
    RESULT_VARIABLE solid_result
    OUTPUT_VARIABLE solid_output
    ERROR_VARIABLE solid_error
    TIMEOUT 30
)
if(NOT solid_result EQUAL 0)
    message(FATAL_ERROR
        "CFD STEP validation failed (${solid_result}):\n"
        "${solid_output}\n${solid_error}")
endif()
file(READ "${solid_path}" solid_text)
if(NOT solid_text MATCHES "MANIFOLD_SOLID_BREP")
    message(FATAL_ERROR
        "CFD STEP does not contain a manifold solid representation")
endif()
foreach(forbidden_product
        "Ribs"
        "Mini-ribs"
        "Diagonals"
        "Lines"
        "Brake lines"
        "Extrados curves"
        "Vent curves"
        "Intrados curves")
    if(solid_text MATCHES "PRODUCT[(]'${forbidden_product}'")
        message(FATAL_ERROR
            "CFD STEP contains internal product '${forbidden_product}'")
    endif()
endforeach()
if(NOT rib_zero_count EQUAL 0 OR NOT rib_one_count EQUAL 2)
    message(FATAL_ERROR
        "Odd-cell STEP rib ownership is invalid: found ${rib_zero_count} "
        "synthetic Rib 0 products and ${rib_one_count} Rib 1 products; "
        "the centre cell must use Rib 1 and its mirror")
endif()

# The Playground must use the same ownership rule. gnuA7's innermost rib has
# holes, so the two centre-cell boundary loops must both receive that hole
# table; the old synthetic-rib-0 key left one side solid.
set(sim_path "${OUTPUT_DIR}/lep-sim.json")
if(NOT EXISTS "${sim_path}")
    message(FATAL_ERROR "Odd-cell preset did not produce ${sim_path}")
endif()
file(READ "${sim_path}" sim_text)
string(JSON rib_loop_count LENGTH "${sim_text}" ribLoops)
string(JSON rib_hole_count LENGTH "${sim_text}" ribHoles)
if(rib_loop_count LESS 2 OR NOT rib_loop_count EQUAL rib_hole_count)
    message(FATAL_ERROR
        "Odd-cell Playground rib loops and hole tables are incomplete")
endif()
string(JSON first_boundary_holes LENGTH "${sim_text}" ribHoles 0)
string(JSON second_boundary_holes LENGTH "${sim_text}" ribHoles 1)
if(first_boundary_holes EQUAL 0
   OR NOT first_boundary_holes EQUAL second_boundary_holes)
    message(FATAL_ERROR
        "Odd-cell Playground centre boundaries do not share mirrored Rib 1 "
        "holes: ${first_boundary_holes} versus ${second_boundary_holes}")
endif()

message(STATUS
    "Odd-cell STEP exports ${panel_count} center-panel products exactly once "
    "and both STEP/Playground boundaries inherit Rib 1; ${solid_output}")
