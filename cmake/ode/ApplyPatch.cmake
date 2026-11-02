foreach(required SOURCE_DIR PATCH_FILE HELPER_FILE GIT_EXECUTABLE)
  if(NOT DEFINED ${required})
    message(FATAL_ERROR "Missing ${required} for bundled ODE math patch")
  endif()
endforeach()
set(previous "${SOURCE_DIR}/.grsim-math-applied.patch")
execute_process(COMMAND "${GIT_EXECUTABLE}" apply --reverse --check "${PATCH_FILE}"
  WORKING_DIRECTORY "${SOURCE_DIR}" RESULT_VARIABLE already_applied
  OUTPUT_QUIET ERROR_QUIET)
if(NOT already_applied EQUAL 0)
  if(EXISTS "${previous}")
    execute_process(COMMAND "${GIT_EXECUTABLE}" apply --reverse --check "${previous}"
      WORKING_DIRECTORY "${SOURCE_DIR}" RESULT_VARIABLE can_remove
      OUTPUT_QUIET ERROR_QUIET)
    if(can_remove EQUAL 0)
      execute_process(COMMAND "${GIT_EXECUTABLE}" apply --reverse "${previous}"
        WORKING_DIRECTORY "${SOURCE_DIR}" RESULT_VARIABLE removed)
      if(NOT removed EQUAL 0)
        message(FATAL_ERROR "Could not update the previous grSim ODE math patch")
      endif()
    endif()
  endif()
  execute_process(COMMAND "${GIT_EXECUTABLE}" apply --check "${PATCH_FILE}"
    WORKING_DIRECTORY "${SOURCE_DIR}" RESULT_VARIABLE can_apply ERROR_VARIABLE error)
  if(NOT can_apply EQUAL 0)
    message(FATAL_ERROR "ODE math patch overlaps local dependency changes: ${error}")
  endif()
  execute_process(COMMAND "${GIT_EXECUTABLE}" apply "${PATCH_FILE}"
    WORKING_DIRECTORY "${SOURCE_DIR}" RESULT_VARIABLE applied)
  if(NOT applied EQUAL 0)
    message(FATAL_ERROR "Could not apply the grSim ODE math patch")
  endif()
endif()
configure_file("${PATCH_FILE}" "${previous}" COPYONLY)
configure_file("${HELPER_FILE}" "${SOURCE_DIR}/ode/src/grsim_ode_math.h" COPYONLY)
