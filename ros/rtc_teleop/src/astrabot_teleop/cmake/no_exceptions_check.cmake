if(NOT DEFINED ROOT)
  message(FATAL_ERROR "ROOT is required")
endif()

file(GLOB_RECURSE TELEOP_SOURCES
  "${ROOT}/include/*.h"
  "${ROOT}/include/*.hpp"
  "${ROOT}/src/*.cpp"
)

set(OFFENDERS "")
foreach(SOURCE_FILE IN LISTS TELEOP_SOURCES)
  file(READ "${SOURCE_FILE}" SOURCE_CONTENT)
  string(REGEX MATCH "(^|[^A-Za-z0-9_])(throw|try|catch)([^A-Za-z0-9_]|$)" MATCHED "${SOURCE_CONTENT}")
  if(MATCHED)
    list(APPEND OFFENDERS "${SOURCE_FILE}")
  endif()
endforeach()

if(OFFENDERS)
  list(JOIN OFFENDERS "\n  " OFFENDER_TEXT)
  message(FATAL_ERROR "handwritten exception keyword found:\n  ${OFFENDER_TEXT}")
endif()

message(STATUS "astrabot_teleop handwritten no-exceptions check passed")
