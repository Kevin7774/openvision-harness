if(NOT DEFINED ROOT)
    message(FATAL_ERROR "ROOT is required")
endif()

if(NOT DEFINED CHECK_DIRS)
    set(CHECK_DIRS src include)
endif()

if(NOT DEFINED EXCLUDE_DIRS)
    set(EXCLUDE_DIRS build install third_party generated test)
endif()

set(FOUND_VIOLATION OFF)

foreach(CHECK_DIR IN LISTS CHECK_DIRS)
    file(GLOB_RECURSE CANDIDATES
        "${ROOT}/${CHECK_DIR}/*.h"
        "${ROOT}/${CHECK_DIR}/*.hpp"
        "${ROOT}/${CHECK_DIR}/*.cpp"
        "${ROOT}/${CHECK_DIR}/*.cc"
        "${ROOT}/${CHECK_DIR}/*.cxx"
    )

    foreach(FILE_PATH IN LISTS CANDIDATES)
        set(SKIP_FILE OFF)
        foreach(EXCLUDE_DIR IN LISTS EXCLUDE_DIRS)
            string(FIND "${FILE_PATH}" "/${EXCLUDE_DIR}/" EXCLUDE_POS)
            if(NOT EXCLUDE_POS EQUAL -1)
                set(SKIP_FILE ON)
            endif()
        endforeach()

        if(SKIP_FILE)
            continue()
        endif()

        file(READ "${FILE_PATH}" CONTENT)
        string(REGEX MATCHALL "(^|[^A-Za-z0-9_])(throw|try|catch)([^A-Za-z0-9_]|$)" MATCHES "${CONTENT}")
        if(MATCHES)
            message(SEND_ERROR "C++ exception keyword found in ${FILE_PATH}: ${MATCHES}")
            set(FOUND_VIOLATION ON)
        endif()
    endforeach()
endforeach()

if(FOUND_VIOLATION)
    message(FATAL_ERROR "no-exceptions check failed")
endif()
