#
#   Copyright 2021
#
#   Licensed under the Apache License, Version 2.0 (the "Apache License")
#   with the following modification; you may not use this file except in
#   compliance with the Apache License and the following modification to it:
#   Section 6. Trademarks. is deleted and replaced with:
#
#   6. Trademarks. This License does not grant permission to use the trade
#      names, trademarks, service marks, or product names of the Licensor
#      and its affiliates, except as required to comply with Section 4(c) of
#      the License and to reproduce the content of the NOTICE file.
#
#   You may obtain a copy of the Apache License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the Apache License with the above modification is
#   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
#   KIND, either express or implied. See the Apache License for the specific
#   language governing permissions and limitations under the Apache License.
#

#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# source & headers
set(SOURCE_FILES
     parameterization.cpp
     tessellation.cpp
)

set(PRIVATE_HEADER_FILES
)

set(PUBLIC_HEADER_FILES
     parameterization.h
     tessellation.h
     types.h
)

set(DOXY_HEADER_FILES ${PUBLIC_HEADER_FILES})

include_directories("${CMAKE_CURRENT_BINARY_DIR}")

if (NOT NO_LIB)

    # Compile objs first for both the CPU and GPU libs -----
    add_library(bfr_obj
        OBJECT
            ${SOURCE_FILES}
            ${PRIVATE_HEADER_FILES}
            ${PUBLIC_HEADER_FILES}
    )

    set_target_properties(bfr_obj
        PROPERTIES
            FOLDER "opensubdiv"
    )

endif()

#-------------------------------------------------------------------------------
osd_add_doxy_headers( "${DOXY_HEADER_FILES}" )

install(
    FILES
        ${PUBLIC_HEADER_FILES}
    DESTINATION
        "${CMAKE_INCDIR_BASE}/bfr"
    PERMISSIONS
        OWNER_READ
        GROUP_READ
        WORLD_READ )

#-------------------------------------------------------------------------------
