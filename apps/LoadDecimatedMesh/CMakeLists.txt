if(MSVC)
	FILE(GLOB LIBRARY_FILES_C "*.cpp" "*.rc")
else()
	FILE(GLOB LIBRARY_FILES_C "*.cpp")
endif()
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
cxx_executable_with_flags_no_pch(LoadDecimatedMesh "Apps" "${cxx_default}" "MVS" ${LIBRARY_FILES_C} ${LIBRARY_FILES_H})

target_link_libraries(LoadDecimatedMesh ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES})
if(OpenMVS_USE_BREAKPAD AND BREAKPAD_FOUND)
	target_link_libraries(LoadDecimatedMesh ${BREAKPAD_LIBS})
endif()

# Install
INSTALL(TARGETS LoadDecimatedMesh
	EXPORT OpenMVSTargets
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
