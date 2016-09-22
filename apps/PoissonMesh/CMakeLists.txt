if(MSVC)
	FILE(GLOB LIBRARY_FILES_C "*.cpp" "*.rc")
else()
	FILE(GLOB LIBRARY_FILES_C "*.cpp")
endif()
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

set(PCL_DIR "/usr/local/share/pcl-1.7")
find_package(PCL REQUIRED COMPONENTS common io surface features search filters)

message(${CMAKE_C_COMPILER_ID})
message(${CMAKE_CXX_COMPILER_ID})
include_directories(${PCL_IO_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
cxx_executable_with_flags_no_pch(PoissonMesh "Apps" "${cxx_default}" "MVS" ${LIBRARY_FILES_C} ${LIBRARY_FILES_H})
target_link_libraries(PoissonMesh ${PCL_LIBRARIES})

if(OpenMVS_USE_BREAKPAD AND BREAKPAD_FOUND)
	target_link_libraries(PoissonMesh ${BREAKPAD_LIBS})
endif()

# Install
INSTALL(TARGETS PoissonMesh
	EXPORT OpenMVSTargets
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
