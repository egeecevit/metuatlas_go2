set (CONTROLSRC MdlSit.cc) 

cmake_policy(SET CMP0069 NEW) # For suppressing error add this optimization which is specified in project_legged CMakeLists.txt

add_library(control_modules STATIC ${CONTROLSRC})
install(TARGETS control_modules)

