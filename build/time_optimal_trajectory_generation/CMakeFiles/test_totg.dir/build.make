# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/cmake/cmake-3.12.0-Linux-x86_64/bin/cmake

# The command to remove a file.
RM = /opt/cmake/cmake-3.12.0-Linux-x86_64/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/birl/worksp/iiwa_stack_birl/src/trajectory_planning/time_optimal_trajectory_generation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/birl/worksp/iiwa_stack_birl/build/time_optimal_trajectory_generation

# Include any dependencies generated for this target.
include CMakeFiles/test_totg.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_totg.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_totg.dir/flags.make

CMakeFiles/test_totg.dir/src/Test.cpp.o: CMakeFiles/test_totg.dir/flags.make
CMakeFiles/test_totg.dir/src/Test.cpp.o: /home/birl/worksp/iiwa_stack_birl/src/trajectory_planning/time_optimal_trajectory_generation/src/Test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/birl/worksp/iiwa_stack_birl/build/time_optimal_trajectory_generation/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_totg.dir/src/Test.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_totg.dir/src/Test.cpp.o -c /home/birl/worksp/iiwa_stack_birl/src/trajectory_planning/time_optimal_trajectory_generation/src/Test.cpp

CMakeFiles/test_totg.dir/src/Test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_totg.dir/src/Test.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/birl/worksp/iiwa_stack_birl/src/trajectory_planning/time_optimal_trajectory_generation/src/Test.cpp > CMakeFiles/test_totg.dir/src/Test.cpp.i

CMakeFiles/test_totg.dir/src/Test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_totg.dir/src/Test.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/birl/worksp/iiwa_stack_birl/src/trajectory_planning/time_optimal_trajectory_generation/src/Test.cpp -o CMakeFiles/test_totg.dir/src/Test.cpp.s

# Object files for target test_totg
test_totg_OBJECTS = \
"CMakeFiles/test_totg.dir/src/Test.cpp.o"

# External object files for target test_totg
test_totg_EXTERNAL_OBJECTS =

/home/birl/worksp/iiwa_stack_birl/devel/.private/time_optimal_trajectory_generation/lib/time_optimal_trajectory_generation/test_totg: CMakeFiles/test_totg.dir/src/Test.cpp.o
/home/birl/worksp/iiwa_stack_birl/devel/.private/time_optimal_trajectory_generation/lib/time_optimal_trajectory_generation/test_totg: CMakeFiles/test_totg.dir/build.make
/home/birl/worksp/iiwa_stack_birl/devel/.private/time_optimal_trajectory_generation/lib/time_optimal_trajectory_generation/test_totg: /home/birl/worksp/iiwa_stack_birl/devel/.private/time_optimal_trajectory_generation/lib/libpath_totg.so
/home/birl/worksp/iiwa_stack_birl/devel/.private/time_optimal_trajectory_generation/lib/time_optimal_trajectory_generation/test_totg: /home/birl/worksp/iiwa_stack_birl/devel/.private/time_optimal_trajectory_generation/lib/libtrajectory_totg.so
/home/birl/worksp/iiwa_stack_birl/devel/.private/time_optimal_trajectory_generation/lib/time_optimal_trajectory_generation/test_totg: CMakeFiles/test_totg.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/birl/worksp/iiwa_stack_birl/build/time_optimal_trajectory_generation/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/birl/worksp/iiwa_stack_birl/devel/.private/time_optimal_trajectory_generation/lib/time_optimal_trajectory_generation/test_totg"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_totg.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_totg.dir/build: /home/birl/worksp/iiwa_stack_birl/devel/.private/time_optimal_trajectory_generation/lib/time_optimal_trajectory_generation/test_totg

.PHONY : CMakeFiles/test_totg.dir/build

CMakeFiles/test_totg.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_totg.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_totg.dir/clean

CMakeFiles/test_totg.dir/depend:
	cd /home/birl/worksp/iiwa_stack_birl/build/time_optimal_trajectory_generation && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/birl/worksp/iiwa_stack_birl/src/trajectory_planning/time_optimal_trajectory_generation /home/birl/worksp/iiwa_stack_birl/src/trajectory_planning/time_optimal_trajectory_generation /home/birl/worksp/iiwa_stack_birl/build/time_optimal_trajectory_generation /home/birl/worksp/iiwa_stack_birl/build/time_optimal_trajectory_generation /home/birl/worksp/iiwa_stack_birl/build/time_optimal_trajectory_generation/CMakeFiles/test_totg.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_totg.dir/depend

