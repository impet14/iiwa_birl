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
include CMakeFiles/trajectory_totg.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/trajectory_totg.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/trajectory_totg.dir/flags.make

CMakeFiles/trajectory_totg.dir/src/Trajectory.cpp.o: CMakeFiles/trajectory_totg.dir/flags.make
CMakeFiles/trajectory_totg.dir/src/Trajectory.cpp.o: /home/birl/worksp/iiwa_stack_birl/src/trajectory_planning/time_optimal_trajectory_generation/src/Trajectory.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/birl/worksp/iiwa_stack_birl/build/time_optimal_trajectory_generation/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/trajectory_totg.dir/src/Trajectory.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trajectory_totg.dir/src/Trajectory.cpp.o -c /home/birl/worksp/iiwa_stack_birl/src/trajectory_planning/time_optimal_trajectory_generation/src/Trajectory.cpp

CMakeFiles/trajectory_totg.dir/src/Trajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trajectory_totg.dir/src/Trajectory.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/birl/worksp/iiwa_stack_birl/src/trajectory_planning/time_optimal_trajectory_generation/src/Trajectory.cpp > CMakeFiles/trajectory_totg.dir/src/Trajectory.cpp.i

CMakeFiles/trajectory_totg.dir/src/Trajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trajectory_totg.dir/src/Trajectory.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/birl/worksp/iiwa_stack_birl/src/trajectory_planning/time_optimal_trajectory_generation/src/Trajectory.cpp -o CMakeFiles/trajectory_totg.dir/src/Trajectory.cpp.s

# Object files for target trajectory_totg
trajectory_totg_OBJECTS = \
"CMakeFiles/trajectory_totg.dir/src/Trajectory.cpp.o"

# External object files for target trajectory_totg
trajectory_totg_EXTERNAL_OBJECTS =

/home/birl/worksp/iiwa_stack_birl/devel/.private/time_optimal_trajectory_generation/lib/libtrajectory_totg.so: CMakeFiles/trajectory_totg.dir/src/Trajectory.cpp.o
/home/birl/worksp/iiwa_stack_birl/devel/.private/time_optimal_trajectory_generation/lib/libtrajectory_totg.so: CMakeFiles/trajectory_totg.dir/build.make
/home/birl/worksp/iiwa_stack_birl/devel/.private/time_optimal_trajectory_generation/lib/libtrajectory_totg.so: CMakeFiles/trajectory_totg.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/birl/worksp/iiwa_stack_birl/build/time_optimal_trajectory_generation/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/birl/worksp/iiwa_stack_birl/devel/.private/time_optimal_trajectory_generation/lib/libtrajectory_totg.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/trajectory_totg.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/trajectory_totg.dir/build: /home/birl/worksp/iiwa_stack_birl/devel/.private/time_optimal_trajectory_generation/lib/libtrajectory_totg.so

.PHONY : CMakeFiles/trajectory_totg.dir/build

CMakeFiles/trajectory_totg.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/trajectory_totg.dir/cmake_clean.cmake
.PHONY : CMakeFiles/trajectory_totg.dir/clean

CMakeFiles/trajectory_totg.dir/depend:
	cd /home/birl/worksp/iiwa_stack_birl/build/time_optimal_trajectory_generation && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/birl/worksp/iiwa_stack_birl/src/trajectory_planning/time_optimal_trajectory_generation /home/birl/worksp/iiwa_stack_birl/src/trajectory_planning/time_optimal_trajectory_generation /home/birl/worksp/iiwa_stack_birl/build/time_optimal_trajectory_generation /home/birl/worksp/iiwa_stack_birl/build/time_optimal_trajectory_generation /home/birl/worksp/iiwa_stack_birl/build/time_optimal_trajectory_generation/CMakeFiles/trajectory_totg.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/trajectory_totg.dir/depend

