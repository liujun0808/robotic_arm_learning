# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lj/project/force_control

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lj/project/force_control/build

# Include any dependencies generated for this target.
include CMakeFiles/demo_dyn.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/demo_dyn.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/demo_dyn.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/demo_dyn.dir/flags.make

CMakeFiles/demo_dyn.dir/demo/demo_dyn.cpp.o: CMakeFiles/demo_dyn.dir/flags.make
CMakeFiles/demo_dyn.dir/demo/demo_dyn.cpp.o: /home/lj/project/force_control/demo/demo_dyn.cpp
CMakeFiles/demo_dyn.dir/demo/demo_dyn.cpp.o: CMakeFiles/demo_dyn.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/lj/project/force_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/demo_dyn.dir/demo/demo_dyn.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/demo_dyn.dir/demo/demo_dyn.cpp.o -MF CMakeFiles/demo_dyn.dir/demo/demo_dyn.cpp.o.d -o CMakeFiles/demo_dyn.dir/demo/demo_dyn.cpp.o -c /home/lj/project/force_control/demo/demo_dyn.cpp

CMakeFiles/demo_dyn.dir/demo/demo_dyn.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/demo_dyn.dir/demo/demo_dyn.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lj/project/force_control/demo/demo_dyn.cpp > CMakeFiles/demo_dyn.dir/demo/demo_dyn.cpp.i

CMakeFiles/demo_dyn.dir/demo/demo_dyn.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/demo_dyn.dir/demo/demo_dyn.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lj/project/force_control/demo/demo_dyn.cpp -o CMakeFiles/demo_dyn.dir/demo/demo_dyn.cpp.s

# Object files for target demo_dyn
demo_dyn_OBJECTS = \
"CMakeFiles/demo_dyn.dir/demo/demo_dyn.cpp.o"

# External object files for target demo_dyn
demo_dyn_EXTERNAL_OBJECTS =

demo_dyn: CMakeFiles/demo_dyn.dir/demo/demo_dyn.cpp.o
demo_dyn: CMakeFiles/demo_dyn.dir/build.make
demo_dyn: libsim_interface.a
demo_dyn: libcontrol_algorithm.a
demo_dyn: libsim_interface.a
demo_dyn: /usr/lib/x86_64-linux-gnu/libglfw.so.3.3
demo_dyn: /opt/pinocchio/lib/libpinocchio_parsers.so.3.7.0
demo_dyn: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
demo_dyn: /usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so
demo_dyn: /usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so
demo_dyn: /usr/lib/x86_64-linux-gnu/liburdfdom_model.so
demo_dyn: /usr/lib/x86_64-linux-gnu/liburdfdom_world.so
demo_dyn: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
demo_dyn: /opt/pinocchio/lib/libpinocchio_visualizers.so.3.7.0
demo_dyn: /opt/pinocchio/lib/libpinocchio_default.so.3.7.0
demo_dyn: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
demo_dyn: /usr/lib/x86_64-linux-gnu/libjsoncpp.so.1.7.4
demo_dyn: /opt/qpOASES/lib/libqpOASES.a
demo_dyn: CMakeFiles/demo_dyn.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/lj/project/force_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable demo_dyn"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/demo_dyn.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/demo_dyn.dir/build: demo_dyn
.PHONY : CMakeFiles/demo_dyn.dir/build

CMakeFiles/demo_dyn.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/demo_dyn.dir/cmake_clean.cmake
.PHONY : CMakeFiles/demo_dyn.dir/clean

CMakeFiles/demo_dyn.dir/depend:
	cd /home/lj/project/force_control/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lj/project/force_control /home/lj/project/force_control /home/lj/project/force_control/build /home/lj/project/force_control/build /home/lj/project/force_control/build/CMakeFiles/demo_dyn.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/demo_dyn.dir/depend

