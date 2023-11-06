# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hannah/ws_bt/src/BehaviorTree.CPP

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hannah/ws_bt/src/BehaviorTree.CPP/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/t06_subtree_port_remapping.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include examples/CMakeFiles/t06_subtree_port_remapping.dir/compiler_depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/t06_subtree_port_remapping.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/t06_subtree_port_remapping.dir/flags.make

examples/CMakeFiles/t06_subtree_port_remapping.dir/t06_subtree_port_remapping.cpp.o: examples/CMakeFiles/t06_subtree_port_remapping.dir/flags.make
examples/CMakeFiles/t06_subtree_port_remapping.dir/t06_subtree_port_remapping.cpp.o: ../examples/t06_subtree_port_remapping.cpp
examples/CMakeFiles/t06_subtree_port_remapping.dir/t06_subtree_port_remapping.cpp.o: examples/CMakeFiles/t06_subtree_port_remapping.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hannah/ws_bt/src/BehaviorTree.CPP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/t06_subtree_port_remapping.dir/t06_subtree_port_remapping.cpp.o"
	cd /home/hannah/ws_bt/src/BehaviorTree.CPP/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/t06_subtree_port_remapping.dir/t06_subtree_port_remapping.cpp.o -MF CMakeFiles/t06_subtree_port_remapping.dir/t06_subtree_port_remapping.cpp.o.d -o CMakeFiles/t06_subtree_port_remapping.dir/t06_subtree_port_remapping.cpp.o -c /home/hannah/ws_bt/src/BehaviorTree.CPP/examples/t06_subtree_port_remapping.cpp

examples/CMakeFiles/t06_subtree_port_remapping.dir/t06_subtree_port_remapping.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/t06_subtree_port_remapping.dir/t06_subtree_port_remapping.cpp.i"
	cd /home/hannah/ws_bt/src/BehaviorTree.CPP/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hannah/ws_bt/src/BehaviorTree.CPP/examples/t06_subtree_port_remapping.cpp > CMakeFiles/t06_subtree_port_remapping.dir/t06_subtree_port_remapping.cpp.i

examples/CMakeFiles/t06_subtree_port_remapping.dir/t06_subtree_port_remapping.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/t06_subtree_port_remapping.dir/t06_subtree_port_remapping.cpp.s"
	cd /home/hannah/ws_bt/src/BehaviorTree.CPP/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hannah/ws_bt/src/BehaviorTree.CPP/examples/t06_subtree_port_remapping.cpp -o CMakeFiles/t06_subtree_port_remapping.dir/t06_subtree_port_remapping.cpp.s

# Object files for target t06_subtree_port_remapping
t06_subtree_port_remapping_OBJECTS = \
"CMakeFiles/t06_subtree_port_remapping.dir/t06_subtree_port_remapping.cpp.o"

# External object files for target t06_subtree_port_remapping
t06_subtree_port_remapping_EXTERNAL_OBJECTS =

examples/t06_subtree_port_remapping: examples/CMakeFiles/t06_subtree_port_remapping.dir/t06_subtree_port_remapping.cpp.o
examples/t06_subtree_port_remapping: examples/CMakeFiles/t06_subtree_port_remapping.dir/build.make
examples/t06_subtree_port_remapping: sample_nodes/lib/libbt_sample_nodes.a
examples/t06_subtree_port_remapping: libbehaviortree_cpp.so
examples/t06_subtree_port_remapping: examples/CMakeFiles/t06_subtree_port_remapping.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hannah/ws_bt/src/BehaviorTree.CPP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable t06_subtree_port_remapping"
	cd /home/hannah/ws_bt/src/BehaviorTree.CPP/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/t06_subtree_port_remapping.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/t06_subtree_port_remapping.dir/build: examples/t06_subtree_port_remapping
.PHONY : examples/CMakeFiles/t06_subtree_port_remapping.dir/build

examples/CMakeFiles/t06_subtree_port_remapping.dir/clean:
	cd /home/hannah/ws_bt/src/BehaviorTree.CPP/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/t06_subtree_port_remapping.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/t06_subtree_port_remapping.dir/clean

examples/CMakeFiles/t06_subtree_port_remapping.dir/depend:
	cd /home/hannah/ws_bt/src/BehaviorTree.CPP/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hannah/ws_bt/src/BehaviorTree.CPP /home/hannah/ws_bt/src/BehaviorTree.CPP/examples /home/hannah/ws_bt/src/BehaviorTree.CPP/build /home/hannah/ws_bt/src/BehaviorTree.CPP/build/examples /home/hannah/ws_bt/src/BehaviorTree.CPP/build/examples/CMakeFiles/t06_subtree_port_remapping.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/t06_subtree_port_remapping.dir/depend

