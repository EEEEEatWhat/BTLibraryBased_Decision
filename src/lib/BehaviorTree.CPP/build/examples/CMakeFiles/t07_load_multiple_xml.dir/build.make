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
include examples/CMakeFiles/t07_load_multiple_xml.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include examples/CMakeFiles/t07_load_multiple_xml.dir/compiler_depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/t07_load_multiple_xml.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/t07_load_multiple_xml.dir/flags.make

examples/CMakeFiles/t07_load_multiple_xml.dir/t07_load_multiple_xml.cpp.o: examples/CMakeFiles/t07_load_multiple_xml.dir/flags.make
examples/CMakeFiles/t07_load_multiple_xml.dir/t07_load_multiple_xml.cpp.o: ../examples/t07_load_multiple_xml.cpp
examples/CMakeFiles/t07_load_multiple_xml.dir/t07_load_multiple_xml.cpp.o: examples/CMakeFiles/t07_load_multiple_xml.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hannah/ws_bt/src/BehaviorTree.CPP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/t07_load_multiple_xml.dir/t07_load_multiple_xml.cpp.o"
	cd /home/hannah/ws_bt/src/BehaviorTree.CPP/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/t07_load_multiple_xml.dir/t07_load_multiple_xml.cpp.o -MF CMakeFiles/t07_load_multiple_xml.dir/t07_load_multiple_xml.cpp.o.d -o CMakeFiles/t07_load_multiple_xml.dir/t07_load_multiple_xml.cpp.o -c /home/hannah/ws_bt/src/BehaviorTree.CPP/examples/t07_load_multiple_xml.cpp

examples/CMakeFiles/t07_load_multiple_xml.dir/t07_load_multiple_xml.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/t07_load_multiple_xml.dir/t07_load_multiple_xml.cpp.i"
	cd /home/hannah/ws_bt/src/BehaviorTree.CPP/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hannah/ws_bt/src/BehaviorTree.CPP/examples/t07_load_multiple_xml.cpp > CMakeFiles/t07_load_multiple_xml.dir/t07_load_multiple_xml.cpp.i

examples/CMakeFiles/t07_load_multiple_xml.dir/t07_load_multiple_xml.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/t07_load_multiple_xml.dir/t07_load_multiple_xml.cpp.s"
	cd /home/hannah/ws_bt/src/BehaviorTree.CPP/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hannah/ws_bt/src/BehaviorTree.CPP/examples/t07_load_multiple_xml.cpp -o CMakeFiles/t07_load_multiple_xml.dir/t07_load_multiple_xml.cpp.s

# Object files for target t07_load_multiple_xml
t07_load_multiple_xml_OBJECTS = \
"CMakeFiles/t07_load_multiple_xml.dir/t07_load_multiple_xml.cpp.o"

# External object files for target t07_load_multiple_xml
t07_load_multiple_xml_EXTERNAL_OBJECTS =

examples/t07_load_multiple_xml: examples/CMakeFiles/t07_load_multiple_xml.dir/t07_load_multiple_xml.cpp.o
examples/t07_load_multiple_xml: examples/CMakeFiles/t07_load_multiple_xml.dir/build.make
examples/t07_load_multiple_xml: sample_nodes/lib/libbt_sample_nodes.a
examples/t07_load_multiple_xml: libbehaviortree_cpp.so
examples/t07_load_multiple_xml: examples/CMakeFiles/t07_load_multiple_xml.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hannah/ws_bt/src/BehaviorTree.CPP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable t07_load_multiple_xml"
	cd /home/hannah/ws_bt/src/BehaviorTree.CPP/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/t07_load_multiple_xml.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/t07_load_multiple_xml.dir/build: examples/t07_load_multiple_xml
.PHONY : examples/CMakeFiles/t07_load_multiple_xml.dir/build

examples/CMakeFiles/t07_load_multiple_xml.dir/clean:
	cd /home/hannah/ws_bt/src/BehaviorTree.CPP/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/t07_load_multiple_xml.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/t07_load_multiple_xml.dir/clean

examples/CMakeFiles/t07_load_multiple_xml.dir/depend:
	cd /home/hannah/ws_bt/src/BehaviorTree.CPP/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hannah/ws_bt/src/BehaviorTree.CPP /home/hannah/ws_bt/src/BehaviorTree.CPP/examples /home/hannah/ws_bt/src/BehaviorTree.CPP/build /home/hannah/ws_bt/src/BehaviorTree.CPP/build/examples /home/hannah/ws_bt/src/BehaviorTree.CPP/build/examples/CMakeFiles/t07_load_multiple_xml.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/t07_load_multiple_xml.dir/depend

