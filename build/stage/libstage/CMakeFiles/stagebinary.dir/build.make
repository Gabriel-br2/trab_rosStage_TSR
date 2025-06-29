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
CMAKE_SOURCE_DIR = /home/rafaella/ros2_ws/src/Stage

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rafaella/ros2_ws/build/stage

# Include any dependencies generated for this target.
include libstage/CMakeFiles/stagebinary.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include libstage/CMakeFiles/stagebinary.dir/compiler_depend.make

# Include the progress variables for this target.
include libstage/CMakeFiles/stagebinary.dir/progress.make

# Include the compile flags for this target's objects.
include libstage/CMakeFiles/stagebinary.dir/flags.make

libstage/CMakeFiles/stagebinary.dir/main.cc.o: libstage/CMakeFiles/stagebinary.dir/flags.make
libstage/CMakeFiles/stagebinary.dir/main.cc.o: /home/rafaella/ros2_ws/src/Stage/libstage/main.cc
libstage/CMakeFiles/stagebinary.dir/main.cc.o: libstage/CMakeFiles/stagebinary.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rafaella/ros2_ws/build/stage/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object libstage/CMakeFiles/stagebinary.dir/main.cc.o"
	cd /home/rafaella/ros2_ws/build/stage/libstage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT libstage/CMakeFiles/stagebinary.dir/main.cc.o -MF CMakeFiles/stagebinary.dir/main.cc.o.d -o CMakeFiles/stagebinary.dir/main.cc.o -c /home/rafaella/ros2_ws/src/Stage/libstage/main.cc

libstage/CMakeFiles/stagebinary.dir/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stagebinary.dir/main.cc.i"
	cd /home/rafaella/ros2_ws/build/stage/libstage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rafaella/ros2_ws/src/Stage/libstage/main.cc > CMakeFiles/stagebinary.dir/main.cc.i

libstage/CMakeFiles/stagebinary.dir/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stagebinary.dir/main.cc.s"
	cd /home/rafaella/ros2_ws/build/stage/libstage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rafaella/ros2_ws/src/Stage/libstage/main.cc -o CMakeFiles/stagebinary.dir/main.cc.s

# Object files for target stagebinary
stagebinary_OBJECTS = \
"CMakeFiles/stagebinary.dir/main.cc.o"

# External object files for target stagebinary
stagebinary_EXTERNAL_OBJECTS =

libstage/stage: libstage/CMakeFiles/stagebinary.dir/main.cc.o
libstage/stage: libstage/CMakeFiles/stagebinary.dir/build.make
libstage/stage: libstage/libstage.so.4.3.0
libstage/stage: /usr/lib/x86_64-linux-gnu/libGL.so
libstage/stage: /usr/lib/x86_64-linux-gnu/libGLU.so
libstage/stage: /usr/lib/x86_64-linux-gnu/libltdl.so
libstage/stage: /usr/lib/x86_64-linux-gnu/libjpeg.so
libstage/stage: /usr/lib/x86_64-linux-gnu/libpng.so
libstage/stage: /usr/lib/x86_64-linux-gnu/libz.so
libstage/stage: /usr/lib/x86_64-linux-gnu/libGL.so
libstage/stage: /usr/lib/x86_64-linux-gnu/libGLU.so
libstage/stage: /usr/lib/x86_64-linux-gnu/libltdl.so
libstage/stage: /usr/lib/x86_64-linux-gnu/libjpeg.so
libstage/stage: /usr/lib/x86_64-linux-gnu/libpng.so
libstage/stage: /usr/lib/x86_64-linux-gnu/libz.so
libstage/stage: /usr/lib/x86_64-linux-gnu/libfltk_images.so
libstage/stage: /usr/lib/x86_64-linux-gnu/libfltk_forms.so
libstage/stage: /usr/lib/x86_64-linux-gnu/libfltk_gl.so
libstage/stage: /usr/lib/x86_64-linux-gnu/libfltk.so
libstage/stage: /usr/lib/x86_64-linux-gnu/libX11.so
libstage/stage: /usr/lib/x86_64-linux-gnu/libm.so
libstage/stage: libstage/CMakeFiles/stagebinary.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rafaella/ros2_ws/build/stage/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable stage"
	cd /home/rafaella/ros2_ws/build/stage/libstage && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stagebinary.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
libstage/CMakeFiles/stagebinary.dir/build: libstage/stage
.PHONY : libstage/CMakeFiles/stagebinary.dir/build

libstage/CMakeFiles/stagebinary.dir/clean:
	cd /home/rafaella/ros2_ws/build/stage/libstage && $(CMAKE_COMMAND) -P CMakeFiles/stagebinary.dir/cmake_clean.cmake
.PHONY : libstage/CMakeFiles/stagebinary.dir/clean

libstage/CMakeFiles/stagebinary.dir/depend:
	cd /home/rafaella/ros2_ws/build/stage && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rafaella/ros2_ws/src/Stage /home/rafaella/ros2_ws/src/Stage/libstage /home/rafaella/ros2_ws/build/stage /home/rafaella/ros2_ws/build/stage/libstage /home/rafaella/ros2_ws/build/stage/libstage/CMakeFiles/stagebinary.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libstage/CMakeFiles/stagebinary.dir/depend

