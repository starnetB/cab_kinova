# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ziye01/realsence/opencv_aruco

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ziye01/realsence/opencv_aruco/build

# Include any dependencies generated for this target.
include src/CMakeFiles/realscene_cam_opt.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/realscene_cam_opt.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/realscene_cam_opt.dir/flags.make

src/CMakeFiles/realscene_cam_opt.dir/realscene_cam_opt.cpp.o: src/CMakeFiles/realscene_cam_opt.dir/flags.make
src/CMakeFiles/realscene_cam_opt.dir/realscene_cam_opt.cpp.o: ../src/realscene_cam_opt.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ziye01/realsence/opencv_aruco/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/realscene_cam_opt.dir/realscene_cam_opt.cpp.o"
	cd /home/ziye01/realsence/opencv_aruco/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/realscene_cam_opt.dir/realscene_cam_opt.cpp.o -c /home/ziye01/realsence/opencv_aruco/src/realscene_cam_opt.cpp

src/CMakeFiles/realscene_cam_opt.dir/realscene_cam_opt.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/realscene_cam_opt.dir/realscene_cam_opt.cpp.i"
	cd /home/ziye01/realsence/opencv_aruco/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ziye01/realsence/opencv_aruco/src/realscene_cam_opt.cpp > CMakeFiles/realscene_cam_opt.dir/realscene_cam_opt.cpp.i

src/CMakeFiles/realscene_cam_opt.dir/realscene_cam_opt.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/realscene_cam_opt.dir/realscene_cam_opt.cpp.s"
	cd /home/ziye01/realsence/opencv_aruco/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ziye01/realsence/opencv_aruco/src/realscene_cam_opt.cpp -o CMakeFiles/realscene_cam_opt.dir/realscene_cam_opt.cpp.s

# Object files for target realscene_cam_opt
realscene_cam_opt_OBJECTS = \
"CMakeFiles/realscene_cam_opt.dir/realscene_cam_opt.cpp.o"

# External object files for target realscene_cam_opt
realscene_cam_opt_EXTERNAL_OBJECTS =

src/librealscene_cam_opt.so: src/CMakeFiles/realscene_cam_opt.dir/realscene_cam_opt.cpp.o
src/librealscene_cam_opt.so: src/CMakeFiles/realscene_cam_opt.dir/build.make
src/librealscene_cam_opt.so: src/CMakeFiles/realscene_cam_opt.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ziye01/realsence/opencv_aruco/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library librealscene_cam_opt.so"
	cd /home/ziye01/realsence/opencv_aruco/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/realscene_cam_opt.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/realscene_cam_opt.dir/build: src/librealscene_cam_opt.so

.PHONY : src/CMakeFiles/realscene_cam_opt.dir/build

src/CMakeFiles/realscene_cam_opt.dir/clean:
	cd /home/ziye01/realsence/opencv_aruco/build/src && $(CMAKE_COMMAND) -P CMakeFiles/realscene_cam_opt.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/realscene_cam_opt.dir/clean

src/CMakeFiles/realscene_cam_opt.dir/depend:
	cd /home/ziye01/realsence/opencv_aruco/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ziye01/realsence/opencv_aruco /home/ziye01/realsence/opencv_aruco/src /home/ziye01/realsence/opencv_aruco/build /home/ziye01/realsence/opencv_aruco/build/src /home/ziye01/realsence/opencv_aruco/build/src/CMakeFiles/realscene_cam_opt.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/realscene_cam_opt.dir/depend

