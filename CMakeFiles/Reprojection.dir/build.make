# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/kirill/coding/MyPy/Stereo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kirill/coding/MyPy/Stereo

# Include any dependencies generated for this target.
include CMakeFiles/Reprojection.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Reprojection.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Reprojection.dir/flags.make

CMakeFiles/Reprojection.dir/reprojection.cpp.o: CMakeFiles/Reprojection.dir/flags.make
CMakeFiles/Reprojection.dir/reprojection.cpp.o: reprojection.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/kirill/coding/MyPy/Stereo/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Reprojection.dir/reprojection.cpp.o"
	/usr/bin/clang++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Reprojection.dir/reprojection.cpp.o -c /home/kirill/coding/MyPy/Stereo/reprojection.cpp

CMakeFiles/Reprojection.dir/reprojection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Reprojection.dir/reprojection.cpp.i"
	/usr/bin/clang++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/kirill/coding/MyPy/Stereo/reprojection.cpp > CMakeFiles/Reprojection.dir/reprojection.cpp.i

CMakeFiles/Reprojection.dir/reprojection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Reprojection.dir/reprojection.cpp.s"
	/usr/bin/clang++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/kirill/coding/MyPy/Stereo/reprojection.cpp -o CMakeFiles/Reprojection.dir/reprojection.cpp.s

CMakeFiles/Reprojection.dir/reprojection.cpp.o.requires:
.PHONY : CMakeFiles/Reprojection.dir/reprojection.cpp.o.requires

CMakeFiles/Reprojection.dir/reprojection.cpp.o.provides: CMakeFiles/Reprojection.dir/reprojection.cpp.o.requires
	$(MAKE) -f CMakeFiles/Reprojection.dir/build.make CMakeFiles/Reprojection.dir/reprojection.cpp.o.provides.build
.PHONY : CMakeFiles/Reprojection.dir/reprojection.cpp.o.provides

CMakeFiles/Reprojection.dir/reprojection.cpp.o.provides.build: CMakeFiles/Reprojection.dir/reprojection.cpp.o

# Object files for target Reprojection
Reprojection_OBJECTS = \
"CMakeFiles/Reprojection.dir/reprojection.cpp.o"

# External object files for target Reprojection
Reprojection_EXTERNAL_OBJECTS =

libReprojection.a: CMakeFiles/Reprojection.dir/reprojection.cpp.o
libReprojection.a: CMakeFiles/Reprojection.dir/build.make
libReprojection.a: CMakeFiles/Reprojection.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libReprojection.a"
	$(CMAKE_COMMAND) -P CMakeFiles/Reprojection.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Reprojection.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Reprojection.dir/build: libReprojection.a
.PHONY : CMakeFiles/Reprojection.dir/build

CMakeFiles/Reprojection.dir/requires: CMakeFiles/Reprojection.dir/reprojection.cpp.o.requires
.PHONY : CMakeFiles/Reprojection.dir/requires

CMakeFiles/Reprojection.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Reprojection.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Reprojection.dir/clean

CMakeFiles/Reprojection.dir/depend:
	cd /home/kirill/coding/MyPy/Stereo && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kirill/coding/MyPy/Stereo /home/kirill/coding/MyPy/Stereo /home/kirill/coding/MyPy/Stereo /home/kirill/coding/MyPy/Stereo /home/kirill/coding/MyPy/Stereo/CMakeFiles/Reprojection.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Reprojection.dir/depend

