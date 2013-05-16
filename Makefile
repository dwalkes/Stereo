# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

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

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running interactive CMake command-line interface..."
	/usr/bin/cmake -i .
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/kirill/coding/MyPy/Stereo/CMakeFiles /home/kirill/coding/MyPy/Stereo/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/kirill/coding/MyPy/Stereo/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean
.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named DM

# Build rule for target.
DM: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 DM
.PHONY : DM

# fast build rule for target.
DM/fast:
	$(MAKE) -f CMakeFiles/DM.dir/build.make CMakeFiles/DM.dir/build
.PHONY : DM/fast

#=============================================================================
# Target rules for targets named brute

# Build rule for target.
brute: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 brute
.PHONY : brute

# fast build rule for target.
brute/fast:
	$(MAKE) -f CMakeFiles/brute.dir/build.make CMakeFiles/brute.dir/build
.PHONY : brute/fast

#=============================================================================
# Target rules for targets named main

# Build rule for target.
main: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 main
.PHONY : main

# fast build rule for target.
main/fast:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/build
.PHONY : main/fast

#=============================================================================
# Target rules for targets named stereoBMtune

# Build rule for target.
stereoBMtune: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 stereoBMtune
.PHONY : stereoBMtune

# fast build rule for target.
stereoBMtune/fast:
	$(MAKE) -f CMakeFiles/stereoBMtune.dir/build.make CMakeFiles/stereoBMtune.dir/build
.PHONY : stereoBMtune/fast

#=============================================================================
# Target rules for targets named stereoVartune

# Build rule for target.
stereoVartune: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 stereoVartune
.PHONY : stereoVartune

# fast build rule for target.
stereoVartune/fast:
	$(MAKE) -f CMakeFiles/stereoVartune.dir/build.make CMakeFiles/stereoVartune.dir/build
.PHONY : stereoVartune/fast

brute.o: brute.cpp.o
.PHONY : brute.o

# target to build an object file
brute.cpp.o:
	$(MAKE) -f CMakeFiles/brute.dir/build.make CMakeFiles/brute.dir/brute.cpp.o
.PHONY : brute.cpp.o

brute.i: brute.cpp.i
.PHONY : brute.i

# target to preprocess a source file
brute.cpp.i:
	$(MAKE) -f CMakeFiles/brute.dir/build.make CMakeFiles/brute.dir/brute.cpp.i
.PHONY : brute.cpp.i

brute.s: brute.cpp.s
.PHONY : brute.s

# target to generate assembly for a file
brute.cpp.s:
	$(MAKE) -f CMakeFiles/brute.dir/build.make CMakeFiles/brute.dir/brute.cpp.s
.PHONY : brute.cpp.s

depthmap.o: depthmap.cpp.o
.PHONY : depthmap.o

# target to build an object file
depthmap.cpp.o:
	$(MAKE) -f CMakeFiles/DM.dir/build.make CMakeFiles/DM.dir/depthmap.cpp.o
.PHONY : depthmap.cpp.o

depthmap.i: depthmap.cpp.i
.PHONY : depthmap.i

# target to preprocess a source file
depthmap.cpp.i:
	$(MAKE) -f CMakeFiles/DM.dir/build.make CMakeFiles/DM.dir/depthmap.cpp.i
.PHONY : depthmap.cpp.i

depthmap.s: depthmap.cpp.s
.PHONY : depthmap.s

# target to generate assembly for a file
depthmap.cpp.s:
	$(MAKE) -f CMakeFiles/DM.dir/build.make CMakeFiles/DM.dir/depthmap.cpp.s
.PHONY : depthmap.cpp.s

main.o: main.cpp.o
.PHONY : main.o

# target to build an object file
main.cpp.o:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/main.cpp.o
.PHONY : main.cpp.o

main.i: main.cpp.i
.PHONY : main.i

# target to preprocess a source file
main.cpp.i:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/main.cpp.i
.PHONY : main.cpp.i

main.s: main.cpp.s
.PHONY : main.s

# target to generate assembly for a file
main.cpp.s:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/main.cpp.s
.PHONY : main.cpp.s

stereoBMtune.o: stereoBMtune.c.o
.PHONY : stereoBMtune.o

# target to build an object file
stereoBMtune.c.o:
	$(MAKE) -f CMakeFiles/stereoBMtune.dir/build.make CMakeFiles/stereoBMtune.dir/stereoBMtune.c.o
.PHONY : stereoBMtune.c.o

stereoBMtune.i: stereoBMtune.c.i
.PHONY : stereoBMtune.i

# target to preprocess a source file
stereoBMtune.c.i:
	$(MAKE) -f CMakeFiles/stereoBMtune.dir/build.make CMakeFiles/stereoBMtune.dir/stereoBMtune.c.i
.PHONY : stereoBMtune.c.i

stereoBMtune.s: stereoBMtune.c.s
.PHONY : stereoBMtune.s

# target to generate assembly for a file
stereoBMtune.c.s:
	$(MAKE) -f CMakeFiles/stereoBMtune.dir/build.make CMakeFiles/stereoBMtune.dir/stereoBMtune.c.s
.PHONY : stereoBMtune.c.s

stereoVartune.o: stereoVartune.cpp.o
.PHONY : stereoVartune.o

# target to build an object file
stereoVartune.cpp.o:
	$(MAKE) -f CMakeFiles/stereoVartune.dir/build.make CMakeFiles/stereoVartune.dir/stereoVartune.cpp.o
.PHONY : stereoVartune.cpp.o

stereoVartune.i: stereoVartune.cpp.i
.PHONY : stereoVartune.i

# target to preprocess a source file
stereoVartune.cpp.i:
	$(MAKE) -f CMakeFiles/stereoVartune.dir/build.make CMakeFiles/stereoVartune.dir/stereoVartune.cpp.i
.PHONY : stereoVartune.cpp.i

stereoVartune.s: stereoVartune.cpp.s
.PHONY : stereoVartune.s

# target to generate assembly for a file
stereoVartune.cpp.s:
	$(MAKE) -f CMakeFiles/stereoVartune.dir/build.make CMakeFiles/stereoVartune.dir/stereoVartune.cpp.s
.PHONY : stereoVartune.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... DM"
	@echo "... brute"
	@echo "... edit_cache"
	@echo "... main"
	@echo "... rebuild_cache"
	@echo "... stereoBMtune"
	@echo "... stereoVartune"
	@echo "... brute.o"
	@echo "... brute.i"
	@echo "... brute.s"
	@echo "... depthmap.o"
	@echo "... depthmap.i"
	@echo "... depthmap.s"
	@echo "... main.o"
	@echo "... main.i"
	@echo "... main.s"
	@echo "... stereoBMtune.o"
	@echo "... stereoBMtune.i"
	@echo "... stereoBMtune.s"
	@echo "... stereoVartune.o"
	@echo "... stereoVartune.i"
	@echo "... stereoVartune.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

