# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_COMMAND = /home/ubuntu/文档/clion-2016.3.3/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/ubuntu/文档/clion-2016.3.3/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/ubuntu/桌面/0708/multi_0708 stable"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug"

# Include any dependencies generated for this target.
include src/CMakeFiles/tag.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/tag.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/tag.dir/flags.make

src/CMakeFiles/tag.dir/tag.cpp.o: src/CMakeFiles/tag.dir/flags.make
src/CMakeFiles/tag.dir/tag.cpp.o: ../src/tag.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/tag.dir/tag.cpp.o"
	cd "/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug/src" && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tag.dir/tag.cpp.o -c "/home/ubuntu/桌面/0708/multi_0708 stable/src/tag.cpp"

src/CMakeFiles/tag.dir/tag.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tag.dir/tag.cpp.i"
	cd "/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug/src" && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/ubuntu/桌面/0708/multi_0708 stable/src/tag.cpp" > CMakeFiles/tag.dir/tag.cpp.i

src/CMakeFiles/tag.dir/tag.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tag.dir/tag.cpp.s"
	cd "/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug/src" && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/ubuntu/桌面/0708/multi_0708 stable/src/tag.cpp" -o CMakeFiles/tag.dir/tag.cpp.s

src/CMakeFiles/tag.dir/tag.cpp.o.requires:

.PHONY : src/CMakeFiles/tag.dir/tag.cpp.o.requires

src/CMakeFiles/tag.dir/tag.cpp.o.provides: src/CMakeFiles/tag.dir/tag.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/tag.dir/build.make src/CMakeFiles/tag.dir/tag.cpp.o.provides.build
.PHONY : src/CMakeFiles/tag.dir/tag.cpp.o.provides

src/CMakeFiles/tag.dir/tag.cpp.o.provides.build: src/CMakeFiles/tag.dir/tag.cpp.o


src/CMakeFiles/tag.dir/taginfo.cpp.o: src/CMakeFiles/tag.dir/flags.make
src/CMakeFiles/tag.dir/taginfo.cpp.o: ../src/taginfo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/tag.dir/taginfo.cpp.o"
	cd "/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug/src" && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tag.dir/taginfo.cpp.o -c "/home/ubuntu/桌面/0708/multi_0708 stable/src/taginfo.cpp"

src/CMakeFiles/tag.dir/taginfo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tag.dir/taginfo.cpp.i"
	cd "/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug/src" && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/ubuntu/桌面/0708/multi_0708 stable/src/taginfo.cpp" > CMakeFiles/tag.dir/taginfo.cpp.i

src/CMakeFiles/tag.dir/taginfo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tag.dir/taginfo.cpp.s"
	cd "/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug/src" && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/ubuntu/桌面/0708/multi_0708 stable/src/taginfo.cpp" -o CMakeFiles/tag.dir/taginfo.cpp.s

src/CMakeFiles/tag.dir/taginfo.cpp.o.requires:

.PHONY : src/CMakeFiles/tag.dir/taginfo.cpp.o.requires

src/CMakeFiles/tag.dir/taginfo.cpp.o.provides: src/CMakeFiles/tag.dir/taginfo.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/tag.dir/build.make src/CMakeFiles/tag.dir/taginfo.cpp.o.provides.build
.PHONY : src/CMakeFiles/tag.dir/taginfo.cpp.o.provides

src/CMakeFiles/tag.dir/taginfo.cpp.o.provides.build: src/CMakeFiles/tag.dir/taginfo.cpp.o


src/CMakeFiles/tag.dir/kalman.cpp.o: src/CMakeFiles/tag.dir/flags.make
src/CMakeFiles/tag.dir/kalman.cpp.o: ../src/kalman.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/CMakeFiles/tag.dir/kalman.cpp.o"
	cd "/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug/src" && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tag.dir/kalman.cpp.o -c "/home/ubuntu/桌面/0708/multi_0708 stable/src/kalman.cpp"

src/CMakeFiles/tag.dir/kalman.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tag.dir/kalman.cpp.i"
	cd "/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug/src" && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/ubuntu/桌面/0708/multi_0708 stable/src/kalman.cpp" > CMakeFiles/tag.dir/kalman.cpp.i

src/CMakeFiles/tag.dir/kalman.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tag.dir/kalman.cpp.s"
	cd "/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug/src" && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/ubuntu/桌面/0708/multi_0708 stable/src/kalman.cpp" -o CMakeFiles/tag.dir/kalman.cpp.s

src/CMakeFiles/tag.dir/kalman.cpp.o.requires:

.PHONY : src/CMakeFiles/tag.dir/kalman.cpp.o.requires

src/CMakeFiles/tag.dir/kalman.cpp.o.provides: src/CMakeFiles/tag.dir/kalman.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/tag.dir/build.make src/CMakeFiles/tag.dir/kalman.cpp.o.provides.build
.PHONY : src/CMakeFiles/tag.dir/kalman.cpp.o.provides

src/CMakeFiles/tag.dir/kalman.cpp.o.provides.build: src/CMakeFiles/tag.dir/kalman.cpp.o


# Object files for target tag
tag_OBJECTS = \
"CMakeFiles/tag.dir/tag.cpp.o" \
"CMakeFiles/tag.dir/taginfo.cpp.o" \
"CMakeFiles/tag.dir/kalman.cpp.o"

# External object files for target tag
tag_EXTERNAL_OBJECTS =

../lib/libtag.a: src/CMakeFiles/tag.dir/tag.cpp.o
../lib/libtag.a: src/CMakeFiles/tag.dir/taginfo.cpp.o
../lib/libtag.a: src/CMakeFiles/tag.dir/kalman.cpp.o
../lib/libtag.a: src/CMakeFiles/tag.dir/build.make
../lib/libtag.a: src/CMakeFiles/tag.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library ../../lib/libtag.a"
	cd "/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug/src" && $(CMAKE_COMMAND) -P CMakeFiles/tag.dir/cmake_clean_target.cmake
	cd "/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug/src" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tag.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/tag.dir/build: ../lib/libtag.a

.PHONY : src/CMakeFiles/tag.dir/build

src/CMakeFiles/tag.dir/requires: src/CMakeFiles/tag.dir/tag.cpp.o.requires
src/CMakeFiles/tag.dir/requires: src/CMakeFiles/tag.dir/taginfo.cpp.o.requires
src/CMakeFiles/tag.dir/requires: src/CMakeFiles/tag.dir/kalman.cpp.o.requires

.PHONY : src/CMakeFiles/tag.dir/requires

src/CMakeFiles/tag.dir/clean:
	cd "/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug/src" && $(CMAKE_COMMAND) -P CMakeFiles/tag.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/tag.dir/clean

src/CMakeFiles/tag.dir/depend:
	cd "/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/ubuntu/桌面/0708/multi_0708 stable" "/home/ubuntu/桌面/0708/multi_0708 stable/src" "/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug" "/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug/src" "/home/ubuntu/桌面/0708/multi_0708 stable/cmake-build-debug/src/CMakeFiles/tag.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : src/CMakeFiles/tag.dir/depend

