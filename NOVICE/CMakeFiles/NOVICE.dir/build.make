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
CMAKE_SOURCE_DIR = /home/graham/docs/codeBase/NOVICE

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/graham/docs/codeBase/NOVICE

# Include any dependencies generated for this target.
include CMakeFiles/NOVICE.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/NOVICE.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/NOVICE.dir/flags.make

CMakeFiles/NOVICE.dir/NOVICE.cpp.o: CMakeFiles/NOVICE.dir/flags.make
CMakeFiles/NOVICE.dir/NOVICE.cpp.o: NOVICE.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/graham/docs/codeBase/NOVICE/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/NOVICE.dir/NOVICE.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NOVICE.dir/NOVICE.cpp.o -c /home/graham/docs/codeBase/NOVICE/NOVICE.cpp

CMakeFiles/NOVICE.dir/NOVICE.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NOVICE.dir/NOVICE.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/graham/docs/codeBase/NOVICE/NOVICE.cpp > CMakeFiles/NOVICE.dir/NOVICE.cpp.i

CMakeFiles/NOVICE.dir/NOVICE.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NOVICE.dir/NOVICE.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/graham/docs/codeBase/NOVICE/NOVICE.cpp -o CMakeFiles/NOVICE.dir/NOVICE.cpp.s

CMakeFiles/NOVICE.dir/util.cpp.o: CMakeFiles/NOVICE.dir/flags.make
CMakeFiles/NOVICE.dir/util.cpp.o: util.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/graham/docs/codeBase/NOVICE/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/NOVICE.dir/util.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NOVICE.dir/util.cpp.o -c /home/graham/docs/codeBase/NOVICE/util.cpp

CMakeFiles/NOVICE.dir/util.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NOVICE.dir/util.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/graham/docs/codeBase/NOVICE/util.cpp > CMakeFiles/NOVICE.dir/util.cpp.i

CMakeFiles/NOVICE.dir/util.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NOVICE.dir/util.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/graham/docs/codeBase/NOVICE/util.cpp -o CMakeFiles/NOVICE.dir/util.cpp.s

# Object files for target NOVICE
NOVICE_OBJECTS = \
"CMakeFiles/NOVICE.dir/NOVICE.cpp.o" \
"CMakeFiles/NOVICE.dir/util.cpp.o"

# External object files for target NOVICE
NOVICE_EXTERNAL_OBJECTS =

NOVICE: CMakeFiles/NOVICE.dir/NOVICE.cpp.o
NOVICE: CMakeFiles/NOVICE.dir/util.cpp.o
NOVICE: CMakeFiles/NOVICE.dir/build.make
NOVICE: CMakeFiles/NOVICE.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/graham/docs/codeBase/NOVICE/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable NOVICE"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/NOVICE.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/NOVICE.dir/build: NOVICE

.PHONY : CMakeFiles/NOVICE.dir/build

CMakeFiles/NOVICE.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/NOVICE.dir/cmake_clean.cmake
.PHONY : CMakeFiles/NOVICE.dir/clean

CMakeFiles/NOVICE.dir/depend:
	cd /home/graham/docs/codeBase/NOVICE && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/graham/docs/codeBase/NOVICE /home/graham/docs/codeBase/NOVICE /home/graham/docs/codeBase/NOVICE /home/graham/docs/codeBase/NOVICE /home/graham/docs/codeBase/NOVICE/CMakeFiles/NOVICE.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/NOVICE.dir/depend
