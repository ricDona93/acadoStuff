# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/riccardodona/CLionProjects/testACADO

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/riccardodona/CLionProjects/testACADO/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/testACADO.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/testACADO.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/testACADO.dir/flags.make

CMakeFiles/testACADO.dir/tests/main.cpp.o: CMakeFiles/testACADO.dir/flags.make
CMakeFiles/testACADO.dir/tests/main.cpp.o: ../tests/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/riccardodona/CLionProjects/testACADO/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/testACADO.dir/tests/main.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testACADO.dir/tests/main.cpp.o -c /Users/riccardodona/CLionProjects/testACADO/tests/main.cpp

CMakeFiles/testACADO.dir/tests/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testACADO.dir/tests/main.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/riccardodona/CLionProjects/testACADO/tests/main.cpp > CMakeFiles/testACADO.dir/tests/main.cpp.i

CMakeFiles/testACADO.dir/tests/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testACADO.dir/tests/main.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/riccardodona/CLionProjects/testACADO/tests/main.cpp -o CMakeFiles/testACADO.dir/tests/main.cpp.s

# Object files for target testACADO
testACADO_OBJECTS = \
"CMakeFiles/testACADO.dir/tests/main.cpp.o"

# External object files for target testACADO
testACADO_EXTERNAL_OBJECTS =

../bin/testACADO: CMakeFiles/testACADO.dir/tests/main.cpp.o
../bin/testACADO: CMakeFiles/testACADO.dir/build.make
../bin/testACADO: /Users/riccardodona/Documents/University/PhD/repositories/ACADOtoolkit/build/lib/libacado_toolkit_s.dylib
../bin/testACADO: CMakeFiles/testACADO.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/riccardodona/CLionProjects/testACADO/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/testACADO"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testACADO.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/testACADO.dir/build: ../bin/testACADO

.PHONY : CMakeFiles/testACADO.dir/build

CMakeFiles/testACADO.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/testACADO.dir/cmake_clean.cmake
.PHONY : CMakeFiles/testACADO.dir/clean

CMakeFiles/testACADO.dir/depend:
	cd /Users/riccardodona/CLionProjects/testACADO/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/riccardodona/CLionProjects/testACADO /Users/riccardodona/CLionProjects/testACADO /Users/riccardodona/CLionProjects/testACADO/cmake-build-debug /Users/riccardodona/CLionProjects/testACADO/cmake-build-debug /Users/riccardodona/CLionProjects/testACADO/cmake-build-debug/CMakeFiles/testACADO.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/testACADO.dir/depend
