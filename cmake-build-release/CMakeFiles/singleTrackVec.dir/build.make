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
CMAKE_BINARY_DIR = /Users/riccardodona/CLionProjects/testACADO/cmake-build-release

# Include any dependencies generated for this target.
include CMakeFiles/singleTrackVec.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/singleTrackVec.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/singleTrackVec.dir/flags.make

CMakeFiles/singleTrackVec.dir/tests/single_track_vector_ref.cpp.o: CMakeFiles/singleTrackVec.dir/flags.make
CMakeFiles/singleTrackVec.dir/tests/single_track_vector_ref.cpp.o: ../tests/single_track_vector_ref.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/riccardodona/CLionProjects/testACADO/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/singleTrackVec.dir/tests/single_track_vector_ref.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/singleTrackVec.dir/tests/single_track_vector_ref.cpp.o -c /Users/riccardodona/CLionProjects/testACADO/tests/single_track_vector_ref.cpp

CMakeFiles/singleTrackVec.dir/tests/single_track_vector_ref.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/singleTrackVec.dir/tests/single_track_vector_ref.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/riccardodona/CLionProjects/testACADO/tests/single_track_vector_ref.cpp > CMakeFiles/singleTrackVec.dir/tests/single_track_vector_ref.cpp.i

CMakeFiles/singleTrackVec.dir/tests/single_track_vector_ref.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/singleTrackVec.dir/tests/single_track_vector_ref.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/riccardodona/CLionProjects/testACADO/tests/single_track_vector_ref.cpp -o CMakeFiles/singleTrackVec.dir/tests/single_track_vector_ref.cpp.s

# Object files for target singleTrackVec
singleTrackVec_OBJECTS = \
"CMakeFiles/singleTrackVec.dir/tests/single_track_vector_ref.cpp.o"

# External object files for target singleTrackVec
singleTrackVec_EXTERNAL_OBJECTS =

../bin/singleTrackVec: CMakeFiles/singleTrackVec.dir/tests/single_track_vector_ref.cpp.o
../bin/singleTrackVec: CMakeFiles/singleTrackVec.dir/build.make
../bin/singleTrackVec: /Users/riccardodona/Documents/University/PhD/repositories/ACADOtoolkit/build/lib/libacado_toolkit_s.dylib
../bin/singleTrackVec: CMakeFiles/singleTrackVec.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/riccardodona/CLionProjects/testACADO/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/singleTrackVec"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/singleTrackVec.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/singleTrackVec.dir/build: ../bin/singleTrackVec

.PHONY : CMakeFiles/singleTrackVec.dir/build

CMakeFiles/singleTrackVec.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/singleTrackVec.dir/cmake_clean.cmake
.PHONY : CMakeFiles/singleTrackVec.dir/clean

CMakeFiles/singleTrackVec.dir/depend:
	cd /Users/riccardodona/CLionProjects/testACADO/cmake-build-release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/riccardodona/CLionProjects/testACADO /Users/riccardodona/CLionProjects/testACADO /Users/riccardodona/CLionProjects/testACADO/cmake-build-release /Users/riccardodona/CLionProjects/testACADO/cmake-build-release /Users/riccardodona/CLionProjects/testACADO/cmake-build-release/CMakeFiles/singleTrackVec.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/singleTrackVec.dir/depend

