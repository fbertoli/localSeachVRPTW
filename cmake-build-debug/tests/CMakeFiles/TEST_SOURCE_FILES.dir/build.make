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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/ber253/projects/CLion/localSearch

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/ber253/projects/CLion/localSearch/cmake-build-debug

# Include any dependencies generated for this target.
include tests/CMakeFiles/TEST_SOURCE_FILES.dir/depend.make

# Include the progress variables for this target.
include tests/CMakeFiles/TEST_SOURCE_FILES.dir/progress.make

# Include the compile flags for this target's objects.
include tests/CMakeFiles/TEST_SOURCE_FILES.dir/flags.make

tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.o: tests/CMakeFiles/TEST_SOURCE_FILES.dir/flags.make
tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.o: ../tests/testMains/testSolution.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/ber253/projects/CLion/localSearch/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.o"
	cd /Users/ber253/projects/CLion/localSearch/cmake-build-debug/tests && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.o -c /Users/ber253/projects/CLion/localSearch/tests/testMains/testSolution.cpp

tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.i"
	cd /Users/ber253/projects/CLion/localSearch/cmake-build-debug/tests && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/ber253/projects/CLion/localSearch/tests/testMains/testSolution.cpp > CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.i

tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.s"
	cd /Users/ber253/projects/CLion/localSearch/cmake-build-debug/tests && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/ber253/projects/CLion/localSearch/tests/testMains/testSolution.cpp -o CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.s

tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.o.requires:

.PHONY : tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.o.requires

tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.o.provides: tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.o.requires
	$(MAKE) -f tests/CMakeFiles/TEST_SOURCE_FILES.dir/build.make tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.o.provides.build
.PHONY : tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.o.provides

tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.o.provides.build: tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.o


tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.o: tests/CMakeFiles/TEST_SOURCE_FILES.dir/flags.make
tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.o: ../tests/testMains/testRelocate.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/ber253/projects/CLion/localSearch/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.o"
	cd /Users/ber253/projects/CLion/localSearch/cmake-build-debug/tests && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.o -c /Users/ber253/projects/CLion/localSearch/tests/testMains/testRelocate.cpp

tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.i"
	cd /Users/ber253/projects/CLion/localSearch/cmake-build-debug/tests && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/ber253/projects/CLion/localSearch/tests/testMains/testRelocate.cpp > CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.i

tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.s"
	cd /Users/ber253/projects/CLion/localSearch/cmake-build-debug/tests && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/ber253/projects/CLion/localSearch/tests/testMains/testRelocate.cpp -o CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.s

tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.o.requires:

.PHONY : tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.o.requires

tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.o.provides: tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.o.requires
	$(MAKE) -f tests/CMakeFiles/TEST_SOURCE_FILES.dir/build.make tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.o.provides.build
.PHONY : tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.o.provides

tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.o.provides.build: tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.o


tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.o: tests/CMakeFiles/TEST_SOURCE_FILES.dir/flags.make
tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.o: ../tests/testMains/testCross.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/ber253/projects/CLion/localSearch/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.o"
	cd /Users/ber253/projects/CLion/localSearch/cmake-build-debug/tests && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.o -c /Users/ber253/projects/CLion/localSearch/tests/testMains/testCross.cpp

tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.i"
	cd /Users/ber253/projects/CLion/localSearch/cmake-build-debug/tests && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/ber253/projects/CLion/localSearch/tests/testMains/testCross.cpp > CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.i

tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.s"
	cd /Users/ber253/projects/CLion/localSearch/cmake-build-debug/tests && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/ber253/projects/CLion/localSearch/tests/testMains/testCross.cpp -o CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.s

tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.o.requires:

.PHONY : tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.o.requires

tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.o.provides: tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.o.requires
	$(MAKE) -f tests/CMakeFiles/TEST_SOURCE_FILES.dir/build.make tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.o.provides.build
.PHONY : tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.o.provides

tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.o.provides.build: tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.o


# Object files for target TEST_SOURCE_FILES
TEST_SOURCE_FILES_OBJECTS = \
"CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.o" \
"CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.o" \
"CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.o"

# External object files for target TEST_SOURCE_FILES
TEST_SOURCE_FILES_EXTERNAL_OBJECTS =

tests/libTEST_SOURCE_FILES.a: tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.o
tests/libTEST_SOURCE_FILES.a: tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.o
tests/libTEST_SOURCE_FILES.a: tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.o
tests/libTEST_SOURCE_FILES.a: tests/CMakeFiles/TEST_SOURCE_FILES.dir/build.make
tests/libTEST_SOURCE_FILES.a: tests/CMakeFiles/TEST_SOURCE_FILES.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/ber253/projects/CLion/localSearch/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libTEST_SOURCE_FILES.a"
	cd /Users/ber253/projects/CLion/localSearch/cmake-build-debug/tests && $(CMAKE_COMMAND) -P CMakeFiles/TEST_SOURCE_FILES.dir/cmake_clean_target.cmake
	cd /Users/ber253/projects/CLion/localSearch/cmake-build-debug/tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TEST_SOURCE_FILES.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tests/CMakeFiles/TEST_SOURCE_FILES.dir/build: tests/libTEST_SOURCE_FILES.a

.PHONY : tests/CMakeFiles/TEST_SOURCE_FILES.dir/build

tests/CMakeFiles/TEST_SOURCE_FILES.dir/requires: tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testSolution.cpp.o.requires
tests/CMakeFiles/TEST_SOURCE_FILES.dir/requires: tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testRelocate.cpp.o.requires
tests/CMakeFiles/TEST_SOURCE_FILES.dir/requires: tests/CMakeFiles/TEST_SOURCE_FILES.dir/testMains/testCross.cpp.o.requires

.PHONY : tests/CMakeFiles/TEST_SOURCE_FILES.dir/requires

tests/CMakeFiles/TEST_SOURCE_FILES.dir/clean:
	cd /Users/ber253/projects/CLion/localSearch/cmake-build-debug/tests && $(CMAKE_COMMAND) -P CMakeFiles/TEST_SOURCE_FILES.dir/cmake_clean.cmake
.PHONY : tests/CMakeFiles/TEST_SOURCE_FILES.dir/clean

tests/CMakeFiles/TEST_SOURCE_FILES.dir/depend:
	cd /Users/ber253/projects/CLion/localSearch/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/ber253/projects/CLion/localSearch /Users/ber253/projects/CLion/localSearch/tests /Users/ber253/projects/CLion/localSearch/cmake-build-debug /Users/ber253/projects/CLion/localSearch/cmake-build-debug/tests /Users/ber253/projects/CLion/localSearch/cmake-build-debug/tests/CMakeFiles/TEST_SOURCE_FILES.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tests/CMakeFiles/TEST_SOURCE_FILES.dir/depend

