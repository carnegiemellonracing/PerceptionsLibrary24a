# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.31

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
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.31.0/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.31.0/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/beeboss/Desktop/CMR/perceptions/PerceptionsLibrary24a

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/beeboss/Desktop/CMR/perceptions/PerceptionsLibrary24a

# Include any dependencies generated for this target.
include CMakeFiles/simple-c-setup.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/simple-c-setup.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/simple-c-setup.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/simple-c-setup.dir/flags.make

CMakeFiles/simple-c-setup.dir/codegen:
.PHONY : CMakeFiles/simple-c-setup.dir/codegen

CMakeFiles/simple-c-setup.dir/src/main.cc.o: CMakeFiles/simple-c-setup.dir/flags.make
CMakeFiles/simple-c-setup.dir/src/main.cc.o: src/main.cc
CMakeFiles/simple-c-setup.dir/src/main.cc.o: CMakeFiles/simple-c-setup.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/beeboss/Desktop/CMR/perceptions/PerceptionsLibrary24a/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/simple-c-setup.dir/src/main.cc.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/simple-c-setup.dir/src/main.cc.o -MF CMakeFiles/simple-c-setup.dir/src/main.cc.o.d -o CMakeFiles/simple-c-setup.dir/src/main.cc.o -c /Users/beeboss/Desktop/CMR/perceptions/PerceptionsLibrary24a/src/main.cc

CMakeFiles/simple-c-setup.dir/src/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/simple-c-setup.dir/src/main.cc.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/beeboss/Desktop/CMR/perceptions/PerceptionsLibrary24a/src/main.cc > CMakeFiles/simple-c-setup.dir/src/main.cc.i

CMakeFiles/simple-c-setup.dir/src/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/simple-c-setup.dir/src/main.cc.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/beeboss/Desktop/CMR/perceptions/PerceptionsLibrary24a/src/main.cc -o CMakeFiles/simple-c-setup.dir/src/main.cc.s

# Object files for target simple-c-setup
simple__c__setup_OBJECTS = \
"CMakeFiles/simple-c-setup.dir/src/main.cc.o"

# External object files for target simple-c-setup
simple__c__setup_EXTERNAL_OBJECTS =

simple-c-setup: CMakeFiles/simple-c-setup.dir/src/main.cc.o
simple-c-setup: CMakeFiles/simple-c-setup.dir/build.make
simple-c-setup: libmath.a
simple-c-setup: CMakeFiles/simple-c-setup.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/beeboss/Desktop/CMR/perceptions/PerceptionsLibrary24a/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable simple-c-setup"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simple-c-setup.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/simple-c-setup.dir/build: simple-c-setup
.PHONY : CMakeFiles/simple-c-setup.dir/build

CMakeFiles/simple-c-setup.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/simple-c-setup.dir/cmake_clean.cmake
.PHONY : CMakeFiles/simple-c-setup.dir/clean

CMakeFiles/simple-c-setup.dir/depend:
	cd /Users/beeboss/Desktop/CMR/perceptions/PerceptionsLibrary24a && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/beeboss/Desktop/CMR/perceptions/PerceptionsLibrary24a /Users/beeboss/Desktop/CMR/perceptions/PerceptionsLibrary24a /Users/beeboss/Desktop/CMR/perceptions/PerceptionsLibrary24a /Users/beeboss/Desktop/CMR/perceptions/PerceptionsLibrary24a /Users/beeboss/Desktop/CMR/perceptions/PerceptionsLibrary24a/CMakeFiles/simple-c-setup.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/simple-c-setup.dir/depend

