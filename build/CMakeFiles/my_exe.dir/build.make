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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/samoun/Dropbox/MAM5/PFE/Analyse_video_drone/code/build

# Include any dependencies generated for this target.
include CMakeFiles/my_exe.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/my_exe.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/my_exe.dir/flags.make

CMakeFiles/my_exe.dir/src/Debug/debug.cpp.o: CMakeFiles/my_exe.dir/flags.make
CMakeFiles/my_exe.dir/src/Debug/debug.cpp.o: /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/Debug/debug.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/samoun/Dropbox/MAM5/PFE/Analyse_video_drone/code/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/my_exe.dir/src/Debug/debug.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/my_exe.dir/src/Debug/debug.cpp.o -c /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/Debug/debug.cpp

CMakeFiles/my_exe.dir/src/Debug/debug.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my_exe.dir/src/Debug/debug.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/Debug/debug.cpp > CMakeFiles/my_exe.dir/src/Debug/debug.cpp.i

CMakeFiles/my_exe.dir/src/Debug/debug.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my_exe.dir/src/Debug/debug.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/Debug/debug.cpp -o CMakeFiles/my_exe.dir/src/Debug/debug.cpp.s

CMakeFiles/my_exe.dir/src/Debug/debug.cpp.o.requires:
.PHONY : CMakeFiles/my_exe.dir/src/Debug/debug.cpp.o.requires

CMakeFiles/my_exe.dir/src/Debug/debug.cpp.o.provides: CMakeFiles/my_exe.dir/src/Debug/debug.cpp.o.requires
	$(MAKE) -f CMakeFiles/my_exe.dir/build.make CMakeFiles/my_exe.dir/src/Debug/debug.cpp.o.provides.build
.PHONY : CMakeFiles/my_exe.dir/src/Debug/debug.cpp.o.provides

CMakeFiles/my_exe.dir/src/Debug/debug.cpp.o.provides.build: CMakeFiles/my_exe.dir/src/Debug/debug.cpp.o

CMakeFiles/my_exe.dir/src/Debug/timer.cpp.o: CMakeFiles/my_exe.dir/flags.make
CMakeFiles/my_exe.dir/src/Debug/timer.cpp.o: /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/Debug/timer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/samoun/Dropbox/MAM5/PFE/Analyse_video_drone/code/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/my_exe.dir/src/Debug/timer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/my_exe.dir/src/Debug/timer.cpp.o -c /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/Debug/timer.cpp

CMakeFiles/my_exe.dir/src/Debug/timer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my_exe.dir/src/Debug/timer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/Debug/timer.cpp > CMakeFiles/my_exe.dir/src/Debug/timer.cpp.i

CMakeFiles/my_exe.dir/src/Debug/timer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my_exe.dir/src/Debug/timer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/Debug/timer.cpp -o CMakeFiles/my_exe.dir/src/Debug/timer.cpp.s

CMakeFiles/my_exe.dir/src/Debug/timer.cpp.o.requires:
.PHONY : CMakeFiles/my_exe.dir/src/Debug/timer.cpp.o.requires

CMakeFiles/my_exe.dir/src/Debug/timer.cpp.o.provides: CMakeFiles/my_exe.dir/src/Debug/timer.cpp.o.requires
	$(MAKE) -f CMakeFiles/my_exe.dir/build.make CMakeFiles/my_exe.dir/src/Debug/timer.cpp.o.provides.build
.PHONY : CMakeFiles/my_exe.dir/src/Debug/timer.cpp.o.provides

CMakeFiles/my_exe.dir/src/Debug/timer.cpp.o.provides.build: CMakeFiles/my_exe.dir/src/Debug/timer.cpp.o

CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.o: CMakeFiles/my_exe.dir/flags.make
CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.o: /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/utilities/videoRead.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/samoun/Dropbox/MAM5/PFE/Analyse_video_drone/code/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.o -c /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/utilities/videoRead.cpp

CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/utilities/videoRead.cpp > CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.i

CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/utilities/videoRead.cpp -o CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.s

CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.o.requires:
.PHONY : CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.o.requires

CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.o.provides: CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.o.requires
	$(MAKE) -f CMakeFiles/my_exe.dir/build.make CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.o.provides.build
.PHONY : CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.o.provides

CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.o.provides.build: CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.o

CMakeFiles/my_exe.dir/src/test/main.cpp.o: CMakeFiles/my_exe.dir/flags.make
CMakeFiles/my_exe.dir/src/test/main.cpp.o: /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/test/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/samoun/Dropbox/MAM5/PFE/Analyse_video_drone/code/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/my_exe.dir/src/test/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/my_exe.dir/src/test/main.cpp.o -c /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/test/main.cpp

CMakeFiles/my_exe.dir/src/test/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my_exe.dir/src/test/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/test/main.cpp > CMakeFiles/my_exe.dir/src/test/main.cpp.i

CMakeFiles/my_exe.dir/src/test/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my_exe.dir/src/test/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/test/main.cpp -o CMakeFiles/my_exe.dir/src/test/main.cpp.s

CMakeFiles/my_exe.dir/src/test/main.cpp.o.requires:
.PHONY : CMakeFiles/my_exe.dir/src/test/main.cpp.o.requires

CMakeFiles/my_exe.dir/src/test/main.cpp.o.provides: CMakeFiles/my_exe.dir/src/test/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/my_exe.dir/build.make CMakeFiles/my_exe.dir/src/test/main.cpp.o.provides.build
.PHONY : CMakeFiles/my_exe.dir/src/test/main.cpp.o.provides

CMakeFiles/my_exe.dir/src/test/main.cpp.o.provides.build: CMakeFiles/my_exe.dir/src/test/main.cpp.o

CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.o: CMakeFiles/my_exe.dir/flags.make
CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.o: /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/core/features/stabilization.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/samoun/Dropbox/MAM5/PFE/Analyse_video_drone/code/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.o -c /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/core/features/stabilization.cpp

CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/core/features/stabilization.cpp > CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.i

CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/core/features/stabilization.cpp -o CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.s

CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.o.requires:
.PHONY : CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.o.requires

CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.o.provides: CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.o.requires
	$(MAKE) -f CMakeFiles/my_exe.dir/build.make CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.o.provides.build
.PHONY : CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.o.provides

CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.o.provides.build: CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.o

CMakeFiles/my_exe.dir/src/core/image.cpp.o: CMakeFiles/my_exe.dir/flags.make
CMakeFiles/my_exe.dir/src/core/image.cpp.o: /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/core/image.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/samoun/Dropbox/MAM5/PFE/Analyse_video_drone/code/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/my_exe.dir/src/core/image.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/my_exe.dir/src/core/image.cpp.o -c /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/core/image.cpp

CMakeFiles/my_exe.dir/src/core/image.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my_exe.dir/src/core/image.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/core/image.cpp > CMakeFiles/my_exe.dir/src/core/image.cpp.i

CMakeFiles/my_exe.dir/src/core/image.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my_exe.dir/src/core/image.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/src/core/image.cpp -o CMakeFiles/my_exe.dir/src/core/image.cpp.s

CMakeFiles/my_exe.dir/src/core/image.cpp.o.requires:
.PHONY : CMakeFiles/my_exe.dir/src/core/image.cpp.o.requires

CMakeFiles/my_exe.dir/src/core/image.cpp.o.provides: CMakeFiles/my_exe.dir/src/core/image.cpp.o.requires
	$(MAKE) -f CMakeFiles/my_exe.dir/build.make CMakeFiles/my_exe.dir/src/core/image.cpp.o.provides.build
.PHONY : CMakeFiles/my_exe.dir/src/core/image.cpp.o.provides

CMakeFiles/my_exe.dir/src/core/image.cpp.o.provides.build: CMakeFiles/my_exe.dir/src/core/image.cpp.o

# Object files for target my_exe
my_exe_OBJECTS = \
"CMakeFiles/my_exe.dir/src/Debug/debug.cpp.o" \
"CMakeFiles/my_exe.dir/src/Debug/timer.cpp.o" \
"CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.o" \
"CMakeFiles/my_exe.dir/src/test/main.cpp.o" \
"CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.o" \
"CMakeFiles/my_exe.dir/src/core/image.cpp.o"

# External object files for target my_exe
my_exe_EXTERNAL_OBJECTS =

/home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/bin/my_exe: CMakeFiles/my_exe.dir/src/Debug/debug.cpp.o
/home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/bin/my_exe: CMakeFiles/my_exe.dir/src/Debug/timer.cpp.o
/home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/bin/my_exe: CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.o
/home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/bin/my_exe: CMakeFiles/my_exe.dir/src/test/main.cpp.o
/home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/bin/my_exe: CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.o
/home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/bin/my_exe: CMakeFiles/my_exe.dir/src/core/image.cpp.o
/home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/bin/my_exe: CMakeFiles/my_exe.dir/build.make
/home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/bin/my_exe: CMakeFiles/my_exe.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/bin/my_exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/my_exe.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/my_exe.dir/build: /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code/bin/my_exe
.PHONY : CMakeFiles/my_exe.dir/build

CMakeFiles/my_exe.dir/requires: CMakeFiles/my_exe.dir/src/Debug/debug.cpp.o.requires
CMakeFiles/my_exe.dir/requires: CMakeFiles/my_exe.dir/src/Debug/timer.cpp.o.requires
CMakeFiles/my_exe.dir/requires: CMakeFiles/my_exe.dir/src/utilities/videoRead.cpp.o.requires
CMakeFiles/my_exe.dir/requires: CMakeFiles/my_exe.dir/src/test/main.cpp.o.requires
CMakeFiles/my_exe.dir/requires: CMakeFiles/my_exe.dir/src/core/features/stabilization.cpp.o.requires
CMakeFiles/my_exe.dir/requires: CMakeFiles/my_exe.dir/src/core/image.cpp.o.requires
.PHONY : CMakeFiles/my_exe.dir/requires

CMakeFiles/my_exe.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/my_exe.dir/cmake_clean.cmake
.PHONY : CMakeFiles/my_exe.dir/clean

CMakeFiles/my_exe.dir/depend:
	cd /home/samoun/Dropbox/MAM5/PFE/Analyse_video_drone/code/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code /home/user/Dropbox/MAM5/PFE/Analyse_video_drone/code /home/samoun/Dropbox/MAM5/PFE/Analyse_video_drone/code/build /home/samoun/Dropbox/MAM5/PFE/Analyse_video_drone/code/build /home/samoun/Dropbox/MAM5/PFE/Analyse_video_drone/code/build/CMakeFiles/my_exe.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/my_exe.dir/depend

