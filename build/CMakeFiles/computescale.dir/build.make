# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zy/SFM_Data/timesample

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zy/SFM_Data/timesample/build

# Include any dependencies generated for this target.
include CMakeFiles/computescale.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/computescale.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/computescale.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/computescale.dir/flags.make

CMakeFiles/computescale.dir/ComputeScale.cpp.o: CMakeFiles/computescale.dir/flags.make
CMakeFiles/computescale.dir/ComputeScale.cpp.o: /home/zy/SFM_Data/timesample/ComputeScale.cpp
CMakeFiles/computescale.dir/ComputeScale.cpp.o: CMakeFiles/computescale.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zy/SFM_Data/timesample/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/computescale.dir/ComputeScale.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/computescale.dir/ComputeScale.cpp.o -MF CMakeFiles/computescale.dir/ComputeScale.cpp.o.d -o CMakeFiles/computescale.dir/ComputeScale.cpp.o -c /home/zy/SFM_Data/timesample/ComputeScale.cpp

CMakeFiles/computescale.dir/ComputeScale.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/computescale.dir/ComputeScale.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zy/SFM_Data/timesample/ComputeScale.cpp > CMakeFiles/computescale.dir/ComputeScale.cpp.i

CMakeFiles/computescale.dir/ComputeScale.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/computescale.dir/ComputeScale.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zy/SFM_Data/timesample/ComputeScale.cpp -o CMakeFiles/computescale.dir/ComputeScale.cpp.s

# Object files for target computescale
computescale_OBJECTS = \
"CMakeFiles/computescale.dir/ComputeScale.cpp.o"

# External object files for target computescale
computescale_EXTERNAL_OBJECTS =

computescale: CMakeFiles/computescale.dir/ComputeScale.cpp.o
computescale: CMakeFiles/computescale.dir/build.make
computescale: /usr/local/lib/libopencv_dnn.so.4.2.0
computescale: /usr/local/lib/libopencv_gapi.so.4.2.0
computescale: /usr/local/lib/libopencv_highgui.so.4.2.0
computescale: /usr/local/lib/libopencv_ml.so.4.2.0
computescale: /usr/local/lib/libopencv_objdetect.so.4.2.0
computescale: /usr/local/lib/libopencv_photo.so.4.2.0
computescale: /usr/local/lib/libopencv_stitching.so.4.2.0
computescale: /usr/local/lib/libopencv_video.so.4.2.0
computescale: /usr/local/lib/libopencv_videoio.so.4.2.0
computescale: /usr/local/lib/libopencv_imgcodecs.so.4.2.0
computescale: /usr/local/lib/libopencv_calib3d.so.4.2.0
computescale: /usr/local/lib/libopencv_features2d.so.4.2.0
computescale: /usr/local/lib/libopencv_flann.so.4.2.0
computescale: /usr/local/lib/libopencv_imgproc.so.4.2.0
computescale: /usr/local/lib/libopencv_core.so.4.2.0
computescale: CMakeFiles/computescale.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zy/SFM_Data/timesample/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable computescale"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/computescale.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/computescale.dir/build: computescale
.PHONY : CMakeFiles/computescale.dir/build

CMakeFiles/computescale.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/computescale.dir/cmake_clean.cmake
.PHONY : CMakeFiles/computescale.dir/clean

CMakeFiles/computescale.dir/depend:
	cd /home/zy/SFM_Data/timesample/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zy/SFM_Data/timesample /home/zy/SFM_Data/timesample /home/zy/SFM_Data/timesample/build /home/zy/SFM_Data/timesample/build /home/zy/SFM_Data/timesample/build/CMakeFiles/computescale.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/computescale.dir/depend

