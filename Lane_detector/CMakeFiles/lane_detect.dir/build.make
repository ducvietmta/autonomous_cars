# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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
CMAKE_SOURCE_DIR = /home/ubuntu/Lane_detector

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/Lane_detector

# Include any dependencies generated for this target.
include CMakeFiles/lane_detect.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/lane_detect.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lane_detect.dir/flags.make

CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.o: CMakeFiles/lane_detect.dir/flags.make
CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.o: main_pid/lane_detection.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ubuntu/Lane_detector/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.o -c /home/ubuntu/Lane_detector/main_pid/lane_detection.cpp

CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ubuntu/Lane_detector/main_pid/lane_detection.cpp > CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.i

CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ubuntu/Lane_detector/main_pid/lane_detection.cpp -o CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.s

CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.o.requires:
.PHONY : CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.o.requires

CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.o.provides: CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.o.requires
	$(MAKE) -f CMakeFiles/lane_detect.dir/build.make CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.o.provides.build
.PHONY : CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.o.provides

CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.o.provides.build: CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.o

# Object files for target lane_detect
lane_detect_OBJECTS = \
"CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.o"

# External object files for target lane_detect
lane_detect_EXTERNAL_OBJECTS =

bin/Release/lane_detect: CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.o
bin/Release/lane_detect: CMakeFiles/lane_detect.dir/build.make
bin/Release/lane_detect: /usr/lib/libopencv_vstab.so.2.4.13
bin/Release/lane_detect: /usr/lib/libopencv_imuvstab.so.2.4.13
bin/Release/lane_detect: /usr/lib/libopencv_facedetect.so.2.4.13
bin/Release/lane_detect: /usr/lib/libopencv_esm_panorama.so.2.4.13
bin/Release/lane_detect: /usr/lib/libopencv_detection_based_tracker.so.2.4.13
bin/Release/lane_detect: /usr/lib/libopencv_videostab.so.2.4.13
bin/Release/lane_detect: /usr/lib/libopencv_ts.a
bin/Release/lane_detect: /usr/lib/libopencv_superres.so.2.4.13
bin/Release/lane_detect: /usr/lib/libopencv_contrib.so.2.4.13
bin/Release/lane_detect: bin/Release/libuart.a
bin/Release/lane_detect: bin/Release/libsign_detect.a
bin/Release/lane_detect: /usr/lib/libopencv_tegra.so.2.4.13
bin/Release/lane_detect: /usr/lib/libopencv_stitching.so.2.4.13
bin/Release/lane_detect: /usr/lib/libopencv_gpu.so.2.4.13
bin/Release/lane_detect: /usr/lib/libopencv_photo.so.2.4.13
bin/Release/lane_detect: /usr/lib/libopencv_legacy.so.2.4.13
bin/Release/lane_detect: /usr/local/cuda-6.5/lib/libcufft.so
bin/Release/lane_detect: /usr/lib/libopencv_video.so.2.4.13
bin/Release/lane_detect: /usr/lib/libopencv_objdetect.so.2.4.13
bin/Release/lane_detect: /usr/lib/libopencv_ml.so.2.4.13
bin/Release/lane_detect: /usr/lib/libopencv_calib3d.so.2.4.13
bin/Release/lane_detect: /usr/lib/libopencv_features2d.so.2.4.13
bin/Release/lane_detect: /usr/lib/libopencv_highgui.so.2.4.13
bin/Release/lane_detect: /usr/lib/libopencv_imgproc.so.2.4.13
bin/Release/lane_detect: /usr/lib/libopencv_flann.so.2.4.13
bin/Release/lane_detect: /usr/lib/libopencv_core.so.2.4.13
bin/Release/lane_detect: /usr/local/cuda-6.5/lib/libcudart.so
bin/Release/lane_detect: /usr/local/cuda-6.5/lib/libnppc.so
bin/Release/lane_detect: /usr/local/cuda-6.5/lib/libnppi.so
bin/Release/lane_detect: /usr/local/cuda-6.5/lib/libnpps.so
bin/Release/lane_detect: CMakeFiles/lane_detect.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/Release/lane_detect"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lane_detect.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lane_detect.dir/build: bin/Release/lane_detect
.PHONY : CMakeFiles/lane_detect.dir/build

CMakeFiles/lane_detect.dir/requires: CMakeFiles/lane_detect.dir/main_pid/lane_detection.cpp.o.requires
.PHONY : CMakeFiles/lane_detect.dir/requires

CMakeFiles/lane_detect.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lane_detect.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lane_detect.dir/clean

CMakeFiles/lane_detect.dir/depend:
	cd /home/ubuntu/Lane_detector && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/Lane_detector /home/ubuntu/Lane_detector /home/ubuntu/Lane_detector /home/ubuntu/Lane_detector /home/ubuntu/Lane_detector/CMakeFiles/lane_detect.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lane_detect.dir/depend

