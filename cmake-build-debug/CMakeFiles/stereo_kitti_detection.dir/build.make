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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sethgi/muddsub_ws/src/ORB_SLAM2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sethgi/muddsub_ws/src/ORB_SLAM2/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/stereo_kitti_detection.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/stereo_kitti_detection.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stereo_kitti_detection.dir/flags.make

CMakeFiles/stereo_kitti_detection.dir/Examples/Stereo/stereo_kitti_detection.cpp.o: CMakeFiles/stereo_kitti_detection.dir/flags.make
CMakeFiles/stereo_kitti_detection.dir/Examples/Stereo/stereo_kitti_detection.cpp.o: ../Examples/Stereo/stereo_kitti_detection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sethgi/muddsub_ws/src/ORB_SLAM2/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/stereo_kitti_detection.dir/Examples/Stereo/stereo_kitti_detection.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereo_kitti_detection.dir/Examples/Stereo/stereo_kitti_detection.cpp.o -c /home/sethgi/muddsub_ws/src/ORB_SLAM2/Examples/Stereo/stereo_kitti_detection.cpp

CMakeFiles/stereo_kitti_detection.dir/Examples/Stereo/stereo_kitti_detection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo_kitti_detection.dir/Examples/Stereo/stereo_kitti_detection.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sethgi/muddsub_ws/src/ORB_SLAM2/Examples/Stereo/stereo_kitti_detection.cpp > CMakeFiles/stereo_kitti_detection.dir/Examples/Stereo/stereo_kitti_detection.cpp.i

CMakeFiles/stereo_kitti_detection.dir/Examples/Stereo/stereo_kitti_detection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo_kitti_detection.dir/Examples/Stereo/stereo_kitti_detection.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sethgi/muddsub_ws/src/ORB_SLAM2/Examples/Stereo/stereo_kitti_detection.cpp -o CMakeFiles/stereo_kitti_detection.dir/Examples/Stereo/stereo_kitti_detection.cpp.s

# Object files for target stereo_kitti_detection
stereo_kitti_detection_OBJECTS = \
"CMakeFiles/stereo_kitti_detection.dir/Examples/Stereo/stereo_kitti_detection.cpp.o"

# External object files for target stereo_kitti_detection
stereo_kitti_detection_EXTERNAL_OBJECTS =

../Examples/Stereo/stereo_kitti_detection: CMakeFiles/stereo_kitti_detection.dir/Examples/Stereo/stereo_kitti_detection.cpp.o
../Examples/Stereo/stereo_kitti_detection: CMakeFiles/stereo_kitti_detection.dir/build.make
../Examples/Stereo/stereo_kitti_detection: ../lib/libORB_SLAM2.so
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_stitching.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_superres.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_videostab.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_aruco.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_bgsegm.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_bioinspired.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_ccalib.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_dnn_objdetect.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_dpm.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_face.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_freetype.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_fuzzy.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_hdf.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_hfs.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_img_hash.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_line_descriptor.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_optflow.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_reg.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_rgbd.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_saliency.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_stereo.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_structured_light.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_viz.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_phase_unwrapping.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_surface_matching.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_tracking.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_datasets.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_plot.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_text.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_dnn.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_xfeatures2d.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_ml.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_shape.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_video.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_ximgproc.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_xobjdetect.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_objdetect.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_calib3d.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_features2d.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_flann.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_highgui.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_videoio.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_imgcodecs.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_xphoto.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_photo.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_imgproc.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /opt/opencv3/lib/libopencv_core.so.3.4.5
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libpangolin.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libGL.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libGLU.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libGLEW.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libEGL.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libwayland-client.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libwayland-egl.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libwayland-cursor.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libSM.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libICE.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libX11.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libXext.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libdc1394.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libavcodec.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libavformat.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libavutil.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libswscale.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libavdevice.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libpng.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libz.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libjpeg.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libtiff.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libIlmImf.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/libzstd.so
../Examples/Stereo/stereo_kitti_detection: /usr/lib/liblz4.so
../Examples/Stereo/stereo_kitti_detection: ../Thirdparty/DBoW2/lib/libDBoW2.so
../Examples/Stereo/stereo_kitti_detection: ../Thirdparty/g2o/lib/libg2o.so
../Examples/Stereo/stereo_kitti_detection: CMakeFiles/stereo_kitti_detection.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sethgi/muddsub_ws/src/ORB_SLAM2/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../Examples/Stereo/stereo_kitti_detection"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stereo_kitti_detection.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stereo_kitti_detection.dir/build: ../Examples/Stereo/stereo_kitti_detection

.PHONY : CMakeFiles/stereo_kitti_detection.dir/build

CMakeFiles/stereo_kitti_detection.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stereo_kitti_detection.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stereo_kitti_detection.dir/clean

CMakeFiles/stereo_kitti_detection.dir/depend:
	cd /home/sethgi/muddsub_ws/src/ORB_SLAM2/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sethgi/muddsub_ws/src/ORB_SLAM2 /home/sethgi/muddsub_ws/src/ORB_SLAM2 /home/sethgi/muddsub_ws/src/ORB_SLAM2/cmake-build-debug /home/sethgi/muddsub_ws/src/ORB_SLAM2/cmake-build-debug /home/sethgi/muddsub_ws/src/ORB_SLAM2/cmake-build-debug/CMakeFiles/stereo_kitti_detection.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stereo_kitti_detection.dir/depend

