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
CMAKE_SOURCE_DIR = /home/ws/minisnap-1.0/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ws/minisnap-1.0/build

# Include any dependencies generated for this target.
include a_star/CMakeFiles/astar_node.dir/depend.make

# Include the progress variables for this target.
include a_star/CMakeFiles/astar_node.dir/progress.make

# Include the compile flags for this target's objects.
include a_star/CMakeFiles/astar_node.dir/flags.make

a_star/CMakeFiles/astar_node.dir/src/astar_node.cpp.o: a_star/CMakeFiles/astar_node.dir/flags.make
a_star/CMakeFiles/astar_node.dir/src/astar_node.cpp.o: /home/ws/minisnap-1.0/src/a_star/src/astar_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ws/minisnap-1.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object a_star/CMakeFiles/astar_node.dir/src/astar_node.cpp.o"
	cd /home/ws/minisnap-1.0/build/a_star && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/astar_node.dir/src/astar_node.cpp.o -c /home/ws/minisnap-1.0/src/a_star/src/astar_node.cpp

a_star/CMakeFiles/astar_node.dir/src/astar_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/astar_node.dir/src/astar_node.cpp.i"
	cd /home/ws/minisnap-1.0/build/a_star && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ws/minisnap-1.0/src/a_star/src/astar_node.cpp > CMakeFiles/astar_node.dir/src/astar_node.cpp.i

a_star/CMakeFiles/astar_node.dir/src/astar_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/astar_node.dir/src/astar_node.cpp.s"
	cd /home/ws/minisnap-1.0/build/a_star && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ws/minisnap-1.0/src/a_star/src/astar_node.cpp -o CMakeFiles/astar_node.dir/src/astar_node.cpp.s

# Object files for target astar_node
astar_node_OBJECTS = \
"CMakeFiles/astar_node.dir/src/astar_node.cpp.o"

# External object files for target astar_node
astar_node_EXTERNAL_OBJECTS =

/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: a_star/CMakeFiles/astar_node.dir/src/astar_node.cpp.o
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: a_star/CMakeFiles/astar_node.dir/build.make
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /home/ws/minisnap-1.0/devel/lib/libAstar_searcher.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /home/ws/minisnap-1.0/devel/lib/libplanner.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /home/ws/minisnap-1.0/devel/lib/libtrajectory_generator.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /opt/ros/noetic/lib/libroscpp.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /opt/ros/noetic/lib/librosconsole.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /home/ws/minisnap-1.0/devel/lib/libencode_msgs.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /home/ws/minisnap-1.0/devel/lib/libdecode_msgs.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /opt/ros/noetic/lib/librostime.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /opt/ros/noetic/lib/libcpp_common.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpcl_people.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libqhull.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/libOpenNI.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/libOpenNI2.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libfreetype.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libz.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libjpeg.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpng.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libtiff.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libexpat.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpcl_features.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpcl_search.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpcl_io.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libpcl_common.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libfreetype.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libz.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libGLEW.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libSM.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libICE.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libX11.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libXext.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: /usr/lib/x86_64-linux-gnu/libXt.so
/home/ws/minisnap-1.0/devel/lib/a_star/astar_node: a_star/CMakeFiles/astar_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ws/minisnap-1.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/ws/minisnap-1.0/devel/lib/a_star/astar_node"
	cd /home/ws/minisnap-1.0/build/a_star && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/astar_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
a_star/CMakeFiles/astar_node.dir/build: /home/ws/minisnap-1.0/devel/lib/a_star/astar_node

.PHONY : a_star/CMakeFiles/astar_node.dir/build

a_star/CMakeFiles/astar_node.dir/clean:
	cd /home/ws/minisnap-1.0/build/a_star && $(CMAKE_COMMAND) -P CMakeFiles/astar_node.dir/cmake_clean.cmake
.PHONY : a_star/CMakeFiles/astar_node.dir/clean

a_star/CMakeFiles/astar_node.dir/depend:
	cd /home/ws/minisnap-1.0/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ws/minisnap-1.0/src /home/ws/minisnap-1.0/src/a_star /home/ws/minisnap-1.0/build /home/ws/minisnap-1.0/build/a_star /home/ws/minisnap-1.0/build/a_star/CMakeFiles/astar_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : a_star/CMakeFiles/astar_node.dir/depend

