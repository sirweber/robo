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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/blyth/Documents/Robo/robo/camera_umd/uvc_camera

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/blyth/Documents/Robo/robo/camera_umd/uvc_camera/build

# Include any dependencies generated for this target.
include CMakeFiles/camera_node.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/camera_node.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/camera_node.dir/flags.make

CMakeFiles/camera_node.dir/src/camera_node.o: CMakeFiles/camera_node.dir/flags.make
CMakeFiles/camera_node.dir/src/camera_node.o: ../src/camera_node.cpp
CMakeFiles/camera_node.dir/src/camera_node.o: ../manifest.xml
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/manifest.xml
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/stacks/image_common/camera_info_manager/manifest.xml
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
CMakeFiles/camera_node.dir/src/camera_node.o: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/blyth/Documents/Robo/robo/camera_umd/uvc_camera/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/camera_node.dir/src/camera_node.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/camera_node.dir/src/camera_node.o -c /home/blyth/Documents/Robo/robo/camera_umd/uvc_camera/src/camera_node.cpp

CMakeFiles/camera_node.dir/src/camera_node.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera_node.dir/src/camera_node.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/blyth/Documents/Robo/robo/camera_umd/uvc_camera/src/camera_node.cpp > CMakeFiles/camera_node.dir/src/camera_node.i

CMakeFiles/camera_node.dir/src/camera_node.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera_node.dir/src/camera_node.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/blyth/Documents/Robo/robo/camera_umd/uvc_camera/src/camera_node.cpp -o CMakeFiles/camera_node.dir/src/camera_node.s

CMakeFiles/camera_node.dir/src/camera_node.o.requires:
.PHONY : CMakeFiles/camera_node.dir/src/camera_node.o.requires

CMakeFiles/camera_node.dir/src/camera_node.o.provides: CMakeFiles/camera_node.dir/src/camera_node.o.requires
	$(MAKE) -f CMakeFiles/camera_node.dir/build.make CMakeFiles/camera_node.dir/src/camera_node.o.provides.build
.PHONY : CMakeFiles/camera_node.dir/src/camera_node.o.provides

CMakeFiles/camera_node.dir/src/camera_node.o.provides.build: CMakeFiles/camera_node.dir/src/camera_node.o

CMakeFiles/camera_node.dir/src/camera.o: CMakeFiles/camera_node.dir/flags.make
CMakeFiles/camera_node.dir/src/camera.o: ../src/camera.cpp
CMakeFiles/camera_node.dir/src/camera.o: ../manifest.xml
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/manifest.xml
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/stacks/image_common/camera_info_manager/manifest.xml
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
CMakeFiles/camera_node.dir/src/camera.o: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/blyth/Documents/Robo/robo/camera_umd/uvc_camera/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/camera_node.dir/src/camera.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/camera_node.dir/src/camera.o -c /home/blyth/Documents/Robo/robo/camera_umd/uvc_camera/src/camera.cpp

CMakeFiles/camera_node.dir/src/camera.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera_node.dir/src/camera.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/blyth/Documents/Robo/robo/camera_umd/uvc_camera/src/camera.cpp > CMakeFiles/camera_node.dir/src/camera.i

CMakeFiles/camera_node.dir/src/camera.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera_node.dir/src/camera.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/blyth/Documents/Robo/robo/camera_umd/uvc_camera/src/camera.cpp -o CMakeFiles/camera_node.dir/src/camera.s

CMakeFiles/camera_node.dir/src/camera.o.requires:
.PHONY : CMakeFiles/camera_node.dir/src/camera.o.requires

CMakeFiles/camera_node.dir/src/camera.o.provides: CMakeFiles/camera_node.dir/src/camera.o.requires
	$(MAKE) -f CMakeFiles/camera_node.dir/build.make CMakeFiles/camera_node.dir/src/camera.o.provides.build
.PHONY : CMakeFiles/camera_node.dir/src/camera.o.provides

CMakeFiles/camera_node.dir/src/camera.o.provides.build: CMakeFiles/camera_node.dir/src/camera.o

CMakeFiles/camera_node.dir/src/uvc_cam.o: CMakeFiles/camera_node.dir/flags.make
CMakeFiles/camera_node.dir/src/uvc_cam.o: ../src/uvc_cam.cpp
CMakeFiles/camera_node.dir/src/uvc_cam.o: ../manifest.xml
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/manifest.xml
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/stacks/image_common/camera_info_manager/manifest.xml
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
CMakeFiles/camera_node.dir/src/uvc_cam.o: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/blyth/Documents/Robo/robo/camera_umd/uvc_camera/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/camera_node.dir/src/uvc_cam.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/camera_node.dir/src/uvc_cam.o -c /home/blyth/Documents/Robo/robo/camera_umd/uvc_camera/src/uvc_cam.cpp

CMakeFiles/camera_node.dir/src/uvc_cam.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera_node.dir/src/uvc_cam.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/blyth/Documents/Robo/robo/camera_umd/uvc_camera/src/uvc_cam.cpp > CMakeFiles/camera_node.dir/src/uvc_cam.i

CMakeFiles/camera_node.dir/src/uvc_cam.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera_node.dir/src/uvc_cam.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/blyth/Documents/Robo/robo/camera_umd/uvc_camera/src/uvc_cam.cpp -o CMakeFiles/camera_node.dir/src/uvc_cam.s

CMakeFiles/camera_node.dir/src/uvc_cam.o.requires:
.PHONY : CMakeFiles/camera_node.dir/src/uvc_cam.o.requires

CMakeFiles/camera_node.dir/src/uvc_cam.o.provides: CMakeFiles/camera_node.dir/src/uvc_cam.o.requires
	$(MAKE) -f CMakeFiles/camera_node.dir/build.make CMakeFiles/camera_node.dir/src/uvc_cam.o.provides.build
.PHONY : CMakeFiles/camera_node.dir/src/uvc_cam.o.provides

CMakeFiles/camera_node.dir/src/uvc_cam.o.provides.build: CMakeFiles/camera_node.dir/src/uvc_cam.o

# Object files for target camera_node
camera_node_OBJECTS = \
"CMakeFiles/camera_node.dir/src/camera_node.o" \
"CMakeFiles/camera_node.dir/src/camera.o" \
"CMakeFiles/camera_node.dir/src/uvc_cam.o"

# External object files for target camera_node
camera_node_EXTERNAL_OBJECTS =

../bin/camera_node: CMakeFiles/camera_node.dir/src/camera_node.o
../bin/camera_node: CMakeFiles/camera_node.dir/src/camera.o
../bin/camera_node: CMakeFiles/camera_node.dir/src/uvc_cam.o
../bin/camera_node: CMakeFiles/camera_node.dir/build.make
../bin/camera_node: CMakeFiles/camera_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/camera_node"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/camera_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/camera_node.dir/build: ../bin/camera_node
.PHONY : CMakeFiles/camera_node.dir/build

CMakeFiles/camera_node.dir/requires: CMakeFiles/camera_node.dir/src/camera_node.o.requires
CMakeFiles/camera_node.dir/requires: CMakeFiles/camera_node.dir/src/camera.o.requires
CMakeFiles/camera_node.dir/requires: CMakeFiles/camera_node.dir/src/uvc_cam.o.requires
.PHONY : CMakeFiles/camera_node.dir/requires

CMakeFiles/camera_node.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/camera_node.dir/cmake_clean.cmake
.PHONY : CMakeFiles/camera_node.dir/clean

CMakeFiles/camera_node.dir/depend:
	cd /home/blyth/Documents/Robo/robo/camera_umd/uvc_camera/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/blyth/Documents/Robo/robo/camera_umd/uvc_camera /home/blyth/Documents/Robo/robo/camera_umd/uvc_camera /home/blyth/Documents/Robo/robo/camera_umd/uvc_camera/build /home/blyth/Documents/Robo/robo/camera_umd/uvc_camera/build /home/blyth/Documents/Robo/robo/camera_umd/uvc_camera/build/CMakeFiles/camera_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/camera_node.dir/depend
