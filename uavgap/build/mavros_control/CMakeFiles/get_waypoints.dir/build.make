# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/odroid/uavgap/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/odroid/uavgap/build

# Include any dependencies generated for this target.
include mavros_control/CMakeFiles/get_waypoints.dir/depend.make

# Include the progress variables for this target.
include mavros_control/CMakeFiles/get_waypoints.dir/progress.make

# Include the compile flags for this target's objects.
include mavros_control/CMakeFiles/get_waypoints.dir/flags.make

mavros_control/CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.o: mavros_control/CMakeFiles/get_waypoints.dir/flags.make
mavros_control/CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.o: /home/odroid/uavgap/src/mavros_control/src/get_waypoints.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/odroid/uavgap/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object mavros_control/CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.o"
	cd /home/odroid/uavgap/build/mavros_control && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.o -c /home/odroid/uavgap/src/mavros_control/src/get_waypoints.cpp

mavros_control/CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.i"
	cd /home/odroid/uavgap/build/mavros_control && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/odroid/uavgap/src/mavros_control/src/get_waypoints.cpp > CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.i

mavros_control/CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.s"
	cd /home/odroid/uavgap/build/mavros_control && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/odroid/uavgap/src/mavros_control/src/get_waypoints.cpp -o CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.s

mavros_control/CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.o.requires:

.PHONY : mavros_control/CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.o.requires

mavros_control/CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.o.provides: mavros_control/CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.o.requires
	$(MAKE) -f mavros_control/CMakeFiles/get_waypoints.dir/build.make mavros_control/CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.o.provides.build
.PHONY : mavros_control/CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.o.provides

mavros_control/CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.o.provides.build: mavros_control/CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.o


# Object files for target get_waypoints
get_waypoints_OBJECTS = \
"CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.o"

# External object files for target get_waypoints
get_waypoints_EXTERNAL_OBJECTS =

/home/odroid/uavgap/devel/lib/mavros_control/get_waypoints: mavros_control/CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.o
/home/odroid/uavgap/devel/lib/mavros_control/get_waypoints: mavros_control/CMakeFiles/get_waypoints.dir/build.make
/home/odroid/uavgap/devel/lib/mavros_control/get_waypoints: /opt/ros/indigo/lib/libroscpp.so
/home/odroid/uavgap/devel/lib/mavros_control/get_waypoints: /usr/lib/arm-linux-gnueabihf/libboost_signals.so
/home/odroid/uavgap/devel/lib/mavros_control/get_waypoints: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
/home/odroid/uavgap/devel/lib/mavros_control/get_waypoints: /opt/ros/indigo/lib/librosconsole.so
/home/odroid/uavgap/devel/lib/mavros_control/get_waypoints: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/odroid/uavgap/devel/lib/mavros_control/get_waypoints: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/odroid/uavgap/devel/lib/mavros_control/get_waypoints: /usr/lib/liblog4cxx.so
/home/odroid/uavgap/devel/lib/mavros_control/get_waypoints: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
/home/odroid/uavgap/devel/lib/mavros_control/get_waypoints: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/odroid/uavgap/devel/lib/mavros_control/get_waypoints: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/odroid/uavgap/devel/lib/mavros_control/get_waypoints: /opt/ros/indigo/lib/librostime.so
/home/odroid/uavgap/devel/lib/mavros_control/get_waypoints: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
/home/odroid/uavgap/devel/lib/mavros_control/get_waypoints: /opt/ros/indigo/lib/libcpp_common.so
/home/odroid/uavgap/devel/lib/mavros_control/get_waypoints: /usr/lib/arm-linux-gnueabihf/libboost_system.so
/home/odroid/uavgap/devel/lib/mavros_control/get_waypoints: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
/home/odroid/uavgap/devel/lib/mavros_control/get_waypoints: /usr/lib/arm-linux-gnueabihf/libpthread.so
/home/odroid/uavgap/devel/lib/mavros_control/get_waypoints: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so
/home/odroid/uavgap/devel/lib/mavros_control/get_waypoints: mavros_control/CMakeFiles/get_waypoints.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/odroid/uavgap/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/odroid/uavgap/devel/lib/mavros_control/get_waypoints"
	cd /home/odroid/uavgap/build/mavros_control && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/get_waypoints.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
mavros_control/CMakeFiles/get_waypoints.dir/build: /home/odroid/uavgap/devel/lib/mavros_control/get_waypoints

.PHONY : mavros_control/CMakeFiles/get_waypoints.dir/build

mavros_control/CMakeFiles/get_waypoints.dir/requires: mavros_control/CMakeFiles/get_waypoints.dir/src/get_waypoints.cpp.o.requires

.PHONY : mavros_control/CMakeFiles/get_waypoints.dir/requires

mavros_control/CMakeFiles/get_waypoints.dir/clean:
	cd /home/odroid/uavgap/build/mavros_control && $(CMAKE_COMMAND) -P CMakeFiles/get_waypoints.dir/cmake_clean.cmake
.PHONY : mavros_control/CMakeFiles/get_waypoints.dir/clean

mavros_control/CMakeFiles/get_waypoints.dir/depend:
	cd /home/odroid/uavgap/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/odroid/uavgap/src /home/odroid/uavgap/src/mavros_control /home/odroid/uavgap/build /home/odroid/uavgap/build/mavros_control /home/odroid/uavgap/build/mavros_control/CMakeFiles/get_waypoints.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : mavros_control/CMakeFiles/get_waypoints.dir/depend

