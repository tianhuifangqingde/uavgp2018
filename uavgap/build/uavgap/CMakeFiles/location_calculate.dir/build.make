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
include uavgap/CMakeFiles/location_calculate.dir/depend.make

# Include the progress variables for this target.
include uavgap/CMakeFiles/location_calculate.dir/progress.make

# Include the compile flags for this target's objects.
include uavgap/CMakeFiles/location_calculate.dir/flags.make

uavgap/CMakeFiles/location_calculate.dir/src/location_calculate.cpp.o: uavgap/CMakeFiles/location_calculate.dir/flags.make
uavgap/CMakeFiles/location_calculate.dir/src/location_calculate.cpp.o: /home/odroid/uavgap/src/uavgap/src/location_calculate.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/odroid/uavgap/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object uavgap/CMakeFiles/location_calculate.dir/src/location_calculate.cpp.o"
	cd /home/odroid/uavgap/build/uavgap && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/location_calculate.dir/src/location_calculate.cpp.o -c /home/odroid/uavgap/src/uavgap/src/location_calculate.cpp

uavgap/CMakeFiles/location_calculate.dir/src/location_calculate.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/location_calculate.dir/src/location_calculate.cpp.i"
	cd /home/odroid/uavgap/build/uavgap && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/odroid/uavgap/src/uavgap/src/location_calculate.cpp > CMakeFiles/location_calculate.dir/src/location_calculate.cpp.i

uavgap/CMakeFiles/location_calculate.dir/src/location_calculate.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/location_calculate.dir/src/location_calculate.cpp.s"
	cd /home/odroid/uavgap/build/uavgap && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/odroid/uavgap/src/uavgap/src/location_calculate.cpp -o CMakeFiles/location_calculate.dir/src/location_calculate.cpp.s

uavgap/CMakeFiles/location_calculate.dir/src/location_calculate.cpp.o.requires:

.PHONY : uavgap/CMakeFiles/location_calculate.dir/src/location_calculate.cpp.o.requires

uavgap/CMakeFiles/location_calculate.dir/src/location_calculate.cpp.o.provides: uavgap/CMakeFiles/location_calculate.dir/src/location_calculate.cpp.o.requires
	$(MAKE) -f uavgap/CMakeFiles/location_calculate.dir/build.make uavgap/CMakeFiles/location_calculate.dir/src/location_calculate.cpp.o.provides.build
.PHONY : uavgap/CMakeFiles/location_calculate.dir/src/location_calculate.cpp.o.provides

uavgap/CMakeFiles/location_calculate.dir/src/location_calculate.cpp.o.provides.build: uavgap/CMakeFiles/location_calculate.dir/src/location_calculate.cpp.o


# Object files for target location_calculate
location_calculate_OBJECTS = \
"CMakeFiles/location_calculate.dir/src/location_calculate.cpp.o"

# External object files for target location_calculate
location_calculate_EXTERNAL_OBJECTS =

/home/odroid/uavgap/devel/lib/uavgap/location_calculate: uavgap/CMakeFiles/location_calculate.dir/src/location_calculate.cpp.o
/home/odroid/uavgap/devel/lib/uavgap/location_calculate: uavgap/CMakeFiles/location_calculate.dir/build.make
/home/odroid/uavgap/devel/lib/uavgap/location_calculate: /opt/ros/indigo/lib/libroscpp.so
/home/odroid/uavgap/devel/lib/uavgap/location_calculate: /usr/lib/arm-linux-gnueabihf/libboost_signals.so
/home/odroid/uavgap/devel/lib/uavgap/location_calculate: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
/home/odroid/uavgap/devel/lib/uavgap/location_calculate: /opt/ros/indigo/lib/librosconsole.so
/home/odroid/uavgap/devel/lib/uavgap/location_calculate: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/odroid/uavgap/devel/lib/uavgap/location_calculate: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/odroid/uavgap/devel/lib/uavgap/location_calculate: /usr/lib/liblog4cxx.so
/home/odroid/uavgap/devel/lib/uavgap/location_calculate: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
/home/odroid/uavgap/devel/lib/uavgap/location_calculate: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/odroid/uavgap/devel/lib/uavgap/location_calculate: /opt/ros/indigo/lib/libserial.so
/home/odroid/uavgap/devel/lib/uavgap/location_calculate: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/odroid/uavgap/devel/lib/uavgap/location_calculate: /opt/ros/indigo/lib/librostime.so
/home/odroid/uavgap/devel/lib/uavgap/location_calculate: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
/home/odroid/uavgap/devel/lib/uavgap/location_calculate: /opt/ros/indigo/lib/libcpp_common.so
/home/odroid/uavgap/devel/lib/uavgap/location_calculate: /usr/lib/arm-linux-gnueabihf/libboost_system.so
/home/odroid/uavgap/devel/lib/uavgap/location_calculate: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
/home/odroid/uavgap/devel/lib/uavgap/location_calculate: /usr/lib/arm-linux-gnueabihf/libpthread.so
/home/odroid/uavgap/devel/lib/uavgap/location_calculate: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so
/home/odroid/uavgap/devel/lib/uavgap/location_calculate: uavgap/CMakeFiles/location_calculate.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/odroid/uavgap/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/odroid/uavgap/devel/lib/uavgap/location_calculate"
	cd /home/odroid/uavgap/build/uavgap && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/location_calculate.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
uavgap/CMakeFiles/location_calculate.dir/build: /home/odroid/uavgap/devel/lib/uavgap/location_calculate

.PHONY : uavgap/CMakeFiles/location_calculate.dir/build

uavgap/CMakeFiles/location_calculate.dir/requires: uavgap/CMakeFiles/location_calculate.dir/src/location_calculate.cpp.o.requires

.PHONY : uavgap/CMakeFiles/location_calculate.dir/requires

uavgap/CMakeFiles/location_calculate.dir/clean:
	cd /home/odroid/uavgap/build/uavgap && $(CMAKE_COMMAND) -P CMakeFiles/location_calculate.dir/cmake_clean.cmake
.PHONY : uavgap/CMakeFiles/location_calculate.dir/clean

uavgap/CMakeFiles/location_calculate.dir/depend:
	cd /home/odroid/uavgap/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/odroid/uavgap/src /home/odroid/uavgap/src/uavgap /home/odroid/uavgap/build /home/odroid/uavgap/build/uavgap /home/odroid/uavgap/build/uavgap/CMakeFiles/location_calculate.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : uavgap/CMakeFiles/location_calculate.dir/depend

