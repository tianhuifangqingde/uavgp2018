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
include uavgap/CMakeFiles/keyboard.dir/depend.make

# Include the progress variables for this target.
include uavgap/CMakeFiles/keyboard.dir/progress.make

# Include the compile flags for this target's objects.
include uavgap/CMakeFiles/keyboard.dir/flags.make

uavgap/CMakeFiles/keyboard.dir/src/keyboard.cpp.o: uavgap/CMakeFiles/keyboard.dir/flags.make
uavgap/CMakeFiles/keyboard.dir/src/keyboard.cpp.o: /home/odroid/uavgap/src/uavgap/src/keyboard.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/odroid/uavgap/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object uavgap/CMakeFiles/keyboard.dir/src/keyboard.cpp.o"
	cd /home/odroid/uavgap/build/uavgap && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/keyboard.dir/src/keyboard.cpp.o -c /home/odroid/uavgap/src/uavgap/src/keyboard.cpp

uavgap/CMakeFiles/keyboard.dir/src/keyboard.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/keyboard.dir/src/keyboard.cpp.i"
	cd /home/odroid/uavgap/build/uavgap && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/odroid/uavgap/src/uavgap/src/keyboard.cpp > CMakeFiles/keyboard.dir/src/keyboard.cpp.i

uavgap/CMakeFiles/keyboard.dir/src/keyboard.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/keyboard.dir/src/keyboard.cpp.s"
	cd /home/odroid/uavgap/build/uavgap && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/odroid/uavgap/src/uavgap/src/keyboard.cpp -o CMakeFiles/keyboard.dir/src/keyboard.cpp.s

uavgap/CMakeFiles/keyboard.dir/src/keyboard.cpp.o.requires:

.PHONY : uavgap/CMakeFiles/keyboard.dir/src/keyboard.cpp.o.requires

uavgap/CMakeFiles/keyboard.dir/src/keyboard.cpp.o.provides: uavgap/CMakeFiles/keyboard.dir/src/keyboard.cpp.o.requires
	$(MAKE) -f uavgap/CMakeFiles/keyboard.dir/build.make uavgap/CMakeFiles/keyboard.dir/src/keyboard.cpp.o.provides.build
.PHONY : uavgap/CMakeFiles/keyboard.dir/src/keyboard.cpp.o.provides

uavgap/CMakeFiles/keyboard.dir/src/keyboard.cpp.o.provides.build: uavgap/CMakeFiles/keyboard.dir/src/keyboard.cpp.o


# Object files for target keyboard
keyboard_OBJECTS = \
"CMakeFiles/keyboard.dir/src/keyboard.cpp.o"

# External object files for target keyboard
keyboard_EXTERNAL_OBJECTS =

/home/odroid/uavgap/devel/lib/uavgap/keyboard: uavgap/CMakeFiles/keyboard.dir/src/keyboard.cpp.o
/home/odroid/uavgap/devel/lib/uavgap/keyboard: uavgap/CMakeFiles/keyboard.dir/build.make
/home/odroid/uavgap/devel/lib/uavgap/keyboard: /opt/ros/indigo/lib/libroscpp.so
/home/odroid/uavgap/devel/lib/uavgap/keyboard: /usr/lib/arm-linux-gnueabihf/libboost_signals.so
/home/odroid/uavgap/devel/lib/uavgap/keyboard: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
/home/odroid/uavgap/devel/lib/uavgap/keyboard: /opt/ros/indigo/lib/librosconsole.so
/home/odroid/uavgap/devel/lib/uavgap/keyboard: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/odroid/uavgap/devel/lib/uavgap/keyboard: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/odroid/uavgap/devel/lib/uavgap/keyboard: /usr/lib/liblog4cxx.so
/home/odroid/uavgap/devel/lib/uavgap/keyboard: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
/home/odroid/uavgap/devel/lib/uavgap/keyboard: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/odroid/uavgap/devel/lib/uavgap/keyboard: /opt/ros/indigo/lib/libserial.so
/home/odroid/uavgap/devel/lib/uavgap/keyboard: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/odroid/uavgap/devel/lib/uavgap/keyboard: /opt/ros/indigo/lib/librostime.so
/home/odroid/uavgap/devel/lib/uavgap/keyboard: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
/home/odroid/uavgap/devel/lib/uavgap/keyboard: /opt/ros/indigo/lib/libcpp_common.so
/home/odroid/uavgap/devel/lib/uavgap/keyboard: /usr/lib/arm-linux-gnueabihf/libboost_system.so
/home/odroid/uavgap/devel/lib/uavgap/keyboard: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
/home/odroid/uavgap/devel/lib/uavgap/keyboard: /usr/lib/arm-linux-gnueabihf/libpthread.so
/home/odroid/uavgap/devel/lib/uavgap/keyboard: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so
/home/odroid/uavgap/devel/lib/uavgap/keyboard: uavgap/CMakeFiles/keyboard.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/odroid/uavgap/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/odroid/uavgap/devel/lib/uavgap/keyboard"
	cd /home/odroid/uavgap/build/uavgap && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/keyboard.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
uavgap/CMakeFiles/keyboard.dir/build: /home/odroid/uavgap/devel/lib/uavgap/keyboard

.PHONY : uavgap/CMakeFiles/keyboard.dir/build

uavgap/CMakeFiles/keyboard.dir/requires: uavgap/CMakeFiles/keyboard.dir/src/keyboard.cpp.o.requires

.PHONY : uavgap/CMakeFiles/keyboard.dir/requires

uavgap/CMakeFiles/keyboard.dir/clean:
	cd /home/odroid/uavgap/build/uavgap && $(CMAKE_COMMAND) -P CMakeFiles/keyboard.dir/cmake_clean.cmake
.PHONY : uavgap/CMakeFiles/keyboard.dir/clean

uavgap/CMakeFiles/keyboard.dir/depend:
	cd /home/odroid/uavgap/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/odroid/uavgap/src /home/odroid/uavgap/src/uavgap /home/odroid/uavgap/build /home/odroid/uavgap/build/uavgap /home/odroid/uavgap/build/uavgap/CMakeFiles/keyboard.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : uavgap/CMakeFiles/keyboard.dir/depend

