# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/amov/command_pub/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/amov/command_pub/build

# Include any dependencies generated for this target.
include command/CMakeFiles/pub_rc.dir/depend.make

# Include the progress variables for this target.
include command/CMakeFiles/pub_rc.dir/progress.make

# Include the compile flags for this target's objects.
include command/CMakeFiles/pub_rc.dir/flags.make

command/CMakeFiles/pub_rc.dir/src/pub_rc.cpp.o: command/CMakeFiles/pub_rc.dir/flags.make
command/CMakeFiles/pub_rc.dir/src/pub_rc.cpp.o: /home/amov/command_pub/src/command/src/pub_rc.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amov/command_pub/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object command/CMakeFiles/pub_rc.dir/src/pub_rc.cpp.o"
	cd /home/amov/command_pub/build/command && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pub_rc.dir/src/pub_rc.cpp.o -c /home/amov/command_pub/src/command/src/pub_rc.cpp

command/CMakeFiles/pub_rc.dir/src/pub_rc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pub_rc.dir/src/pub_rc.cpp.i"
	cd /home/amov/command_pub/build/command && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amov/command_pub/src/command/src/pub_rc.cpp > CMakeFiles/pub_rc.dir/src/pub_rc.cpp.i

command/CMakeFiles/pub_rc.dir/src/pub_rc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pub_rc.dir/src/pub_rc.cpp.s"
	cd /home/amov/command_pub/build/command && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amov/command_pub/src/command/src/pub_rc.cpp -o CMakeFiles/pub_rc.dir/src/pub_rc.cpp.s

command/CMakeFiles/pub_rc.dir/src/pub_rc.cpp.o.requires:

.PHONY : command/CMakeFiles/pub_rc.dir/src/pub_rc.cpp.o.requires

command/CMakeFiles/pub_rc.dir/src/pub_rc.cpp.o.provides: command/CMakeFiles/pub_rc.dir/src/pub_rc.cpp.o.requires
	$(MAKE) -f command/CMakeFiles/pub_rc.dir/build.make command/CMakeFiles/pub_rc.dir/src/pub_rc.cpp.o.provides.build
.PHONY : command/CMakeFiles/pub_rc.dir/src/pub_rc.cpp.o.provides

command/CMakeFiles/pub_rc.dir/src/pub_rc.cpp.o.provides.build: command/CMakeFiles/pub_rc.dir/src/pub_rc.cpp.o


# Object files for target pub_rc
pub_rc_OBJECTS = \
"CMakeFiles/pub_rc.dir/src/pub_rc.cpp.o"

# External object files for target pub_rc
pub_rc_EXTERNAL_OBJECTS =

/home/amov/command_pub/devel/lib/command/pub_rc: command/CMakeFiles/pub_rc.dir/src/pub_rc.cpp.o
/home/amov/command_pub/devel/lib/command/pub_rc: command/CMakeFiles/pub_rc.dir/build.make
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/libmavros.so
/home/amov/command_pub/devel/lib/command/pub_rc: /usr/lib/aarch64-linux-gnu/libGeographic.so
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/libdiagnostic_updater.so
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/libeigen_conversions.so
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/liborocos-kdl.so.1.4.0
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/libmavconn.so
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/libclass_loader.so
/home/amov/command_pub/devel/lib/command/pub_rc: /usr/lib/libPocoFoundation.so
/home/amov/command_pub/devel/lib/command/pub_rc: /usr/lib/aarch64-linux-gnu/libdl.so
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/libroslib.so
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/librospack.so
/home/amov/command_pub/devel/lib/command/pub_rc: /usr/lib/aarch64-linux-gnu/libpython2.7.so
/home/amov/command_pub/devel/lib/command/pub_rc: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
/home/amov/command_pub/devel/lib/command/pub_rc: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/libtf.so
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/libtf2_ros.so
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/libactionlib.so
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/libmessage_filters.so
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/libtf2.so
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/libroscpp.so
/home/amov/command_pub/devel/lib/command/pub_rc: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/librosconsole.so
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/amov/command_pub/devel/lib/command/pub_rc: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/amov/command_pub/devel/lib/command/pub_rc: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/librostime.so
/home/amov/command_pub/devel/lib/command/pub_rc: /opt/ros/melodic/lib/libcpp_common.so
/home/amov/command_pub/devel/lib/command/pub_rc: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/amov/command_pub/devel/lib/command/pub_rc: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/amov/command_pub/devel/lib/command/pub_rc: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/amov/command_pub/devel/lib/command/pub_rc: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/amov/command_pub/devel/lib/command/pub_rc: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/amov/command_pub/devel/lib/command/pub_rc: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/amov/command_pub/devel/lib/command/pub_rc: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/amov/command_pub/devel/lib/command/pub_rc: command/CMakeFiles/pub_rc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/amov/command_pub/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/amov/command_pub/devel/lib/command/pub_rc"
	cd /home/amov/command_pub/build/command && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pub_rc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
command/CMakeFiles/pub_rc.dir/build: /home/amov/command_pub/devel/lib/command/pub_rc

.PHONY : command/CMakeFiles/pub_rc.dir/build

command/CMakeFiles/pub_rc.dir/requires: command/CMakeFiles/pub_rc.dir/src/pub_rc.cpp.o.requires

.PHONY : command/CMakeFiles/pub_rc.dir/requires

command/CMakeFiles/pub_rc.dir/clean:
	cd /home/amov/command_pub/build/command && $(CMAKE_COMMAND) -P CMakeFiles/pub_rc.dir/cmake_clean.cmake
.PHONY : command/CMakeFiles/pub_rc.dir/clean

command/CMakeFiles/pub_rc.dir/depend:
	cd /home/amov/command_pub/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/amov/command_pub/src /home/amov/command_pub/src/command /home/amov/command_pub/build /home/amov/command_pub/build/command /home/amov/command_pub/build/command/CMakeFiles/pub_rc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : command/CMakeFiles/pub_rc.dir/depend

