# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/Desktop/Turing/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/Desktop/Turing/src/build

# Include any dependencies generated for this target.
include CMakeFiles/unit_test_controlal.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/unit_test_controlal.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/unit_test_controlal.dir/flags.make

CMakeFiles/unit_test_controlal.dir/unit_test/test_controlal.cpp.o: CMakeFiles/unit_test_controlal.dir/flags.make
CMakeFiles/unit_test_controlal.dir/unit_test/test_controlal.cpp.o: ../unit_test/test_controlal.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Turing/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/unit_test_controlal.dir/unit_test/test_controlal.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/unit_test_controlal.dir/unit_test/test_controlal.cpp.o -c /home/pi/Desktop/Turing/src/unit_test/test_controlal.cpp

CMakeFiles/unit_test_controlal.dir/unit_test/test_controlal.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/unit_test_controlal.dir/unit_test/test_controlal.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Turing/src/unit_test/test_controlal.cpp > CMakeFiles/unit_test_controlal.dir/unit_test/test_controlal.cpp.i

CMakeFiles/unit_test_controlal.dir/unit_test/test_controlal.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/unit_test_controlal.dir/unit_test/test_controlal.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Turing/src/unit_test/test_controlal.cpp -o CMakeFiles/unit_test_controlal.dir/unit_test/test_controlal.cpp.s

CMakeFiles/unit_test_controlal.dir/control/controlal.cpp.o: CMakeFiles/unit_test_controlal.dir/flags.make
CMakeFiles/unit_test_controlal.dir/control/controlal.cpp.o: ../control/controlal.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Turing/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/unit_test_controlal.dir/control/controlal.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/unit_test_controlal.dir/control/controlal.cpp.o -c /home/pi/Desktop/Turing/src/control/controlal.cpp

CMakeFiles/unit_test_controlal.dir/control/controlal.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/unit_test_controlal.dir/control/controlal.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Turing/src/control/controlal.cpp > CMakeFiles/unit_test_controlal.dir/control/controlal.cpp.i

CMakeFiles/unit_test_controlal.dir/control/controlal.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/unit_test_controlal.dir/control/controlal.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Turing/src/control/controlal.cpp -o CMakeFiles/unit_test_controlal.dir/control/controlal.cpp.s

CMakeFiles/unit_test_controlal.dir/robot/device_CM4.cpp.o: CMakeFiles/unit_test_controlal.dir/flags.make
CMakeFiles/unit_test_controlal.dir/robot/device_CM4.cpp.o: ../robot/device_CM4.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Turing/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/unit_test_controlal.dir/robot/device_CM4.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/unit_test_controlal.dir/robot/device_CM4.cpp.o -c /home/pi/Desktop/Turing/src/robot/device_CM4.cpp

CMakeFiles/unit_test_controlal.dir/robot/device_CM4.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/unit_test_controlal.dir/robot/device_CM4.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Turing/src/robot/device_CM4.cpp > CMakeFiles/unit_test_controlal.dir/robot/device_CM4.cpp.i

CMakeFiles/unit_test_controlal.dir/robot/device_CM4.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/unit_test_controlal.dir/robot/device_CM4.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Turing/src/robot/device_CM4.cpp -o CMakeFiles/unit_test_controlal.dir/robot/device_CM4.cpp.s

CMakeFiles/unit_test_controlal.dir/robot/device_ROCKS.cpp.o: CMakeFiles/unit_test_controlal.dir/flags.make
CMakeFiles/unit_test_controlal.dir/robot/device_ROCKS.cpp.o: ../robot/device_ROCKS.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Turing/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/unit_test_controlal.dir/robot/device_ROCKS.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/unit_test_controlal.dir/robot/device_ROCKS.cpp.o -c /home/pi/Desktop/Turing/src/robot/device_ROCKS.cpp

CMakeFiles/unit_test_controlal.dir/robot/device_ROCKS.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/unit_test_controlal.dir/robot/device_ROCKS.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Turing/src/robot/device_ROCKS.cpp > CMakeFiles/unit_test_controlal.dir/robot/device_ROCKS.cpp.i

CMakeFiles/unit_test_controlal.dir/robot/device_ROCKS.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/unit_test_controlal.dir/robot/device_ROCKS.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Turing/src/robot/device_ROCKS.cpp -o CMakeFiles/unit_test_controlal.dir/robot/device_ROCKS.cpp.s

CMakeFiles/unit_test_controlal.dir/robot/robotz.cpp.o: CMakeFiles/unit_test_controlal.dir/flags.make
CMakeFiles/unit_test_controlal.dir/robot/robotz.cpp.o: ../robot/robotz.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Turing/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/unit_test_controlal.dir/robot/robotz.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/unit_test_controlal.dir/robot/robotz.cpp.o -c /home/pi/Desktop/Turing/src/robot/robotz.cpp

CMakeFiles/unit_test_controlal.dir/robot/robotz.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/unit_test_controlal.dir/robot/robotz.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Turing/src/robot/robotz.cpp > CMakeFiles/unit_test_controlal.dir/robot/robotz.cpp.i

CMakeFiles/unit_test_controlal.dir/robot/robotz.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/unit_test_controlal.dir/robot/robotz.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Turing/src/robot/robotz.cpp -o CMakeFiles/unit_test_controlal.dir/robot/robotz.cpp.s

CMakeFiles/unit_test_controlal.dir/robot/wifiz.cpp.o: CMakeFiles/unit_test_controlal.dir/flags.make
CMakeFiles/unit_test_controlal.dir/robot/wifiz.cpp.o: ../robot/wifiz.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Turing/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/unit_test_controlal.dir/robot/wifiz.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/unit_test_controlal.dir/robot/wifiz.cpp.o -c /home/pi/Desktop/Turing/src/robot/wifiz.cpp

CMakeFiles/unit_test_controlal.dir/robot/wifiz.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/unit_test_controlal.dir/robot/wifiz.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Turing/src/robot/wifiz.cpp > CMakeFiles/unit_test_controlal.dir/robot/wifiz.cpp.i

CMakeFiles/unit_test_controlal.dir/robot/wifiz.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/unit_test_controlal.dir/robot/wifiz.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Turing/src/robot/wifiz.cpp -o CMakeFiles/unit_test_controlal.dir/robot/wifiz.cpp.s

CMakeFiles/unit_test_controlal.dir/unit_test/test_comm.cpp.o: CMakeFiles/unit_test_controlal.dir/flags.make
CMakeFiles/unit_test_controlal.dir/unit_test/test_comm.cpp.o: ../unit_test/test_comm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Turing/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/unit_test_controlal.dir/unit_test/test_comm.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/unit_test_controlal.dir/unit_test/test_comm.cpp.o -c /home/pi/Desktop/Turing/src/unit_test/test_comm.cpp

CMakeFiles/unit_test_controlal.dir/unit_test/test_comm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/unit_test_controlal.dir/unit_test/test_comm.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Turing/src/unit_test/test_comm.cpp > CMakeFiles/unit_test_controlal.dir/unit_test/test_comm.cpp.i

CMakeFiles/unit_test_controlal.dir/unit_test/test_comm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/unit_test_controlal.dir/unit_test/test_comm.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Turing/src/unit_test/test_comm.cpp -o CMakeFiles/unit_test_controlal.dir/unit_test/test_comm.cpp.s

CMakeFiles/unit_test_controlal.dir/unit_test/test_motors.cpp.o: CMakeFiles/unit_test_controlal.dir/flags.make
CMakeFiles/unit_test_controlal.dir/unit_test/test_motors.cpp.o: ../unit_test/test_motors.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Turing/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/unit_test_controlal.dir/unit_test/test_motors.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/unit_test_controlal.dir/unit_test/test_motors.cpp.o -c /home/pi/Desktop/Turing/src/unit_test/test_motors.cpp

CMakeFiles/unit_test_controlal.dir/unit_test/test_motors.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/unit_test_controlal.dir/unit_test/test_motors.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Turing/src/unit_test/test_motors.cpp > CMakeFiles/unit_test_controlal.dir/unit_test/test_motors.cpp.i

CMakeFiles/unit_test_controlal.dir/unit_test/test_motors.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/unit_test_controlal.dir/unit_test/test_motors.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Turing/src/unit_test/test_motors.cpp -o CMakeFiles/unit_test_controlal.dir/unit_test/test_motors.cpp.s

# Object files for target unit_test_controlal
unit_test_controlal_OBJECTS = \
"CMakeFiles/unit_test_controlal.dir/unit_test/test_controlal.cpp.o" \
"CMakeFiles/unit_test_controlal.dir/control/controlal.cpp.o" \
"CMakeFiles/unit_test_controlal.dir/robot/device_CM4.cpp.o" \
"CMakeFiles/unit_test_controlal.dir/robot/device_ROCKS.cpp.o" \
"CMakeFiles/unit_test_controlal.dir/robot/robotz.cpp.o" \
"CMakeFiles/unit_test_controlal.dir/robot/wifiz.cpp.o" \
"CMakeFiles/unit_test_controlal.dir/unit_test/test_comm.cpp.o" \
"CMakeFiles/unit_test_controlal.dir/unit_test/test_motors.cpp.o"

# External object files for target unit_test_controlal
unit_test_controlal_EXTERNAL_OBJECTS =

unit_test_controlal: CMakeFiles/unit_test_controlal.dir/unit_test/test_controlal.cpp.o
unit_test_controlal: CMakeFiles/unit_test_controlal.dir/control/controlal.cpp.o
unit_test_controlal: CMakeFiles/unit_test_controlal.dir/robot/device_CM4.cpp.o
unit_test_controlal: CMakeFiles/unit_test_controlal.dir/robot/device_ROCKS.cpp.o
unit_test_controlal: CMakeFiles/unit_test_controlal.dir/robot/robotz.cpp.o
unit_test_controlal: CMakeFiles/unit_test_controlal.dir/robot/wifiz.cpp.o
unit_test_controlal: CMakeFiles/unit_test_controlal.dir/unit_test/test_comm.cpp.o
unit_test_controlal: CMakeFiles/unit_test_controlal.dir/unit_test/test_motors.cpp.o
unit_test_controlal: CMakeFiles/unit_test_controlal.dir/build.make
unit_test_controlal: /usr/lib/arm-linux-gnueabihf/libfmt.so.7.1.3
unit_test_controlal: CMakeFiles/unit_test_controlal.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/Desktop/Turing/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking CXX executable unit_test_controlal"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/unit_test_controlal.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/unit_test_controlal.dir/build: unit_test_controlal

.PHONY : CMakeFiles/unit_test_controlal.dir/build

CMakeFiles/unit_test_controlal.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/unit_test_controlal.dir/cmake_clean.cmake
.PHONY : CMakeFiles/unit_test_controlal.dir/clean

CMakeFiles/unit_test_controlal.dir/depend:
	cd /home/pi/Desktop/Turing/src/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/Desktop/Turing/src /home/pi/Desktop/Turing/src /home/pi/Desktop/Turing/src/build /home/pi/Desktop/Turing/src/build /home/pi/Desktop/Turing/src/build/CMakeFiles/unit_test_controlal.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/unit_test_controlal.dir/depend

