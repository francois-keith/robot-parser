cmake_minimum_required(VERSION 2.6)
enable_language(CXX)
set(CMAKE_VERBOSE_MAKEFILE ON)

# The sources of the program
set(RobotParserFiles
src/robotParser.h

src/tools.cpp

src/parseExcel.cpp
src/parseWRL.cpp

src/writeAmelif.cpp
src/writeOpenHRP.cpp
src/writeUrdf.cpp
src/writeMaple.cpp

src/main.cpp
src/main-romeo.cpp
)

include_directories(src)
add_executable(robotParser ${RobotParserFiles})

# The executable is stored in bin
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)

