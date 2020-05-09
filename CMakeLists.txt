#CMake版本要求：3.12及以上
cmake_minimum_required(VERSION 3.12)

# 采用C++17标准
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_FLAGS "-std=c++17")

#编译模式
#set(CMAKE_BUILD_TYPE debug)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug)
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

#工程名称命名为"ftcctrl"
project(ftcctrl)

# 找到依赖库的包，find_package（包的名称及最低版本）
#linux操作系统下引用aris库
IF(UNIX)
	add_definitions(-DUNIX)
	find_package(aris REQUIRED PATHS /usr/aris)
ENDIF(UNIX)
#windows系统下引用aris库
IF(WIN32)
	find_package(aris REQUIRED PATHS C:/aris/aris-1.5.0)
ENDIF(WIN32)

#找到头文件，include_directories("路径")
include_directories(${aris_INCLUDE_DIRS})
#打印相关消息
message(${aris_DIR})

#设置生成共享库文件的路径（生成共享库的路径，即生成的共享库在工程文件夹下的lib文件夹中）
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
#创建共享库（把工程内的cpp文件都创建成共享库文件，方便通过头文件来调用）
#add_library(fcserver_lib STATIC src/ftc.cpp src/rsi_lib.h src/serialport.h src/serialport.cpp src/tinyxml2.h src/tinyxml2.cpp src/filter.h src/filter.cpp src/ftc_admit.h src/ftc_admit.cpp)
set(SOURCE_FILES src/ftc.cpp src/rsi_lib.h src/serialport.h src/serialport.cpp src/tinyxml2.h src/tinyxml2.cpp src/ftc_ctrl.h src/ftc_ctrl.cpp)
# 编译主函数，生成可执行文件
# 设置生成可执行文件的路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
#生成可执行文件
add_executable(ftcctlr src/main.cpp ${SOURCE_FILES})
#可执行文件的链接库
#target_link_libraries(ftcctlr ftcctlr_lib)
if(UNIX)
	target_link_libraries(ftcctlr ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
	target_link_libraries(ftcctlr ${aris_LIBRARIES})
endif(WIN32)



