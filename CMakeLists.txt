cmake_minimum_required(VERSION 3.0)
project(SmartCar)

# 寻找系统中的 OpenCV 库
find_package(OpenCV REQUIRED)

# 包含 OpenCV 头文件路径
include_directories(${OpenCV_INCLUDE_DIRS})

# 编译 car.cpp 并在最终生成名为 car 的可执行文件
add_executable(car car.cpp)

# 将 OpenCV 的动态链接库链接到生成的可执行文件 car 上
# 你可能需要链接 rt 库以支持 shm_open 和 munmap (POSIX 共享内存)
target_link_libraries(car ${OpenCV_LIBS} rt)