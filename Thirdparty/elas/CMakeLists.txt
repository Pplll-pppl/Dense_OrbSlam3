cmake_minimum_required(VERSION 3.10)
project(elas)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3")
# 添加位置无关代码编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set(CMAKE_BUILD_TYPE Release)


## package
find_package(OpenCV REQUIRED)
##include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/elas  # 暴露elas目录的头文件，让外部能include
)
# ========== 新增：添加Eigen依赖（Elas核心依赖矩阵运算，必须加）==========
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
## source code
aux_source_directory(./elas SOURCE)

# library
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
# 若坚持用SHARED动态库，后续需确保运行时能找到libelas.so，建议改STATIC
add_library(elas STATIC
    ${SOURCE}
)

target_link_libraries(elas
    ${OpenCV_LIBS}
    Eigen3::Eigen  # 新增：链接Eigen
)

# ========== 新增：向外部暴露Elas库的头文件和链接信息 ==========
target_include_directories(elas PUBLIC
    ${PROJECT_SOURCE_DIR}/elas  # 对应你的源码目录
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

