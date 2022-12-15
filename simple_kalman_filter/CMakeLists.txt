cmake_minimum_required(VERSION 3.0.0)
project(skf VERSION 0.1.0)

include(CTest)
enable_testing()

## add opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(include)
add_executable(skf src/skf.cpp)

target_link_libraries(skf ${OpenCV_LIBS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
