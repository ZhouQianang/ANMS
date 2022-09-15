cmake_minimum_required(VERSION 2.8.3)
project(ANMS_detector)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin_simple REQUIRED)
catkin_simple()

list(APPEND SOURCE_FILES
  src/detector.cpp
  src/evHarris_ANMS_detector.cpp
  src/evFAST_ANMS_detector.cpp
  src/ArcFAST_ANMS_detector.cpp
  src/fa_harris_detector.cpp
  src/distinct_queue.cpp
  src/fixed_distinct_queue.cpp
)

list(APPEND HEADER_FILES
  include/ANMS_detector/detector.h
  include/ANMS_detector/evHarris_ANMS_detector.h
  include/ANMS_detector/evFAST_ANMS_detector.h
  include/ANMS_detector/ArcFAST_ANMS_detector.h
  include/ANMS_detector/fa_harris_detector.h
  include/ANMS_detector/local_event_queues.h
  include/ANMS_detector/distinct_queue.h
  include/ANMS_detector/fixed_distinct_queue.h
)

cs_add_executable(detector ${SOURCE_FILES} src/node.cpp)

cs_export()
