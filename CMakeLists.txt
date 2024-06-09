project(CodeCraftSDK)
cmake_minimum_required (VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_VERBOSE_MAKEFILE ON)

add_compile_definitions(DISABLE_PTHREAD_HOOK)
if (NOT WIN32)
    link_libraries(pthread rt m)
endif (NOT WIN32)

find_package(Python3 COMPONENTS Development NumPy)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

AUX_SOURCE_DIRECTORY(./src src)
list(REMOVE_ITEM src "./load_pthread.cpp")
ADD_EXECUTABLE(main ${src})
target_link_libraries(main ${catkin_LIBRARIES}
)

# project(CodeCraftSDK)
# cmake_minimum_required (VERSION 3.8)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_C_STANDARD 11)
# set(CMAKE_BUILD_TYPE Release)
# # set(CMAKE_VERBOSE_MAKEFILE ON)

# add_compile_definitions(DISABLE_PTHREAD_HOOK)
# if (NOT WIN32)
#     link_libraries(pthread rt m)
# endif (NOT WIN32)

# include_directories(
#   include
#   SYSTEM
# )

# AUX_SOURCE_DIRECTORY(. src)
# list(REMOVE_ITEM src "./load_pthread.cpp")
# ADD_EXECUTABLE(main ${src})
# target_link_libraries(main)
