set(MRT_PKG_VERSION 2.2)
# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 2.8.12)
project(kitti_helper)

###################
## find packages ##
###################
set(MRT_ARCH "None")
find_package(mrt_cmake_modules REQUIRED)
include(UseMrtStdCompilerFlags)
include(UseMrtAutoTarget)

include(GatherDeps)

# Remove libs which cannot be found automatically
#list(REMOVE_ITEM DEPENDEND_PACKAGES <package name 1> <package name 2> ...)
find_package(AutoDeps REQUIRED COMPONENTS ${DEPENDEND_PACKAGES})


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
    INCLUDE_DIRS include ${mrt_EXPORT_INCLUDE_DIRS}
    LIBRARIES ${LIBRARY_NAME} ${mrt_EXPORT_LIBRARIES}
    CATKIN_DEPENDS ${catkin_EXPORT_DEPENDS}
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${mrt_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

link_directories(
    ${mrt_LIBRARY_DIRS}
    )


file(GLOB PROJECT_SOURCE_FILES_SRC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "src/*.cpp")
# message(STATUS "------------- PROJECT_SOURCE_FILES_SRC: ${PROJECT_SOURCE_FILES_SRC}")
# # glob_folders(SRC_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/src")
# mrt_add_node_and_nodelet(${PROJECT_NAME}_node SOURCES ${PROJECT_SOURCE_FILES_SRC})

mrt_add_executable(${PROJECT_NAME}_node FILES ${PROJECT_SOURCE_FILES_SRC})