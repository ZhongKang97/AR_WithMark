cmake_minimum_required( VERSION 2.8 )
project( MR_POSE )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )
############### dependencies ######################
find_package( OpenCV  REQUIRED )
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )

include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    ${G2O_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    "/usr/include/eigen3/"
)

set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
	${CSPARSE_LIBRARY}
	g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
)
############### source and test ######################
include_directories(${PROJECT_SOURCE_DIR}/include )
#添加含有CMakeLists文件的子文件夹
add_subdirectory( src )
