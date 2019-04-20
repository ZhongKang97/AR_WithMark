cmake_minimum_required( VERSION 2.8 )
project( MR_POSE )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11" )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )
############### dependencies ######################
find_package( OpenCV  REQUIRED )
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )

include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    ${CSPARSE_INCLUDE_DIR}
    ${G2O_INCLUDE_DIRS}
    "/usr/include/eigen3/"
)
set(g2o_libs g2o_csparse_extension g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d  g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse)

set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${CSPARSE_LIBRARY}
    ${g2o_libs}
)
############### source and test ######################
include_directories(${PROJECT_SOURCE_DIR}/include )
#添加含有CMakeLists文件的子文件夹
add_subdirectory( src )
