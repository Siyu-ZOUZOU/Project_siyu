cmake_minimum_required( VERSION 2.6 )
project(collada2ply)

#########################################################
# FIND GLM
#########################################################
find_package(glm REQUIRED)
include_directories(${glm_INCLUDE_DIRS})
link_directories(${glm_LIBRARY_DIRS})
add_definitions(${glm_DEFINITIONS})
if(NOT glm_FOUND)
    message(ERROR " GLM not found!")
endif(NOT glm_FOUND)
#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)

include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)

#########################################################
# FIND GLEW
#########################################################
find_package(GLEW REQUIRED)

include_directories(${GLEW_INCLUDE_DIRS})
link_directories(${GLEW_LIBRARY_DIRS})
add_definitions(${GLEW_DEFINITIONS})

if(NOT GLEW_FOUND)
    message(ERROR " GLEW not found!")
endif(NOT GLEW_FOUND)

#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

#########################################################
# FIND ASSIMP
#########################################################
find_package( Assimp REQUIRED )
INCLUDE_DIRECTORIES(
  ${Assimp_SOURCE_DIR}/include
  ${ASSIMP_INCLUDE_DIRS}
)

LINK_DIRECTORIES( ${Assimp_BINARY_DIR} ${Assimp_BINARY_DIR}/lib ${ASSIMP_LIBRARY_DIRS})
add_definitions(${ASSIMP_DEFINITIONS})

#########################################################
# FIND PCL
#########################################################
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#########################################################
# FIND CUDA
#########################################################
#Searching CUDA
FIND_PACKAGE(CUDA)

#Include the FindCUDA script
INCLUDE(FindCUDA)

#########################################################
# FIND VTK
#########################################################
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
link_directories(${VTK_LIBRARY_DIRS})
add_definitions(${VTK_DEFINITIONS})

#########################################################
# FIND OPENCV
#########################################################
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})


########################################################
# Linking & stuff  
#########################################################

ADD_EXECUTABLE( Collada2Ply converter/Mesh.h  converter/Model.h converter/Collada2Ply.cpp)
target_link_libraries(Collada2Ply ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${ASSIMP_LIBRARIES} ${GLEW_LIBRARIES} ${glm_LIBRARIES})
TARGET_LINK_LIBRARIES( Collada2Ply assimp ${ZLIB_LIBRARIES} )
target_compile_options(Collada2Ply PUBLIC -std=c++11)

add_executable (ply2pcd converter/ply2pcd.cpp)
target_link_libraries (ply2pcd ${PCL_LIBRARIES})
target_link_libraries(ply2pcd ${VTK_LIBRARIES})

add_executable (pcd2dataset create_dataset/pcd2dataset.cpp)
target_link_libraries (pcd2dataset ${PCL_LIBRARIES})
target_link_libraries (pcd2dataset ${VTK_LIBRARIES})

add_executable (ply2dataset create_dataset/ply2dataset.cpp)
target_link_libraries (ply2dataset ${PCL_LIBRARIES})
target_link_libraries (ply2dataset ${VTK_LIBRARIES})

add_executable (create_vfh_dataset create_dataset/create_vfh_dataset.cpp)
target_link_libraries (create_vfh_dataset ${PCL_LIBRARIES})
target_link_libraries (create_vfh_dataset ${VTK_LIBRARIES})

add_executable (write_VFH_pcd create_dataset/write_VFH_pcd.cpp)
target_link_libraries (write_VFH_pcd ${PCL_LIBRARIES})

add_library (Eva tsdf_cloud//evaluation.cpp tsdf_cloud//evaluation.h)
cuda_add_executable (tsdf_cloud tsdf_cloud/test_kinfu.cpp tsdf_cloud/openni_capture.h tsdf_cloud/color_handler.h tsdf_cloud/evaluation.h)
target_link_libraries (tsdf_cloud ${PCL_LIBRARIES} Eva)
target_link_libraries(tsdf_cloud ${VTK_LIBRARIES})
target_link_libraries (tsdf_cloud ${OpenCV_LIBRARIES})

add_executable (capture_seg matching/method1/capture_seg.cpp)
target_link_libraries (capture_seg ${PCL_LIBRARIES})

add_executable (features_matching matching/method1/features_matching.cpp)
target_link_libraries (features_matching ${PCL_LIBRARIES})

add_executable (svm_test matching/method2/svm_test.cpp)
target_link_libraries (svm_test ${PCL_LIBRARIES})

