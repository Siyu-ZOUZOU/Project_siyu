<snippet>
  <content>
# Project


## Installation
1. `cd build`
2. `cmake ..`
3. `make`

## Introduction
###converter
Convert model collada to ply and pcd, create and train the dataset for later use.</br>
</br>
1. To convert the Collada modle to *.ply: `./Collada2Ply ColladaFileName(.dae)`</br>
2. To convert *.ply to *.pcd: `./mesh_sampling OutputName(.pcd) [Options]` (Must finish step 1.)</br>
</br>
###create_dataset
Create a set of models with different orientations and scales.</br>
</br>
1. To create the data set with a model *.pcd (Must finish converter steps): `./pcd2dataset <model_filename(.pcd)> [Options]`</br>
2. To create the data set with a model *.ply (Must finish converter first step): `./pcd2dataset <model_filename(.ply)> [Options]` </br
3. To calculate the VFH descriptors and do libSVM training : ./create_vfh_dataset <output_SVM_model(*.model)> [Options] (with the pcd files dataset named training_model_[number].pcd)  
</br>
4. To calculate the VFH descriptors then reserve as pcd file : `./write_VFH_pcd` (with the pcd files dataset named training_model_[number].pcd)</br>
</br>
</br>
###tsdf_cloud

To get the TSDF point cloud :  `./tsdf_cloud` then press Exit.     

###matching
####method1: Global and local pipelines
To capture the point clouds and segment planar surfaces: `./capture_seg`</br>
To do the global and local pipelines: `./features_matching <scene_filename.pcd> <model_filename.pcd> <model_room_filename.pcd>`</br>
     
####method2: SVM classification (with Kinfu)
To do the svm and local pipelines: `./svm_test <TSDF_point_cloud> <SVM_model>`</br>
###examples
Some input and output examples, and algorithm flowchart</br>
</br> 
###others
Others methods I tried</br>
</br>
  </content>
</snippet>
