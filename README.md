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
To convert the Collada modle to *.ply: `./Collada2Ply ColladaFileName(.dae)`</br>
To convert *.ply to *.pcd: `./mesh_sampling OutputName(.pcd) [Options]` (Must finish step 1.)</br>
</br>
###create_dataset
Create a set of models with different orientations and scales.</br>
</br>
To create the data set with a model *.pcd (Must finish converter steps): `./pcd2dataset <model_filename(.pcd)> [Options]`</br>
To create the data set with a model *.ply (Must finish converter first step): `./pcd2dataset <model_filename(.ply)> [Options]`</br>
To calculate the VFH descriptors and do libSVM training (with the pcd files dataset named training_model_[number].pcd): `./create_vfh_dataset <output_SVM_model(*.model)> [Options]` </br>
To calculate the VFH descriptors then reserve as pcd file (with the pcd files dataset named training_model_[number].pcd):   `./write_VFH_pcd`</br>

###tsdf_cloud

To get the TSDF point cloud :  `./tsdf_cloud` then press Exit.     

###matching
####method1: Global and local pipelines

     
####method2: SVM classification (with Kinfu)

###examples
Some input and output examples, and algorithm flowchart</br>
</br> 
###others
Others methods I tried</br>
</br>
  </content>
</snippet>
