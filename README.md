<snippet>
  <content>
# Project


## Installation
1. `cd build`
2. `cmake ..`
3. `make`

## Introduction
###converter
Convert model collada to ply and pcd, create and train the dataset for later use.

To convert the Collada modle to *.ply: `./Collada2Ply ColladaFileName(.dae)`
To convert *.ply to *.pcd: `./mesh_sampling OutputName(.pcd) [Options]` (Must finish step 1.)

###create_dataset
Create a set of models with different orientations and scales.

To create the data set with a model *.pcd (Must finish converter steps): `./pcd2dataset model_filename(.pcd) [Options]`
To create the data set with a model *.ply (Must finish converter first step): `./pcd2dataset model_filename(.ply) [Options]`
To calculate the VFH descriptors and do libSVM training (with the pcd files dataset named training_model_[number].pcd):  `./create_vfh_dataset output_SVM_model(*.model) [Options]`
To calculate the VFH descriptors then reserve as pcd file (with the pcd files dataset named training_model_[number].pcd):   `./write_VFH_pcd`

###tsdf_cloud

To get the TSDF point cloud :  `./tsdf_cloud` then press Exit.

###matching
Method1


Method2

###examples
Some output examples

###others
Others methods I tried

  </content>
</snippet>
