<snippet>
  <content>
# Project


## Installation
1. `cd build`
2. `cmake ..`
3. `make`

## Usage
###converter
Convert model collada to ply and pcd, create and train the dataset for later use.

1. To convert the Collada modle to *.ply: `./Collada2Ply ColladaFileName(.dae)`
2. To convert *.ply to *.pcd: `./mesh_sampling OutputName(.pcd) [Options]` (Must finish step 1.)

###create_dataset
Create a set of models with different orientations and scales.

1. To create the data set with a model *.pcd (Must finish converter step 1.and 2.): `./pcd2dataset model_filename(.pcd) [Options]`
2. To create the data set with a model *.ply (Must finish converter step 1): `./pcd2dataset model_filename(.ply) [Options]`
3. To calculate the VFH descriptors and do libSVM training (with the pcd files dataset named training_model_[number].pcd):  `./create_vfh_dataset output_SVM_model(*.model) [Options]`
4. To calculate the VFH descriptors then reserve as pcd file (with the pcd files dataset named training_model_[number].pcd):   `./write_VFH_pcd`

###tsdf_cloud


###matching
  </content>
</snippet>
