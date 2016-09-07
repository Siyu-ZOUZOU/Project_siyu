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

1. To create the data set with a model *.pcd: `./pcd2dataset model_filename(.pcd) [Options]`(Must finish converter step 1.and 2.)
2. To create the data set with a model *.ply (integrate the two above): `./pcd2dataset model_filename(.ply) [Options]`(Must finish converter step 1)
3. To calculate the VFH descriptors and do libSVM training:  `./create_vfh_dataset output_SVM_model(*.model) [Options]`(with the pcd files dataset named training_model_[number].pcd)
4. To calculate the VFH descriptors then reserve as pcd file:   `./write_VFH_pcd`(with the pcd files dataset named training_model_[number].pcd)

###tsdf_cloud


###matching
  </content>
</snippet>
