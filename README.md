<snippet>
  <content>
#Project
_A project for object recognition and camera localization_
</br>
## Installation
1. ```cd build```
2. ```cmake ..```
3. ```make```
</br>

## Introduction
### converter
_Convert model collada to ply and pcd, create and train the dataset for later use._</br>

1. To convert the Collada modle to *.ply: ```./Collada2Ply <ColladaFileName(.dae)>```</br>
2. To convert *.ply to *.pcd: ```./ply2pcd <OutputName(.pcd)> [Options]``` (Must finish step 1.)</br>

### create_dataset
_Create a set of models with different orientations and scales._</br>

* To create the data set with a model *.pcd (Must finish converter steps): ```./pcd2dataset <model_filename(.pcd)> [Options]```</br>
* To create the data set with a model *.ply (Must finish converter first step): ```./ply2dataset <model_filename(.ply)> [Options]``` </br>
* To calculate the VFH descriptors and do libSVM training : ```./create_vfh_dataset <output_SVM_model(*.model)> [Options]``` (with the pcd files dataset named training_model_[number].pcd)</br>
* To calculate the VFH descriptors then reserve as pcd file : ```./write_VFH_pcd ``` (with the pcd files dataset named training_model_[number].pcd)</br>

### tsdf_cloud
* To get the TSDF point cloud :  ```./tsdf_cloud``` , then press Exit.     

### matching
#### method1: Global and local pipelines
* To capture the point clouds and segment planar surfaces: ```./capture_seg```</br>
* To do the global and local pipelines: ```./features_matching <scene_filename.pcd> <model_filename.pcd> <model_room_filename.pcd>```</br>

#### method2: SVM classification (with Kinfu)
* To do the svm and local pipelines: ```./svm_test <TSDF_point_cloud> <SVM_model>```</br>

### examples
Some input and output examples, and algorithm flowcharts</br>
### others
Other methods I tried.</br>
</br>
## Usage
### Method 1
1. ```./Collada2Ply <ColladaFileName(.dae)>```</br>
2. ```./ply2pcd <OutputName(.pcd)> [Options]```</br>
3. ```./capture_seg```</br>
4. ```./features_matching <scene_filename.pcd> <model_filename.pcd> <model_room_filename.pcd>```</br>
</br>

### Method2
1. ```./Collada2Ply <ColladaFileName(.dae)>```</br>
2. ```./ply2dataset <model_filename(.ply)> [Options]```</br>
3. ```./create_vfh_dataset <output_SVM_model(*.model)> [Options]```</br>
4. ```./tsdf_cloud```</br>
5. ```./svm_test <TSDF_point_cloud> <SVM_model>```</br>
</br>

## Flowchart (French)
* __Method 1__
![](https://github.com/Siyu-ZOUZOU/Project/blob/master/examples/methode1.png) 
</br>
* __Method 2__
![](https://github.com/Siyu-ZOUZOU/Project/blob/master/examples/methode2.png) 
</br>

## Report for the project
Report (French) is available in google drive :
[Report](https://drive.google.com/open?id=0B6L4aoLTHNX1bFlNR1pIUlNmbGc)  

</br>
  </content>
</snippet>
