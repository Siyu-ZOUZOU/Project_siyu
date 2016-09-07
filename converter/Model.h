#pragma once
// Std. Includes
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <vector>
using namespace std;
// GL Includes
#include <GL/glew.h> // Contains all the necessery OpenGL includes
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "Mesh.h"

class Model 
{
public:
  /*  Functions   */
  // Constructor, expects a filepath to a 3D model.
  Model(const std::string& path)
  {
    this->loadModel(path);
  }

  int numvertex()
  {
    int NumberVert=0;
    for(GLuint i = 0; i < this->meshes.size(); i++)
    {
      NumberVert +=this->meshes[i].indices.size();
    }
      return NumberVert;
  }

  int numface()
  {
    int NumberFace=0;
    for(GLuint i = 0; i < this->meshes.size(); i++)
    {
      NumberFace +=this->meshes[i].faces.size();
    }
      return NumberFace;
  }


  void OutputTest()
  {
    int NumV = numvertex();
    int NumF = numface();
    for(GLuint i = 0; i < this->meshes.size(); i++)
    {
      this->meshes[i].writePLY_cor(int(i), NumV, NumF);
    }

    for(GLuint i = 0; i < this->meshes.size(); i++)
    {
      this->meshes[i].writePLY_face();
    }

  }
private:
  /*  Model Data  */
  vector<Mesh> meshes;
  unsigned int indexStart = 0 ;

  //int try_total_number=0;
  /*  Functions   */
  // Loads a model with supported ASSIMP extensions from file and stores the resulting meshes in the meshes vector.
  void loadModel(string path)
  {
    // Read file via ASSIMP
    unsigned int indexStart = 0 ;
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate  | aiProcess_FlipWindingOrder); //
    // Check for errors
    if(!scene || scene->mFlags == AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) // if is Not Zero
    {
      cout << "ERROR::ASSIMP:: " << importer.GetErrorString() << endl;
      return;
    }

    // Process ASSIMP's root node recursively
    this->processNode(scene->mRootNode, scene, scene->mRootNode->mTransformation);
  }


  // Processes a node in a recursive fashion. Processes each individual mesh located at the node and repeats this process on its children nodes (if any).
  void processNode(aiNode* node, const aiScene* scene, const aiMatrix4x4 &accumulatedTransform)
  {   
  
    aiMatrix4x4 transform = accumulatedTransform * node->mTransformation;
    aiMatrix3x3 normalTransform(transform);
    normalTransform.Transpose();
    normalTransform.Inverse();

    // Process each mesh located at the current node
    for(GLuint i = 0; i < node->mNumMeshes; i++)
    {
      // The node object only contains indices to index the actual objects in the scene. 
      // The scene contains all the data, node is just to keep stuff organized (like relations between nodes).
      aiMesh* mesh = scene->mMeshes[node->mMeshes[i]]; 
      this->meshes.push_back(this->processMesh(mesh, transform, normalTransform));			
    }
    // After we've processed all of the meshes (if any) we then recursively process each of the children nodes
    for(GLuint i = 0; i < node->mNumChildren; i++)
    {
      this->processNode(node->mChildren[i], scene, transform);
    }

  }

  Mesh processMesh(aiMesh* mesh, aiMatrix4x4 &transform, aiMatrix3x3 &normalTransform)
  {
    // Data to fill
    vector<Vertex> vertices;
    vector<GLuint> indices;
    vector<Vertex> vertices_trans;
    vector<Face> faces; 

    // Now wak through each of the mesh's faces (a face is a mesh its triangle) and retrieve the corresponding vertex indices.

    for(GLuint i = 0; i < mesh->mNumFaces; i++)
    { 
      Vertex vertex;
      Vertex vertex_trans;
      glm::vec3 vector; 

      Face face_t;
      glm::vec3 faceindices_;

      aiFace face = mesh->mFaces[i];

      for(GLuint j = 0; j < face.mNumIndices; j++)
      {

        // First  transform the position.
  
        aiVector3D position = transform * mesh->mVertices[face.mIndices[j]];
        vector.x = position.x;
        vector.y = position.y;
        vector.z = position.z;
        vertex_trans.Position = vector;

        // Now transform the normals.
        aiVector3D normal = normalTransform * mesh->mNormals[face.mIndices[j]];

        vector.x = normal.x;
        vector.y = normal.y;
        vector.z = normal.z;
        vertex_trans.Normal = vector;

        vertices_trans.push_back(vertex_trans);

        //indice
        indices.push_back(face.mIndices[j]);

      }
                
      //Store the indices for every face
      faceindices_.x = face.mIndices[0]+ indexStart;
      faceindices_.y = face.mIndices[1]+ indexStart;
      faceindices_.z = face.mIndices[2]+ indexStart;
      face_t.Faceindices = faceindices_;
      face_t.Numindices=(int)face.mNumIndices;
      faces.push_back(face_t);

      // Retrieve all indices of the face and store them in the indices vector
    }
    indexStart += mesh->mNumVertices;
        
    // Return a mesh object created from the extracted mesh data
    return Mesh(vertices_trans, indices, faces);
  }
   
};


