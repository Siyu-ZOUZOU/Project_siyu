#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
// GL Includes
#include <GL/glew.h> // Contains all the necessery OpenGL includes
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

struct Vertex
{
  // Position
  glm::vec3 Position;
  // Normal
  glm::vec3 Normal;
};

struct Face
{
  // indices of face
  int Numindices;
  glm::vec3 Faceindices;
};

class Mesh 
{
public:
  /*  Mesh Data  */
  vector<Vertex> vertices;
  vector<GLuint> indices;
  vector<Face> faces;

  /*  Functions  */
  // Constructor
  Mesh(vector<Vertex> vertices, vector<GLuint> indices, vector<Face> faces)
  {
    this->vertices = vertices;
    this->indices = indices;
    this->faces = faces;
  } 
/*
    void Output() 
    {
        for(auto& it : vertices) {
            const glm::vec3& pt_po = it.Position;
            const glm::vec3& pt_no = it.Normal;
            cout<< "coordinate x: " <<(float)pt_po.x <<endl;
            cout<< "normal x: " <<(float)pt_no.x <<endl;
        }
        cout << indices.size()<<endl;
    }
*/

//Try to write to ply

  void writePLY_cor(int count, int NumberVertex, int NumberFace)
  {


        ////
        // Header
        ////

    if(count == 0)
    {

      ofstream outFile( "Output.ply" );
      if ( !outFile ) 
      {
        //cerr << "Error opening output file: " << Output << "!" << endl;
        printf("Error opening output file: %s!\n", "Output.ply");
        exit( 1 );
      }

      outFile << "ply" << endl;
      outFile << "format ascii 1.0" << std::endl;
      outFile << "element vertex " << NumberVertex << std::endl;
      outFile << "property float x" << std::endl;
      outFile << "property float y" << std::endl;
      outFile << "property float z" << std::endl;

      outFile << "property float nx" << std::endl;
      outFile << "property float ny" << std::endl;
      outFile << "property float nz" << std::endl;
      outFile << "element face " << NumberFace << std::endl;
      outFile << "property list uchar int vertex_indices" << std::endl;
      outFile << "end_header" << std::endl;

      for(auto& it : vertices) 
      {
        const glm::vec3& pt_po = it.Position;
        const glm::vec3& pt_no = it.Normal;
        outFile << (float)pt_po.x << " "<< (float)pt_po.y << " "<< (float)pt_po.z;
        outFile <<" "<< (float)pt_no.x << " "<< (float)pt_no.y << " "<< (float)pt_no.z;
        outFile << std::endl;
      }
            
    }

        ////
        // Points
        ////

    else 
    {
      ofstream ofresult( "Output.ply", ios::app); 
            
      for(auto& it : vertices) 
      {
        const glm::vec3& pt_po = it.Position;
        const glm::vec3& pt_no = it.Normal;
        ofresult << (float)pt_po.x << " "<< (float)pt_po.y << " "<< (float)pt_po.z;
        ofresult <<" "<< (float)pt_no.x << " "<< (float)pt_no.y << " "<< (float)pt_no.z;
        ofresult << std::endl;
      }
    }  
  }
  void writePLY_face()
  {
    ofstream ofresult( "Output.ply", ios::app); 
  
    for(auto& itr : faces) 
    {
      const int& pt_NumInd = itr.Numindices;
      const glm::vec3& pt_FaceInd = itr.Faceindices;
      ofresult << pt_NumInd<< " " <<(int)pt_FaceInd.x << " "<< (int)pt_FaceInd.y << " "<< (int)pt_FaceInd.z;
      ofresult << std::endl;
    }
  }
};

