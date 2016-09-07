#include "Model.h"
#include "Mesh.h"

int main(int argc, char** argv) {

  static const std::string modelFileName =  argv[1];
  Model MyLittleModel(modelFileName);
  MyLittleModel.OutputTest();
}
