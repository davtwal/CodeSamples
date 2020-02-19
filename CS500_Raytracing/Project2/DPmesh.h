// DPmesh.h
// -- 3D triangular mesh for DP logo
// cs500 9/19

#ifndef CS500_DPMESH_H
#define CS500_DPMESH_H


#include <vector>
#include "glm/glm.hpp"


struct DPmesh {
  DPmesh(int n=20);
  std::vector<glm::vec3> vertices;
  std::vector<glm::vec3> normals;
  struct Face {
    Face(int i=-1, int j=-1, int k=-1) : index1(i), index2(j), index3(k) {}
    int index1, index2, index3;
  };
  std::vector<Face> faces;

  // face index ranges
  int bowl_face_count;
  int stemStartIndex(void) const;
  int stemIndexCount(void) const;
  int bowl1StartIndex(void) const;
  int bowl1IndexCount(void) const;
  int bowl2StartIndex(void) const;
  int bowl2IndexCount(void) const;
};


#endif

