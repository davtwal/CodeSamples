// DPmesh.cpp
// cs500 9/19


#include <cmath>
#include "DPmesh.h"
using namespace std;


const float PI = 4.0f*atan(1.0f);


int DPmesh::stemStartIndex(void) const {
  return 0;
}


int DPmesh::stemIndexCount(void) const {
  return 12;
}


int DPmesh::bowl1StartIndex(void) const {
  return 12;
}


int DPmesh::bowl1IndexCount(void) const {
  return bowl_face_count;
}


int DPmesh::bowl2StartIndex(void) const {
  return 12+bowl_face_count;
}


int DPmesh::bowl2IndexCount(void) const {
  return bowl_face_count;
}


DPmesh::DPmesh(int n)
    : bowl_face_count(4*n+4) {
  vertices.resize(48+8*n);
  normals.resize(48+8*n);
  faces.resize(20+8*n);

  // d/p common stem  (v/n: 0 .. 23, f: 0 .. 11)
  const glm::vec3 S0(0.1f,1,1),
                  S1(0.1f,-1,1),
                  S2(-0.1f,-1,1),
                  S3(-0.1f,1,1),
                  S4(0.1f,1,-1),
                  S5(0.1f,-1,-1),
                  S6(-0.1f,-1,-1),
                  S7(-0.1f,1,-1),
                  EX(1,0,0),
                  EY(0,1,0),
                  EZ(0,0,1);
  vertices[0] = S0;     normals[0] = EZ;     faces[0] = Face(0,2,1);
  vertices[1] = S1;     normals[1] = EZ;     faces[1] = Face(0,3,2);
  vertices[2] = S2;     normals[2] = EZ;
  vertices[3] = S3;     normals[3] = EZ;
  vertices[4] = S4;     normals[4] = -EZ;    faces[2] = Face(4,5,6);
  vertices[5] = S5;     normals[5] = -EZ;    faces[3] = Face(4,6,7);
  vertices[6] = S6;     normals[6] = -EZ;
  vertices[7] = S7;     normals[7] = -EZ;

  vertices[8] = S0;     normals[8] = EX;     faces[4] = Face(8,9,10);
  vertices[9] = S1;     normals[9] = EX;     faces[5] = Face(8,10,11);
  vertices[10] = S5;    normals[10] = EX;
  vertices[11] = S4;    normals[11] = EX;
  vertices[12] = S3;    normals[12] = -EX;   faces[6] = Face(12,15,14);
  vertices[13] = S2;    normals[13] = -EX;   faces[7] = Face(12,14,13);
  vertices[14] = S6;    normals[14] = -EX;
  vertices[15] = S7;    normals[15] = -EX;

  vertices[16] = S0;    normals[16] = EY;    faces[8] = Face(16,17,18);
  vertices[17] = S4;    normals[17] = EY;    faces[9] = Face(16,18,19);
  vertices[18] = S7;    normals[18] = EY;
  vertices[19] = S3;    normals[19] = EY;
  vertices[20] = S1;    normals[20] = -EY;   faces[10] = Face(20,23,22);
  vertices[21] = S5;    normals[21] = -EY;   faces[11] = Face(20,22,21);
  vertices[22] = S6;    normals[22] = -EY;
  vertices[23] = S2;    normals[23] = -EY;

  // p bowl (v/n: 24 .. 35+4n, f: 12 .. 16+4n)
  const glm::vec3 B0(0.2f,1,1),
                  B1(0.2f,0,1),
                  B2(0.2f,0,-1),
                  B3(0.2f,1,-1),
                  B4(0.44f,1,1),
                  B5(0.44f,0,1),
                  B6(0.44f,0,-1),
                  B7(0.44f,1,-1);
  vertices[24] = B0;    normals[24] = -EX;   faces[12] = Face(24,26,25);
  vertices[25] = B1;    normals[25] = -EX;   faces[13] = Face(24,27,26);
  vertices[26] = B2;    normals[26] = -EX;
  vertices[27] = B3;    normals[27] = -EX;

  vertices[28] = B1;  normals[28] = EZ;  // v/n: 28 .. 29+n
  for (int i=0; i < n; ++i) {
    float t = PI*(i/float(n-1)-0.5f);
    vertices[29+i] = 0.5f*(B4 + B5) + 0.5f*cos(t)*EX + 0.5f*sin(t)*EY;
    normals[29+i] = EZ;
  }
  vertices[29+n] = B0;  normals[29+n] = EZ;
  for (int i=0; i < n; ++i)              // f: 14 .. 13+n
    faces[14+i] = Face(28,29+i,30+i);

  for (int i=0; i < n+2; ++i) {   // v/n: 30+n .. 31+2n
    vertices[30+n+i] = vertices[28+i] - 2.0f*EZ;
    normals[30+n+i] = -EZ;
  }
  for (int i=0; i < n; ++i)     // f: 14+n .. 13+2n
    faces[14+n+i] = Face(30+n,31+n+i,32+n+i);

  for (int i=0; i < n+2; ++i) {  // v/n: 32+2n .. 33+3n & 34+3n .. 35+4n
    vertices[32+2*n+i] = vertices[28+i];
    vertices[34+3*n+i] = vertices[30+n+i];
  }
  normals[32+2*n] = normals[34+3*n] = -EY;
  for (int i=0; i < n; ++i) {
    float t = PI*(i/float(n-1)-0.5f);
    normals[33+2*n+i] = normals[35+3*n+i]
                      = cos(t)*EX + sin(t)*EY;
  }
  normals[33+3*n] = normals[35+4*n] = EY;
  for (int i=0; i < n+1; ++i) {  // f: 14+2n .. 15+4n
    faces[14+2*n+2*i+0] = Face(32+2*n+i,33+2*n+i,35+3*n+i);
    faces[14+2*n+2*i+1] = Face(32+2*n+i,35+3*n+i,34+3*n+i);
  }

  // d bowl
  for (int i=0; i < 4*n+12; ++i) {  // v/n: 36+4n .. 47+8n
    vertices[36+4*n+i] = glm::vec3(-1,-1,1) * vertices[24+i];
    normals[36+4*n+i] = glm::vec3(-1,-1,1) * normals[24+i];
  }
  for (int i=0; i < 4*n+4; ++i) { // f: 16+4n .. 19+8n
    const Face &f = faces[12+i];
    faces[16+4*n+i] = Face(f.index1+4*n+12,f.index2+4*n+12,f.index3+4*n+12);
  }

}

