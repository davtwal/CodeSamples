// RayCaster.h
// -- ray casting partial framework
// cs500 9/19

#ifndef CS500_RAYCASTER_H
#define CS500_RAYCASTER_H


#include <vector>
#include "glm/glm.hpp"
#include "DPmesh.h"


/////////////////////////////////////////////////////////////////
struct Ray;
struct Camera;
struct IntersectionData;
struct Scene;
class Bitmap;

// Objects
struct Object;
struct Sphere;
struct Ellipsoid;
struct Box;
struct Polygon;

struct DPMeshObj;

void      clearScene(Scene& s);
glm::vec3 applyLighting(const IntersectionData& d, const Scene& s);
glm::vec3 castRay(const Ray& r, const Scene& s, int depth = 0);
void      loadFile(const char* fname, Scene& s);
void      renderScene(const Scene& s, Bitmap& b, bool* kill = nullptr);


/////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////

struct Ray {
  glm::vec3 origin;
  glm::vec3 dir;
};


struct Object {
  glm::vec3 diffuse_coef = {1, 1, 1};
  glm::vec3 atten_coef = { 0, 0, 0 };
  float specular_coef = 0.f;
  float specular_exp = 1.f;
  float permittivity = 1.1f;
  float permeability = 1.1f;

  virtual ~Object(void) {
  }

  virtual bool isIntersection(const Ray& r, IntersectionData& data) = 0;
  virtual void getIntersectionData(const Ray& r, IntersectionData& data) = 0;
};


struct IntersectionData {
  float     depth  = std::numeric_limits<float>::max();
  glm::vec3 point  = {0, 0, 0},
            normal = {0, 0, 0},
            view   = {0, 0, 0};
  Object* object   = nullptr;
};


struct Sphere : Object {
  Sphere(glm::vec3 const& c = {0, 0, 0}, float r = 1)
    : center(c),
      radius(r) {
  }

  glm::vec3 center;
  float     radius;
  bool      isIntersection(const Ray& r, IntersectionData& data) override;
  void      getIntersectionData(const Ray& r, IntersectionData& d) override;
};

struct Ellipsoid : Object {
  glm::vec3 center;
  glm::vec3 u, v, w;

  bool      precomputeDone = false;
  glm::mat3 L;
  glm::mat3 Linv;

  void preprocess();

  bool isIntersection(const Ray& r, IntersectionData& data) override;
  void getIntersectionData(const Ray& r, IntersectionData& d) override;
};

struct TVal {
  TVal(float _min = 0,
       float _max = std::numeric_limits<float>::max())
    : min(_min),
      max(_max) {
  }

  float min = 0;
  float max = std::numeric_limits<float>::max();

  static TVal Intersect(TVal const& a, TVal const& b) {
    return TVal{
      a.min > b.min ? a.min : b.min,
      a.max > b.max ? b.max : a.max
    };
  }

  bool isValid() const;
};

struct HalfPlane {
  glm::vec3 point;
  glm::vec3 normal; // points outwards

  TVal intersect(const Ray& r) const;
};

struct Box : Object {
  glm::vec3 center = {0, 0, 0};
  glm::vec3 u      = {1, 0, 0},
            v      = {0, 1, 0},
            w      = {0, 0, 1};

  std::vector<HalfPlane> halfPlanes;
  bool                   halfPlanesComputed = false;

  void preprocess();

  bool isIntersection(const Ray& r, IntersectionData& data) override;
  void getIntersectionData(const Ray& r, IntersectionData& d) override;
};

struct Polygon : Object {
  std::vector<glm::vec3> points;

  HalfPlane halfPlane = {};

  bool isIntersection(const Ray& r, IntersectionData& data) override;
  void getIntersectionData(const Ray& r, IntersectionData& d) override;
};

struct DPMeshObj : DPmesh, Object {
  glm::vec3 bb_center = {0, 0, 0};
  glm::vec3 bb_u      = {1, 0, 0},
            bb_v      = {0, 1, 0},
            bb_w      = {0, 0, 1};

  glm::mat3 cob_to_bb;
  glm::mat3 cob_to_std;

  void preprocess();

  void getMeshIntersectionData(const Ray& r, IntersectionData& d);

  bool isIntersection(const Ray& r, IntersectionData& data) override;
  void getIntersectionData(const Ray& r, IntersectionData& d) override;
};

struct Camera {
  glm::vec3 center = glm::vec3(0, 0, 0),
            right  = glm::vec3(1, 0, 0),
            up     = glm::vec3(0, 1, 0),
            eye    = glm::vec3(0, 0, 1);
  Ray getRay(float x, float y) const;
};

struct Light {
  glm::vec3 pos     = {0.f, 0.f, 0.f};
  glm::vec3 color   = {1.f, 1.f, 1.f};
  float     radius  = 1.f;
};

struct Scene {
  std::vector<Object*>  objects;
  std::vector<Light>    lights;
  Camera                camera;
  glm::vec3             globalAmbient {0.f };
};

class Bitmap {
public:
  Bitmap(int W, int H)
    : bmp_width(W),
      bmp_height(H),
      bmp_stride(4 * int(ceil(0.75f * W))),
      data(new unsigned char[H * bmp_stride]) {
  }

  ~Bitmap(void) {
    delete data;
  }

  char* bytes(void) {
    return reinterpret_cast<char*>(data);
  }

  int width(void) const {
    return bmp_width;
  }

  int height(void) const {
    return bmp_height;
  }

  int stride(void) const {
    return bmp_stride;
  }

  void setPixel(int i, int j, int r, int g, int b) {
    data[bmp_stride * j + 3 * i + 0] = r;
    data[bmp_stride * j + 3 * i + 1] = g;
    data[bmp_stride * j + 3 * i + 2] = b;
  }

private:
  int bmp_width,
      bmp_height,
      bmp_stride;
  unsigned char* data;
};


#endif
