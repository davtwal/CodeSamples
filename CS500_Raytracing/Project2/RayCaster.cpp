// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * CS500P1 : RayCaster.cpp
// * Copyright (C) DigiPen Institute of Technology 2019
// * 
// * Created     : 2019y 10m 01d
// * Last Altered: 2019y 10m 01d
// * 
// * Author      : David Walker
// * E-mail      : d.walker\@digipen.edu
// * 
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * Description :

#include <limits>
#include <string>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include "RayCaster.h"
using namespace std;

// DEFINES AVAILABLE:
// VERBOSE - Makes print-outs of when things get parsed
// MULTITHREAD_SCENE_RENDER - Multithreads scene rendering, splitting the bitmap image into sections:
//   - 3x3 in debug mode
//   - 10x10 in release mode.
//   Each section runs on a seperate thread using C++11 threads
#define MULTITHREAD_SCENE_RENDER
#define VERBOSE

static constexpr float    EPSILON                 = 0.0001f;
static constexpr float    BUMP_EPSILON            = EPSILON * 10;
static constexpr unsigned MAX_RAY_RECURSION_DEPTH = 10;

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
// OBJECTS
/////////////////////////////////////////////////////////////////
/// sphere object ///////////////////////////////////////////////
bool Sphere::isIntersection(const Ray& r, IntersectionData& id) {
  if (!id.object || id.depth == numeric_limits<float>::max())
    return false;
  return true;
}

void Sphere::getIntersectionData(const Ray& r, IntersectionData& id) {
  // |P0 - C|^2 + 2td0(P0 - C) + t^2|d0|^2 = 0
  // a = dot(d0, d0)
  // b = 2d0(P0 - C)
  // c = |P0 - C|^2 - r^2
  glm::vec3 p0c = r.origin - center;
  float     a   = dot(r.dir, r.dir);
  float     b   = 2 * dot(r.dir, p0c);
  float     c   = dot(p0c, p0c) - radius * radius;

  float discr = b * b - 4 * a * c;

  if (discr < 0) {
    // no intersection
    id.object = nullptr;
    id.depth  = numeric_limits<float>::max();
    id.point  = glm::vec3{0};
    id.normal = glm::vec3{0};
    id.view   = glm::vec3{0};
    return;
  }

  float sqrt_discr = sqrt(discr);
  float t_minus    = (-b - sqrt_discr) / (2 * a);
  float t_plus     = (-b + sqrt_discr) / (2 * a);

  if (t_plus < EPSILON)
    return;

  // t value gotten
  id.object = this;
  id.depth  = abs(t_minus) < EPSILON ? t_plus : t_minus;
  id.point  = r.origin + id.depth * r.dir;
  id.normal = id.point - center;
  id.view   = -r.dir;
}

/////////////////////////////////////////////////////////////////
/// ellipsoid object ////////////////////////////////////////////
bool Ellipsoid::isIntersection(const Ray& r, IntersectionData& id) {
  if (!id.object || id.depth == numeric_limits<float>::max())
    return false;
  return true;
}

void Ellipsoid::preprocess() {
  if (!precomputeDone) {
    L              = glm::mat3(u, v, w);
    Linv           = inverse(L);
    precomputeDone = true;
  }
}

void Ellipsoid::getIntersectionData(const Ray& r, IntersectionData& id) {
  // Point on ellipsoid P = M(p') = C + Lp'
  static Sphere unitSphere;

  Ray transf_ray = {
    Linv * (r.origin - center),
    Linv * r.dir
  };

  unitSphere.getIntersectionData(transf_ray, id);

  if (id.object) {
    id.object = this;
    id.point  = L * id.point + center;
    id.normal = transpose(Linv) * id.normal;
    id.view   = -r.dir;
  }
}

/////////////////////////////////////////////////////////////////
/// ellipsoid object ////////////////////////////////////////////
bool Box::isIntersection(const Ray& r, IntersectionData& id) {
  if (!id.object || id.depth == numeric_limits<float>::max())
    return false;
  return true;
}

bool TVal::isValid() const {
  return max - min > -EPSILON;
}

bool pointInHalfPlane(const HalfPlane& hp, const glm::vec3& p) {
  return dot(hp.normal, hp.point - p) <= 0;
}

TVal HalfPlane::intersect(const Ray& r) const {
  glm::vec3 o_c      = r.origin - point;
  float     oc_dot_n = dot(o_c, normal);
  float     d_dot_n  = dot(r.dir, normal);

  TVal retval;

  if (abs(d_dot_n) < EPSILON) {
    if (oc_dot_n > 0) {
      std::swap(retval.min, retval.max);
    }
  }
  else {
    float t = -oc_dot_n / d_dot_n;

    if (d_dot_n < 0)
      retval.min = t;
    else
      retval.max = t;
  }

  return retval;
}

void Box::preprocess() {
  if (!halfPlanesComputed) {
    halfPlanes.resize(6);

    // Construct halfplanes
    halfPlanes[0].point = center + u;
    halfPlanes[1].point = center + v;
    halfPlanes[2].point = center + w;
    halfPlanes[3].point = center - u;
    halfPlanes[4].point = center - v;
    halfPlanes[5].point = center - w;

    glm::mat3 basisMtr(u, v, w);
    basisMtr = transpose(inverse(basisMtr));

    halfPlanes[0].normal = basisMtr * glm::vec3(1, 0, 0);
    halfPlanes[1].normal = basisMtr * glm::vec3(0, 1, 0);
    halfPlanes[2].normal = basisMtr * glm::vec3(0, 0, 1);
    halfPlanes[3].normal = basisMtr * glm::vec3(-1, 0, 0);
    halfPlanes[4].normal = basisMtr * glm::vec3(0, -1, 0);
    halfPlanes[5].normal = basisMtr * glm::vec3(0, 0, -1);

    halfPlanesComputed = true;
  }
}

void Box::getIntersectionData(const Ray& r, IntersectionData& id) {
  if (!halfPlanesComputed)
    preprocess();

  id.depth = numeric_limits<float>::max();

  TVal       tval;
  HalfPlane* minHP = nullptr;
  HalfPlane* maxHP = nullptr;
  for (int i = 0; i < halfPlanes.size() && tval.isValid(); ++i) {
    TVal hpt = halfPlanes[i].intersect(r);

    if (!hpt.isValid())
      return;

    tval = TVal::Intersect(tval, hpt);
    if (tval.min == hpt.min)
      minHP = &halfPlanes[i];
    if (tval.max == hpt.max)
      maxHP = &halfPlanes[i];
  }

  if (!tval.isValid())
    return;

  if (tval.min < EPSILON) {
    if (!maxHP)
      return;

    id.depth  = tval.max;
    id.normal = maxHP->normal;
  }
  else {
    if (!minHP)
      return;

    id.depth  = tval.min;
    id.normal = minHP->normal;
  }

  id.object = this;
  id.point  = r.origin + id.depth * r.dir;
  id.view   = -r.dir;
}

/////////////////////////////////////////////////////////////////
/// polygon object //////////////////////////////////////////////

bool isInTriangle(glm::vec3 const& p,
                  glm::vec3 const& v0,
                  glm::vec3 const& v1,
                  glm::vec3 const& v2,
                  glm::vec3*       out_bary = nullptr) {


  glm::vec3 v0v1 = v1 - v0; // a
  glm::vec3 v0v2 = v2 - v0; // b
  glm::vec3 v0p  = p - v0; // P - C

  glm::vec3 n = cross(v0v1, v0v2);

  float a_b = dot(v0v1, v0v2);

  glm::mat2 figure = inverse(glm::mat2{
                               {dot(v0v1, v0v1), a_b},
                               {a_b, dot(v0v2, v0v2)}
                             });

  glm::vec2 uv = figure * glm::vec2{dot(v0p, v0v1), dot(v0p, v0v2)};

  if (out_bary)
    *out_bary = {uv.x, uv.y, 1 - uv.x - uv.y};

  return uv.x > -EPSILON && uv.y > -EPSILON && uv.x + uv.y <= 1;
}

bool Polygon::isIntersection(const Ray& r, IntersectionData& id) {
  if (!id.object || id.depth == numeric_limits<float>::max())
    return false;
  return true;
}

void Polygon::getIntersectionData(const Ray& r, IntersectionData& id) {
  TVal tval = halfPlane.intersect(r);

  if (tval.isValid()) {
    id.depth = tval.min < EPSILON ? tval.max : tval.min;
    id.point = id.depth * r.dir + r.origin;

    bool found = false;
    for (unsigned i = 1; !found && i < points.size() - 1; ++i) {
      found = isInTriangle(id.point, points.front(), points[i], points[i + 1]);
    }

    if (!found)
      return;

    id.object = this;
    id.normal = halfPlane.normal;
    id.view   = -r.dir;
  }
}

/////////////////////////////////////////////////////////////////
/// dpmesh object ///////////////////////////////////////////////

bool DPMeshObj::isIntersection(const struct Ray& r, IntersectionData& id) {
  if (!id.object || id.depth == numeric_limits<float>::max())
    return false;
  return true;
}

void DPMeshObj::preprocess() {
  cob_to_std = glm::mat3(bb_u, bb_v, bb_w);
  cob_to_bb  = inverse(cob_to_std);
}

void DPMeshObj::getMeshIntersectionData(const Ray& r, IntersectionData& id) {
  float earliestBoundary = std::numeric_limits<float>::max();

  glm::vec3 earliestNormal = {0, 0, 0};

  for (auto& face : faces) {
    glm::vec3 &v0 = vertices[face.index1],
              &v1 = vertices[face.index2],
              &v2 = vertices[face.index3];

    glm::vec3 &n0 = normals[face.index1],
              &n1 = normals[face.index2],
              &n2 = normals[face.index3];

    glm::vec3 faceNorm = cross(v1 - v0, v2 - v0);
    if (dot(faceNorm, n0) < 0)
      faceNorm = -faceNorm;

    HalfPlane hp{v0, faceNorm};

    TVal tval = hp.intersect(r);

    if (tval.isValid()) {
      glm::vec3 barycentric = {0, 0, 0};

      if (tval.min > EPSILON) {
        glm::vec3 point = r.dir * tval.min + r.origin;

        if (isInTriangle(point, v0, v1, v2, &barycentric)) {
          if (earliestBoundary > tval.min) {
            faceNorm         = n0 * barycentric.z + n1 * barycentric.x + n2 * barycentric.y;
            earliestBoundary = tval.min;
            earliestNormal   = normalize(faceNorm);
          }
        }
      }

      else {
        glm::vec3 point = r.dir * tval.max + r.origin;

        if (isInTriangle(point, v0, v1, v2, &barycentric)) {
          if (earliestBoundary > tval.max) {
            faceNorm         = n0 * barycentric.z + n1 * barycentric.x + n2 * barycentric.y;
            earliestBoundary = tval.max;
            earliestNormal   = -normalize(faceNorm); // flipped
          }
        }
      }
    }
  }

  if (earliestBoundary != std::numeric_limits<float>::max() && earliestBoundary > EPSILON) {
    id.depth  = earliestBoundary;
    id.normal = earliestNormal;
    id.point = id.depth * r.dir + r.origin;
    // for the issue i sent an e-mail about, this was where the issue was.
    // i wasn't setting id.point - whoops! dumb mistake
  }

  else
    return;

  id.object = this;
}

void DPMeshObj::getIntersectionData(const Ray& r, IntersectionData& id) {
  Ray transf_ray{
    cob_to_bb * (r.origin - bb_center),
    cob_to_bb * r.dir
  };

  static Box unitBox;
  unitBox.getIntersectionData(transf_ray, id);

  if (id.object) {
    id = IntersectionData();

    getMeshIntersectionData(transf_ray, id);
    if (id.object) {
      id.view   = -r.dir;
      id.point  = cob_to_std * id.point + bb_center;
      id.normal = transpose(cob_to_bb) * id.normal;
    }
  }
}

/////////////////////////////////////////////////////////////////
// camera
/////////////////////////////////////////////////////////////////
Ray Camera::getRay(float x, float y) const {
  Ray r = {};

  glm::vec3 p = center + x * right + y * up;

  r.origin = center + eye;
  r.dir    = normalize(p - r.origin);

  return r;
}


/////////////////////////////////////////////////////////////////
// helpers
/////////////////////////////////////////////////////////////////
void clearScene(Scene& s) {
  for (unsigned i = 0; i < s.objects.size(); ++i)
    delete s.objects[i];

  s.globalAmbient = {0, 0, 0};
  s.objects.clear();
  s.lights.clear();
}

IntersectionData rayTraceObjects(const Ray& r, const Scene& s) {
  IntersectionData best_intersect;
  best_intersect.depth = numeric_limits<float>::max();

  for (Object* obj : s.objects) {
    IntersectionData data;
    obj->getIntersectionData(r, data);

    if (data.object && data.depth < best_intersect.depth)
      best_intersect = data;
  }

  return best_intersect;
}

glm::vec3 applyLighting(const IntersectionData& d, const Scene& s) {
  assert(d.object);

  glm::vec3 N = normalize(d.normal);
  glm::vec3 V = normalize(d.view);

  glm::vec3 n_v = N - V;
  if (abs(n_v.x) < EPSILON && abs(n_v.y) < EPSILON && abs(n_v.z) < EPSILON)
    cout << "Equivalent normal and view";

  glm::vec3 color(0.f);
  for (const Light& light : s.lights) {

    glm::vec3 L       = light.pos - d.point;
    float     light_t = length(L);

    L = normalize(L);

    Ray shadowFeeler;
    shadowFeeler.origin = d.point + BUMP_EPSILON * N;  // bump to remove shadow acne
    shadowFeeler.dir    = L;

    IntersectionData intersectData = rayTraceObjects(shadowFeeler, s);

    // if in shadow, skip light
    if (intersectData.object && intersectData.depth > -EPSILON && intersectData.depth < light_t) {
      continue;
    }

    float n_l = dot(N, L);

    // check if diffuse
    if (n_l > EPSILON)
      color += d.object->diffuse_coef * light.color * n_l;

    // note: glm::reflect expects that the incident vector is pointing towards the
    //       surface: e.g. dot(I, N) < 0, so we don't use that here.
    glm::vec3 R = normalize(2 * n_l * N - L);

    float v_r     = dot(V, R);
    float v_r_exp = pow(v_r, d.object->specular_exp);

    // check if specular
    if (v_r > EPSILON)
      color += light.color * d.object->specular_coef * v_r_exp;
  }

  return color + s.globalAmbient * d.object->diffuse_coef;
}

glm::vec3 castRay(const Ray& r, const Scene& s, int depth) {
  glm::vec3 color(0);

  if (depth < 0)
    return color;

  IntersectionData best_intersect = rayTraceObjects(r, s);

  if (!best_intersect.object)
    return color;

  // local illumination
  color = applyLighting(best_intersect, s);

  // global reflections
  float R = best_intersect.object->specular_coef;

  // note: glm::reflect expects that the incident vector is pointing towards the
  //       surface: e.g. dot(I, N) < 0, so we use it here
  Ray reflectRay = {
    best_intersect.point + BUMP_EPSILON * best_intersect.normal,
    reflect(normalize(r.dir), normalize(best_intersect.normal))
  };

  return color + R * castRay(reflectRay, s, depth - 1);
}

#ifndef MULTITHREAD_SCENE_RENDER
void renderScene(const Scene& s, Bitmap& b, bool* kp) {
  bool   defkill = false,
        *killptr = (kp == nullptr) ? &defkill : kp,
        &kill    = *killptr;

  for (int j = 0; !kill && j < b.height(); ++j)
    for (int i = 0; !kill && i < b.width(); ++i) {
      float x = 2.0f * (i + 0.5f) / b.width() - 1.0f,
            y = 2.0f * (j + 0.5f) / b.height() - 1.0f;

      Ray       r     = s.camera.getRay(x, y);
      glm::vec3 color = castRay(r, s, MAX_RAY_RECURSION_DEPTH);

      // gamma correction
//float igamma = 1.0f/2.2f;
//color = glm::vec3(pow(color.r,igamma),
//                  pow(color.g,igamma),
      //                  pow(color.b,igamma));

      color = glm::clamp(255.0f * color, 0.0f, 255.0f);
      b.setPixel(i, j, color.r, color.g, color.b);
    }
}
#else

#include <thread>

void renderScene(const Scene& s, Bitmap& b, bool* kp) {
  bool   defkill = false,
        *killptr = (kp == nullptr) ? &defkill : kp,
        &kill    = *killptr;

  // my computer has 12 "cores" (6 hyper-threaded)
  static unsigned XSECTS = 3;
  static unsigned YSECTS = 4;

  auto drawFN = [&](int ybegin, int yend, int xbegin, int xend) {
    for (int j = ybegin; !kill && j < yend; ++j) {
      for (int i = xbegin; !kill && i < xend; ++i) {
        float x = 2.0f * (i + 0.5f) / b.width() - 1.0f,
              y = 2.0f * (j + 0.5f) / b.height() - 1.0f;

        Ray       r     = s.camera.getRay(x, y);
        glm::vec3 color = castRay(r, s, MAX_RAY_RECURSION_DEPTH);

        // gamma correction
        //float igamma = 1.0f/2.2f;
        //color = glm::vec3(pow(color.r,igamma),
        //                  pow(color.g,igamma),
        //                  pow(color.b,igamma));

        color = clamp(255.0f * color, 0.0f, 255.0f);
        b.setPixel(i, j, color.r, color.g, color.b);
      }
    }
  };

  std::vector<std::thread> threads;
  threads.reserve(XSECTS * YSECTS);

  const int pix_per_x_sect = b.width() / XSECTS;
  const int pix_per_y_sect = b.height() / YSECTS;
  const int leftover_x_pix = b.width() % XSECTS;
  const int leftover_y_pix = b.height() % YSECTS;

  for (int j = 0; j < YSECTS; ++j) {
    int ybegin = j * pix_per_y_sect;
    int yend   = (j + 1) * pix_per_y_sect;

    if (j == YSECTS - 1) {
      yend += leftover_y_pix;
    }

    for (int i = 0; i < XSECTS; ++i) {
      int xbegin = i * pix_per_x_sect;
      int xend   = (i + 1) * pix_per_x_sect;

      if(i == XSECTS - 1) {
        xend += leftover_x_pix;
      }

      threads.emplace_back(drawFN, ybegin, yend, xbegin, xend);
    }
  }

  for (auto& thread : threads)
    thread.join();
}
#endif


/////////////////////////////////////////////////////////////////
// file parsing
/////////////////////////////////////////////////////////////////
bool parseTriple(istream& in, float& x, float& y, float& z) {
  char lp, c1, c2, rp;
  in >> lp >> x >> c1 >> y >> c2 >> z >> rp;
  return bool(in) && lp == '(' && c1 == ',' && c2 == ',' && rp == ')';
}


bool parseTriple(istream& in, glm::vec3& P) {
  return parseTriple(in, P.x, P.y, P.z);
}


bool parseFloat(istream& in, float& x) {
  in >> x;
  return bool(in);
}


bool parseInt(istream& in, int& n) {
  in >> n;
  return bool(in);
}


bool parseMaterial(istream& in, Object& o) {
  return parseTriple(in, o.diffuse_coef)
         && parseFloat(in, o.specular_coef)
         && parseFloat(in, o.specular_exp)
         && parseTriple(in, o.atten_coef)
         && parseFloat(in, o.permittivity)
         && parseFloat(in, o.permeability);
}

#ifdef VERBOSE
#include <iomanip>
#endif

void loadFile(const char* fname, Scene& s) {
  ifstream in(fname);

  while (in) {
    string start;
    in >> start;
    if (start[0] == '#')
      in.ignore(numeric_limits<streamsize>::max(), '\n');
    else {

      if (start == "CAMERA") {
        Camera cam;
        bool   okay = parseTriple(in, cam.center)
                      && parseTriple(in, cam.right)
                      && parseTriple(in, cam.up)
                      && parseTriple(in, cam.eye);
        if (!okay)
          throw runtime_error("failed to parse camera");
        s.camera = cam;
#ifdef VERBOSE
        cout << "camera parsed: " << endl;
#endif
      }

      else if (start == "SPHERE") {
        Sphere* sphere = new Sphere();
        bool    okay   = parseTriple(in, sphere->center)
                         && parseFloat(in, sphere->radius)
                         && parseMaterial(in, *sphere);
        if (!okay)
          throw runtime_error("failed to parse sphere");
#ifdef VERBOSE
        cout << "sphere parsed: " << sphere->center.x << " " << sphere->center.y << " " << sphere->center.z << endl;
#endif
        s.objects.push_back(sphere);
      }

      else if (start == "ELLIPSOID") {
        Ellipsoid* ellipsoid = new Ellipsoid();
        bool       okay      = parseTriple(in, ellipsoid->center)
                               && parseTriple(in, ellipsoid->u)
                               && parseTriple(in, ellipsoid->v)
                               && parseTriple(in, ellipsoid->w)
                               && parseMaterial(in, *ellipsoid);

        if (!okay)
          throw runtime_error("failed to parse ellipsoid");

        ellipsoid->preprocess();
        s.objects.push_back(ellipsoid);

#ifdef VERBOSE
        cout << "ellipsoid parsed: c("
            << ellipsoid->diffuse_coef.x << ", "
            << ellipsoid->diffuse_coef.y << ", "
            << ellipsoid->diffuse_coef.z << ")"
            << endl;
#endif
      }

      else if (start == "BOX") {
        Box* box  = new Box();
        bool okay = parseTriple(in, box->center)
                    && parseTriple(in, box->u)
                    && parseTriple(in, box->v)
                    && parseTriple(in, box->w)
                    && parseMaterial(in, *box);

        if (!okay)
          throw runtime_error("failed to parse box");

        box->preprocess();
        s.objects.push_back(box);

#ifdef VERBOSE
        cout << "box parsed" << endl;
#endif
      }

      else if (start == "POLYGON") {
        Polygon* poly     = new Polygon();
        int      numVerts = 0;
        bool     ok       = parseInt(in, numVerts);

        if (numVerts && ok) {
          poly->points.resize(numVerts);
          for (int i = 0; ok && i < numVerts; ++i)
            ok       = parseTriple(in, poly->points[i]);
        }

        ok = ok && parseMaterial(in, *poly);

        if (!ok)
          throw runtime_error("failed to parse polygon");

        if (numVerts > 2) {
          poly->halfPlane.point  = poly->points[0];
          poly->halfPlane.normal =
              cross(poly->points[1] - poly->points[0], poly->points[2] - poly->points[0]);
        }
        s.objects.push_back(poly);

#ifdef VERBOSE
        cout << "polygon parsed" << endl;
#endif
      }

      else if (start == "DPLOGO") {
        DPMeshObj* dp = new DPMeshObj();

        bool okay = parseTriple(in, dp->bb_center)
                    && parseTriple(in, dp->bb_u)
                    && parseTriple(in, dp->bb_v)
                    && parseTriple(in, dp->bb_w)
                    && parseMaterial(in, *dp);

        if (!okay)
          throw runtime_error("failed to parse dplogo");

        dp->preprocess();
        s.objects.push_back(dp);

#ifdef VERBOSE
        cout << "dplogo parsed" << endl;
#endif
      }

      else if (start == "LIGHT") {
        Light l;
        bool  okay = parseTriple(in, l.pos)
                     && parseTriple(in, l.color)
                     && parseFloat(in, l.radius);

        if (!okay)
          throw runtime_error("failed to parse light");

        //if (s.lights.empty())
        s.lights.push_back(l);
        //else s.lights.front() = l;

#ifdef VERBOSE
        cout << "light parsed: (" << setw(4) << l.pos.x << "," << l.pos.y << "," << l.pos.z << ")" << endl;
#endif
      }

      else if (start == "AMBIENT") {
        if (!parseTriple(in, s.globalAmbient))
          throw runtime_error("failed to parse ambient");

#ifdef VERBOSE
        cout << "parsed ambient" << endl;
#endif
      }

      else if (in) {
#ifdef VERBOSE
        cout << "unrecognized symbol: " << start << endl;
#endif
        in.ignore(numeric_limits<streamsize>::max(), '\n');
      }
    }

  }
}
