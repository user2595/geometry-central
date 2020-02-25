#pragma once

#include <cstdlib>
#include <fstream>
#include <memory>
#include <regex>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <geometrycentral/utilities/utilities.h>
#include <geometrycentral/utilities/vector2.h>
#include <geometrycentral/utilities/vector3.h>

namespace geometrycentral {

class PolygonSoupMesh {
public:
  PolygonSoupMesh();
  PolygonSoupMesh(std::string meshFilename, bool loadTexture = false);
  PolygonSoupMesh(std::string meshFilename, std::string type);
  PolygonSoupMesh(const std::vector<std::vector<size_t>>& polygons_, const std::vector<Vector3>& vertexCoordinates_);

  // Mutate this mesh and by naively triangulating polygons
  void triangulate();

  // Mesh data
  std::vector<std::vector<size_t>> polygons;
  std::vector<Vector3> vertexCoordinates;

  std::vector<std::vector<Vector2>> textureCoordinates;

  void mergeIdenticalVertices();
  void mergeByDistance(double tol);

private:
  void readMeshFromFile(std::string filename, bool loadTexture = false);
  void readMeshFromStlFile(std::string filename);
  void readMeshFromAsciiStlFile(std::ifstream& in);
  void readMeshFromBinaryStlFile(std::ifstream in);

  std::tuple<std::vector<Vector3>, std::vector<size_t>> merge(const std::vector<Vector3>& points, double tol);
  std::tuple<std::vector<Vector3>, std::vector<size_t>> naiveMerge(const std::vector<Vector3>& points, double tol);
};

} // namespace geometrycentral
