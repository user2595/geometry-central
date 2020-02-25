#include <map>
#include <set>
#include <sstream>
#include <string>

#include "geometrycentral/surface/halfedge_mesh.h"
#include "geometrycentral/surface/polygon_soup_mesh.h"

namespace geometrycentral {

PolygonSoupMesh::PolygonSoupMesh() {}

PolygonSoupMesh::PolygonSoupMesh(std::string meshFilename, bool loadTexture) {
  readMeshFromFile(meshFilename, loadTexture);
}

// TODO, char* can get cast to bool, so using bool and string as optional arguments leads to horrible bugs
PolygonSoupMesh::PolygonSoupMesh(std::string meshFilename, std::string type) {

  // Attempt to detect filename
  bool typeGiven = type != "";
  std::string::size_type sepInd = meshFilename.rfind('.');
  if (!typeGiven) {
    if (sepInd != std::string::npos) {
      std::string extension;
      extension = meshFilename.substr(sepInd + 1);

      // Convert to all lowercase
      std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
      type = extension;
    }
  }

  if (type == "obj") {
    readMeshFromFile(meshFilename, false);
  } else if (type == "stl") {
    readMeshFromStlFile(meshFilename);
  } else {
    if (typeGiven) {
      throw std::runtime_error("Did not recognize mesh file type " + type);
    } else {
      throw std::runtime_error("Could not detect file type to load mesh from " + meshFilename + ". (Found type " +
                               type + ", but cannot load this)");
    }
  }
}

PolygonSoupMesh::PolygonSoupMesh(const std::vector<std::vector<size_t>>& polygons_,
                                 const std::vector<Vector3>& vertexCoordinates_)
    : polygons(polygons_), vertexCoordinates(vertexCoordinates_) {}

// String manipulation helpers to parse .obj files
// See http://stackoverflow.com/a/236803
std::vector<std::string>& split(const std::string& s, char delim, std::vector<std::string>& elems) {
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}
std::vector<std::string> split(const std::string& s, char delim) {
  std::vector<std::string> elems;
  split(s, delim, elems);
  return elems;
}

class Index {
public:
  Index() {}

  Index(int v, int vt, int vn) : position(v), uv(vt), normal(vn) {}

  bool operator<(const Index& i) const {
    if (position < i.position) return true;
    if (position > i.position) return false;
    if (uv < i.uv) return true;
    if (uv > i.uv) return false;
    if (normal < i.normal) return true;
    if (normal > i.normal) return false;

    return false;
  }

  int position;
  int uv;
  int normal;
};

Index parseFaceIndex(const std::string& token) {
  std::stringstream in(token);
  std::string indexString;
  int indices[3] = {1, 1, 1};

  int i = 0;
  while (std::getline(in, indexString, '/')) {
    if (indexString != "\\") {
      std::stringstream ss(indexString);
      ss >> indices[i++];
    }
  }

  // decrement since indices in OBJ files are 1-based
  return Index(indices[0] - 1, indices[1] - 1, indices[2] - 1);
}

// Read a .obj file containing a polygon mesh
void PolygonSoupMesh::readMeshFromFile(std::string filename, bool loadTexture) {
  // std::cout << "Reading mesh from file: " << filename << std::endl;

  polygons.clear();
  vertexCoordinates.clear();

  // Open the file
  std::ifstream in(filename);
  if (!in) throw std::invalid_argument("Could not open mesh file " + filename);

  // parse obj format
  std::string line;
  std::vector<Vector2> textureCoordinateList;
  while (getline(in, line)) {
    std::stringstream ss(line);
    std::string token;

    ss >> token;

    if (token == "v") {
      double x, y, z;
      ss >> x >> y >> z;

      vertexCoordinates.push_back(Vector3{x, y, z});

    } else if (token == "vt") {
      if (loadTexture) {
        double x, y;
        ss >> x >> y;

        textureCoordinateList.push_back(Vector2{x, y});
      }

    } else if (token == "vn") {
      // Do nothing

    } else if (token == "f") {
      std::vector<size_t> face;
      std::vector<Vector2> textureFace;
      while (ss >> token) {
        Index index = parseFaceIndex(token);
        if (index.position < 0) {
          getline(in, line);
          size_t i = line.find_first_not_of("\t\n\v\f\r ");
          index = parseFaceIndex(line.substr(i));
        }

        face.push_back(index.position);
        if (loadTexture) textureFace.push_back(textureCoordinateList[index.uv]);
      }

      polygons.push_back(face);
      if (loadTexture) textureCoordinates.push_back(textureFace);
    }
  }
}

// Assumes that first line has already been consumed
void PolygonSoupMesh::readMeshFromAsciiStlFile(std::ifstream& in) {
  std::string line;
  std::stringstream ss;
  size_t lineNum = 1;

  auto assertEq = [&](std::string expected, std::string found) {
    if (found != expected) {
      std::cerr << "Error on line " << lineNum << ". Expected \"" << expected << "\" but found \"" << found << "\""
                << std::endl;
      std::cerr << "Full line: \"" << line << "\"" << std::endl;
      exit(1);
    }
  };

  auto nextLine = [&]() {
    if (!getline(in, line)) {
      return false;
    }
    ss = std::stringstream(line);
    lineNum++;
    return true;
  };

  // Parse STL file
  while (nextLine()) {

    std::string token;

    ss >> token;
    if (token == "endsolid") {
      break;
    }
    assertEq("facet", token);

    ss >> token;
    assertEq("normal", token);

    // TODO: store this normal?
    // TODO: orient face according to normal
    double nX = 0, nY = 0, nZ = 0;
    ss >> nX >> nY >> nZ;
    Vector3 normal{nX, nY, nZ};

    nextLine();

    assertEq("outer loop", line);
    std::vector<size_t> face;

    while (nextLine() && line != "endloop") {
      ss >> token;
      assertEq("vertex", token);

      double vX = 0, vY = 0, vZ = 0;
      ss >> vX >> vY >> vZ;
      vertexCoordinates.push_back(Vector3{vX, vY, vZ});

      face.push_back(vertexCoordinates.size() - 1);
    }

    nextLine();
    assertEq("endfacet", line);

    // Orient face using normal
    Vector3 faceNormal = cross(vertexCoordinates[face[1]] - vertexCoordinates[face[0]],
                               vertexCoordinates[face[2]] - vertexCoordinates[face[0]]);
    if (dot(faceNormal, normal) < 0) {
      std::reverse(std::begin(face), std::end(face));
    }

    polygons.push_back(face);
  }
}

void PolygonSoupMesh::readMeshFromBinaryStlFile(std::ifstream in) {
  auto parseVector3 = [&](std::ifstream& in) {
    char buffer[3 * sizeof(float)];
    in.read(buffer, 3 * sizeof(float));
    float* fVec = (float*)buffer;
    return Vector3{fVec[0], fVec[1], fVec[2]};
  };

  char header[80];
  char nTriangleChars[4];
  in.read(header, 80);
  in.read(nTriangleChars, 4);
  unsigned int* intPtr = (unsigned int*)nTriangleChars;
  size_t nTriangles = *intPtr;

  for (size_t iT = 0; iT < nTriangles; ++iT) {
    // TODO: store this normal?
    Vector3 normal = parseVector3(in);
    std::vector<size_t> face;
    for (size_t iV = 0; iV < 3; ++iV) {
      vertexCoordinates.push_back(parseVector3(in));
      face.push_back(vertexCoordinates.size() - 1);
    }

    // Orient face using normal
    Vector3 faceNormal = cross(vertexCoordinates[face[1]] - vertexCoordinates[face[0]],
                               vertexCoordinates[face[2]] - vertexCoordinates[face[0]]);
    if (dot(faceNormal, normal) < 0) {
      std::reverse(std::begin(face), std::end(face));
    }

    polygons.push_back(face);
    char dummy[2];
    in.read(dummy, 2);
  }
}

void PolygonSoupMesh::readMeshFromStlFile(std::string filename) {
  // TODO: read stl file name
  polygons.clear();
  vertexCoordinates.clear();

  // Open the file
  std::ifstream in(filename);
  if (!in) throw std::invalid_argument("Could not open mesh file " + filename);

  // parse stl format
  std::string line;
  std::stringstream ss;
  getline(in, line);
  std::string token;
  ss >> token;
  if (token == "solid") {
    readMeshFromAsciiStlFile(in);
  } else {
    readMeshFromBinaryStlFile(std::ifstream(filename, std::ios::in | std::ios::binary));
  }
}

void PolygonSoupMesh::mergeByDistance(double tol) {

  size_t nV = vertexCoordinates.size();

  std::vector<std::pair<Vector3, size_t>> indexedPositions;
  indexedPositions.reserve(nV);
  for (size_t iV = 0; iV < nV; ++iV) {
    indexedPositions.push_back(std::make_pair(vertexCoordinates[iV], iV));
  }
  vertexCoordinates.clear();

  // Sort the vertices in lexicographic order by position
  // This ensures that all vertices in the same position will be next to each other in the list
  std::sort(std::begin(indexedPositions), std::end(indexedPositions),
            [](const std::pair<Vector3, size_t>& a, const std::pair<Vector3, size_t> b) {
              const Vector3& vA = std::get<0>(a);
              const Vector3& vB = std::get<0>(b);
              if (vA.x < vB.x) {
                return true;
              } else if (vA.x == vB.x && vA.y < vB.y) {
                return true;
              } else if (vA.x == vB.x && vA.y == vB.y && vA.z < vB.z) {
                return true;
              } else {
                return false;
              }
            });

  // Store mapping from original vertex index to merged vertex index
  std::vector<size_t> compressVertex;
  compressVertex.resize(nV);

  // Merge vertices together
  // We just iterate down the sorted list and collect all vertices which
  // are within tol of the current vertex
  size_t iV = 0;
  do {
    Vector3 pos = std::get<0>(indexedPositions[iV]);
    vertexCoordinates.push_back(pos);

    Vector3 posV;
    do {
      size_t oldIndex = std::get<1>(indexedPositions[iV]);
      compressVertex[oldIndex] = vertexCoordinates.size() - 1;

      iV++;
      posV = std::get<0>(indexedPositions[iV]);
    } while ((pos - posV).norm() < tol);
  } while (iV < nV);

  // Update face indices
  for (std::vector<size_t>& face : polygons) {
    for (size_t& iV : face) {
      iV = compressVertex[iV];
    }
  }
}

void PolygonSoupMesh::triangulate() {
  std::vector<std::vector<size_t>> newPolygons;

  for (auto poly : polygons) {
    if (poly.size() <= 2) {
      throw std::runtime_error("ERROR: PolygonSoupMesh has degree < 3 polygon");
    }

    for (size_t i = 2; i < poly.size(); i++) {
      std::vector<size_t> tri = {poly[0], poly[i - 1], poly[i]};
      newPolygons.push_back(tri);
    }
  }

  polygons = newPolygons;
}

} // namespace geometrycentral
