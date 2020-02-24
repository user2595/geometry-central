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
  auto parseFloat = [](std::ifstream& in) {
    char buffer[sizeof(float)];
    in.read(buffer, sizeof(float));
    float* floatPointer = (float*)buffer;
    return *floatPointer;
  };
  auto parseVector3 = [&](std::ifstream& in) {
    double vX = parseFloat(in);
    double vY = parseFloat(in);
    double vZ = parseFloat(in);
    return Vector3{vX, vY, vZ};
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

  std::vector<Vector3> rawVertexPositions = std::move(vertexCoordinates);
  vertexCoordinates.clear();

  // dedupe vertices so that we can build a mesh later if we want to
  std::vector<size_t> compressVertex;
  compressVertex.resize(rawVertexPositions.size());

  if (nV < 1e6) {
    std::vector<char> visited;
    visited.reserve(nV);
    for (size_t iV = 0; iV < nV; ++iV) visited[iV] = false;

    for (size_t iV = 0; iV < rawVertexPositions.size(); ++iV) {
      if (visited[iV]) {
        continue;
      } else {
        Vector3 pos = rawVertexPositions[iV];
        vertexCoordinates.push_back(pos);
        compressVertex[iV] = vertexCoordinates.size() - 1;

        // Mark all future vertices at this position as visited
        for (size_t iW = iV + 1; iW < nV; ++iW) {
          Vector3 posW = rawVertexPositions[iW];
          if ((pos - posW).norm() < tol) {
            compressVertex[iW] = compressVertex[iV];
            visited[iW] = true;
          }
        }
      }
      if (iV % (size_t)1e2 == 0) {
        std::cout << "\r" << iV << " / " << nV << " " << ((double)iV) / ((double)nV) * 100.0 << "%\t\t" << std::flush;
      }
    }
    std::cout << std::endl;
  } else {

    // TODO: make this not take n^2 time
    // Smarter spatial data structure?

    double minX = rawVertexPositions[0].x;
    double maxX = rawVertexPositions[0].x;
    double minY = rawVertexPositions[0].y;
    double maxY = rawVertexPositions[0].y;
    double minZ = rawVertexPositions[0].z;
    double maxZ = rawVertexPositions[0].z;

    bool* visited = new bool[nV];
    for (size_t iV = 0; iV < nV; ++iV) {
      visited[iV] = false;
      minX = fmin(minX, rawVertexPositions[iV].x);
      minY = fmin(minY, rawVertexPositions[iV].y);
      minZ = fmin(minZ, rawVertexPositions[iV].z);
      maxX = fmax(maxX, rawVertexPositions[iV].x);
      maxY = fmax(maxY, rawVertexPositions[iV].y);
      maxZ = fmax(maxZ, rawVertexPositions[iV].z);
    }

    size_t gridSize = std::cbrt((double)nV) / 8;
    size_t nBuckets = gridSize * gridSize * gridSize;
    std::vector<std::vector<size_t>> vertexBuckets;
    vertexBuckets.reserve(nBuckets);
    for (size_t iB = 0; iB < nBuckets; ++iB) {
      vertexBuckets.push_back(std::vector<size_t>());
      vertexBuckets[iB].reserve(((double)nV) / ((double)nBuckets));
    }

    auto getBucketCoords = [&](Vector3 pos) {
      // Multiply by 0.99 so that the answer is strictly less than gridSize
      int xIndex = (int)((pos.x - minX) / (maxX - minX) * gridSize * 0.99);
      int yIndex = (int)((pos.y - minY) / (maxY - minY) * gridSize * 0.99);
      int zIndex = (int)((pos.z - minZ) / (maxZ - minZ) * gridSize * 0.99);
      return std::array<int, 3>{xIndex, yIndex, zIndex};
    };

    auto getBucketIndex = [&](std::array<int, 3> bucketCoords) {
      return bucketCoords[0] + gridSize * bucketCoords[1] + gridSize * gridSize * bucketCoords[2];
    };

    for (size_t iV = 0; iV < nV; ++iV) {
      Vector3 pos = rawVertexPositions[iV];
      size_t vBucketIndex = getBucketIndex(getBucketCoords(pos));

      if (vBucketIndex >= nBuckets) {
        std::cerr << "Error: bucket index " << vBucketIndex << " bigger than maximum " << nBuckets << std::endl;
        exit(1);
      }
      vertexBuckets[vBucketIndex].push_back(iV);
    }

    for (size_t iV = 0; iV < nV; ++iV) {
      if (visited[iV]) {
        continue;
      } else {
        Vector3 pos = rawVertexPositions[iV];
        vertexCoordinates.push_back(pos);
        compressVertex[iV] = vertexCoordinates.size() - 1;

        // Check neighboring buckets for nearby vertices
        std::array<int, 3> bucketCoords = getBucketCoords(pos);
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
              std::array<int, 3> neighborBucketCoords = bucketCoords;
              neighborBucketCoords[0] += dx;
              neighborBucketCoords[1] += dy;
              neighborBucketCoords[2] += dz;
              if (neighborBucketCoords[0] >= 0 && neighborBucketCoords[0] < gridSize && neighborBucketCoords[1] >= 0 &&
                  neighborBucketCoords[1] < gridSize && neighborBucketCoords[2] >= 0 &&
                  neighborBucketCoords[2] < gridSize) {

                for (size_t iW : vertexBuckets[getBucketIndex(neighborBucketCoords)]) {
                  Vector3 posW = rawVertexPositions[iW];
                  if ((pos - posW).norm() < tol) {

                    compressVertex[iW] = compressVertex[iV];
                    visited[iW] = true;
                  }
                }
              }
            }
          }
        }
      }

      if (iV % (size_t)1e3 == 0) {
        std::cout << "\r" << iV << " / " << nV << " " << ((double)iV) / ((double)nV) * 100.0 << "%\t\t" << std::flush;
      }
    }
    delete[] visited;
  }


  // Update face indices
  for (std::vector<size_t>& face : polygons) {
    for (size_t iFV = 0; iFV < face.size(); ++iFV) {
      face[iFV] = compressVertex[face[iFV]];
    }
  }
}


//   // Update face indices
//   for (std::vector<size_t>& face : polygons) {
//     for (size_t& iV : face) {
//       iV = compressVertex[iV];
//     }
//   }
// }

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
