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

// STL files are just a triangle soup, so we need a tolerance to decide when to merge vertices together
void PolygonSoupMesh::readMeshFromStlFile(std::string filename) {
  polygons.clear();
  vertexCoordinates.clear();

  // Open the file
  std::ifstream in(filename);
  if (!in) throw std::invalid_argument("Could not open mesh file " + filename);

  // parse stl format
  std::string line;
  std::vector<Vector2> textureCoordinateList;

  getline(in, line);
  if (line.rfind("solid", 0) != 0) {
    throw std::runtime_error("STL parser for binary files not implemented yet");
  }
  // TODO: read stl file name
  size_t lineNum = 1;

  auto assertEq = [&](std::string expected, std::string found) {
    if (found != expected) {
      std::cerr << "Error on line " << lineNum << ". Expected \"" << expected << "\" but found \"" << found << "\""
                << std::endl;
      std::cerr << "Full line: \"" << line << "\"" << std::endl;
      exit(1);
    }
  };

  // Parse STL file
  while (getline(in, line)) {
    lineNum++;

    std::stringstream ss(line);
    std::string token;

    ss >> token;
    assertEq("facet", token);

    ss >> token;
    assertEq("normal", token);

    // TODO: store this normal?
    // TODO: orient face according to normal
    double nX, nY, nZ;
    ss >> nX, nY, nZ;
    Vector3 normal{nX, nY, nZ};

    getline(in, line);
    lineNum++;

    assertEq("outer loop", line);
    std::vector<size_t> face;
    do {
      getline(in, line);
      lineNum++;
      ss >> token;
      assertEq("vertex", token);

      double vX, vY, vZ;
      ss >> vX, vY, vZ;
      vertexCoordinates.push_back(Vector3{vX, vY, vZ});
      face.push_back(vertexCoordinates.size());
    } while (line != "endloop");
    polygons.push_back(face);
  }
}

void PolygonSoupMesh::mergeByDistance(double tol) {
  std::vector<Vector3> rawVertexPositions = std::move(vertexCoordinates);
  vertexCoordinates.clear();

  // dedupe vertices so that we can build a mesh later if we want to
  std::vector<size_t> compressVertex;
  compressVertex.reserve(rawVertexPositions.size());

  // TODO: make this not take n^2 time
  for (size_t iV = 0; iV < rawVertexPositions.size(); ++iV) {
    Vector3 pos = rawVertexPositions[iV];

    // Find the first vertex at the same position as iV.
    // We're guaranteed to find something since eventually iW = iV
    for (size_t iW = 0; iW <= iV; ++iW) {
      Vector3 posW = rawVertexPositions[iW];
      if ((pos - posW).norm() < tol) {
        if (iV == iW) {
          vertexCoordinates.push_back(pos);
          compressVertex[iV] = vertexCoordinates.size();
        } else {
          compressVertex[iV] = compressVertex[iW];
        }
        break;
      }
    }
  }

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
