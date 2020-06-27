#!/usr/local/bin/python3

import os
import re
import subprocess
import time

# stolen from
# https://svn.blender.org/svnroot/bf-blender/trunk/blender/build_files/scons/tools/bcolors.py
# via
# https://stackoverflow.com/questions/287871/how-to-print-colored-text-in-python
class pcolors:
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

usefulIncludes = {
    'Vector2' : '#include "geometrycentral/utilities/vector2.h;',
    'Vector3' : '#include "geometrycentral/utilities/vector3.h;',
    'HalfedgeMesh' : '#include "geometrycentral/surface/halfedge_mesh.h;',
    'VertexPositionGeometry' : '#include "geometrycentral/surface/vertex_position_geometry.h;'
}

defaultVariableDeclarations = {
    re.compile('Vector2 (?P<name>.*?) =\s*/\*[^\*]*\*/') : (lambda match : f'Vector2 {match.group("name")}{{1, 0}};'),
    re.compile('Vector3 (?P<name>.*?) =\s*/\*[^\*]*\*/') : (lambda match : f'Vector3 {match.group("name")}{{1, 0, 0}};'),
    re.compile('Vertex (?P<name>.*?) =\s*/\*[^\*]*\*/') : (lambda match : f'Vertex {match.group("name")} = defaultMeshPtr->vertex(0);'),
    re.compile('Halfedge (?P<name>.*?) =\s*/\*[^\*]*\*/') : (lambda match : f'Halfedge {match.group("name")} = defaultMeshPtr->halfedge(0);'),
    re.compile('Face (?P<name>.*?) =\s*/\*[^\*]*\*/') : (lambda match : f'Face {match.group("name")} = defaultMeshPtr->face(0);'),
    re.compile('VertexPositionGeometry& (?P<name>.*?) =\s*/\*[^\*]*\*/') : (lambda match : f'VertexPositionGeometry& {match.group("name")} = *defaultGeomPtr;'),
    re.compile('HalfedgeMesh& (?P<name>.*?) =\s*/\*[^\*]*\*/') : (lambda match : f'HalfedgeMesh& {match.group("name")} = *defaultMeshPtr;'),
    re.compile('VertexData<(?P<T>.*?)> (?P<name>.*?) =\s*/\*[^\*]*\*/') : (lambda match : f'VertexData<{match.group("T")}> {match.group("name")}(*defaultMeshPtr);'),
    re.compile('FaceData<(?P<T>.*?)> (?P<name>.*?) =\s*/\*[^\*]*\*/') : (lambda match : f'FaceData<{match.group("T")}> {match.group("name")}(*defaultMeshPtr);'),
}

miscInclude = """
#include "geometrycentral/utilities/vector2.h"
#include "geometrycentral/utilities/vector3.h"
#include "geometrycentral/surface/halfedge_mesh.h"
#include "geometrycentral/surface/meshio.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

std::string filename = "spot.obj";

"""

miscCode = """
std::unique_ptr<HalfedgeMesh> defaultMeshPtr;
std::unique_ptr<VertexPositionGeometry> defaultGeomPtr;
std::tie(defaultMeshPtr, defaultGeomPtr) = loadMesh(filename);

"""

cmakeHeader = """
cmake_minimum_required(VERSION 3.8.0)

# Maybe stop from CMAKEing in the wrong place
if (CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
  message(FATAL_ERROR "Source and build directories cannot be the same. Go use the /build directory.")
endif()
"""

# Match from ```cpp to ```. The ? modifies .* to search for a _minimal_ match,
# IE only up to the next ```
codeBlockMatcher = re.compile('(?:<!---[^`]*-->)?\n[^`]*```cpp[^`]*```', re.MULTILINE)

# Match the contents of an HTML comment. Store the contents in a group named 'conents'
setupMatcher = re.compile('<!-*(?P<contents>[^`]*)-->', re.MULTILINE)

# Match the contents of a codeblock of the form ```cpp ... ```.
# Store the contents in a group named 'conents'
codeMatcher = re.compile('``cpp(?P<contents>[^`]*)```', re.MULTILINE)

# Match if input contains the string #include or using. Any line that doesn't match
# this is stuck inside a main function
includeMatcher = re.compile('(#include|using)')

def extractCpp(contents):
    matches = []
    for match in codeBlockMatcher.findall(contents):
        setup = setupMatcher.search(match)
        code = codeMatcher.search(match)
        if setup:
            if "NO TEST" not in setup.group('contents'):
                matches.append(setup.group('contents') + '\n' + code.group('contents'))
        else:
            matches.append(code.group('contents'))
    return matches

def parseCodeLine(line):
    for pattern, replacement in defaultVariableDeclarations.items():
        match = pattern.search(line)
        if (match):
            return replacement(match)
    return line

def findIncludes(codeblock):
    codeLines = codeblock.split('\n')
    includes = ''
    code = ''
    defaultVariablesUsed = set()
    for line in codeLines:
        if includeMatcher.match(line):
            includes += line + '\n'
        elif line and not line.isspace():
            code += parseCodeLine(line) + '\n'

    return [includes, code]

def makeMain(includes, code):
    return includes + miscInclude + '\nint main(int argc, char **argv) {\n' + miscCode + code + '\nreturn 0;\n}\n'

def cmakeEntry(progName):
    return ( f'add_executable(\n'
             f'{progName}\n'
             f'{progName}.cpp\n'
             f')\n'
             f'target_link_libraries({progName} geometry-central)\n'
            )


# ===========================================================================
#
#                            Parse Docs
#
# ===========================================================================

# save original directory
path = os.getcwd()


CMakeLists = cmakeHeader
executables = {}
nExecutables = 0

skip = ['linear_solvers', 'linear_algebra_utilities', 'vector_heat_method', 'io']

if not os.path.isdir(path + '/test/src'):
    os.mkdir(path + '/test/src')
for root, dirs, files in os.walk(path + '/docs/'):
    for name in files:
        if name[-3:] == '.md':
            with open(root + '/' + name) as f:
                contents = f.read()
            name = name[:-3]
            if name in skip:
                continue;
            executables[name] = []
            idx = 0
            for codeblock in extractCpp(contents):
                includes, code = findIncludes(codeblock)

                fullCode = '// ' + root + '/' + name + '\n' + makeMain(includes, code)

                fullName = name + '_' + str(idx);

                CMakeLists += cmakeEntry(fullName) + "\n"

                executables[name].append(fullName)
                nExecutables += 1

                with open(path + '/test/src/' + fullName + '.cpp', 'w') as f:
                    f.write(fullCode)

                idx += 1

with open(path + '/test/src/CMakeLists.txt', 'w') as f:
    f.write(CMakeLists)

# ===========================================================================
#
#                             Build Code
#
# ===========================================================================

if not os.path.isdir(path + '/test/build'):
    os.mkdir(path + '/test/build')

os.chdir(path + '/test/build')
os.system('cmake ..')
os.system('make -j')


# ===========================================================================
#
#                             Run Code
#
# ===========================================================================

totalTime = 0
failures = []

print(pcolors.OKGREEN + '[==========] ' + pcolors.ENDC + ' Running ' + str(nExecutables) + ' snippets from ' + str(len(executables)) + ' sections')

for section, progs in executables.items():
    print(pcolors.OKGREEN + '[----------] ' + pcolors.ENDC + str(len(progs)) + ' snippets from ' + section)
    sectionTime = 0
    for prog in progs:
        print(pcolors.OKGREEN + '[ RUN      ] ' + pcolors.ENDC + prog)
        if os.path.isfile('bin/' + prog):
            start = time.time()
            try:
                subprocess.check_call('bin/' + prog)
            except subprocess.CalledProcessError as e:
                print("\t" + str(e))
                print(pcolors.FAIL + "[  FAILED  ] " + pcolors.ENDC, end='')
                failures.append(prog)
            else:
                print(pcolors.OKGREEN + "[       OK ] " + pcolors.ENDC, end='')
            end = time.time()
            elapsed = int(1000 * (end - start))
            sectionTime += elapsed
            print(prog + " (" + str(elapsed) + " ms)")
        else:
            print(pcolors.FAIL + "[  FAILED  ] " + pcolors.ENDC + prog + ' compilation failed')
            failures.append(prog)
    print(pcolors.OKGREEN + '[----------] ' + pcolors.ENDC + str(len(progs)) + ' snippets from ' + section + ' (' + str(sectionTime) + ' ms)\n')
    totalTime += sectionTime

print(pcolors.OKGREEN + '[==========] ' + pcolors.ENDC + str(nExecutables) + ' snippets (' + str(totalTime) + ' ms)')
print(pcolors.OKGREEN + '[  PASSED  ] ' + pcolors.ENDC + str(nExecutables - len(failures)) + ' snippets')
print(pcolors.FAIL + '[  FAILED  ] ' + pcolors.ENDC + str(len(failures)) + ' snippets, listed below')

for prog in failures:
    print(pcolors.FAIL + '[  FAILED  ] ' + pcolors.ENDC + prog)

print("")

# move back to original directory
os.chdir(path)

for name in skip:
    print('skipped ' + name)
