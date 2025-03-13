# Ensure Pybind11 is found
find_package(pybind11 CONFIG REQUIRED)

# Include Pybind11 headers
include_directories(${pybind11_INCLUDE_DIRS})

open3d_ispc_add_library(geometry OBJECT)

# Link Pybind11 to the geometry library
target_link_libraries(geometry PRIVATE pybind11::module)

target_sources(geometry PRIVATE
    BoundingVolume.cpp
    CryoEMOctree.cpp
    EstimateNormals.cpp
    Geometry3D.cpp
    HalfEdgeTriangleMesh.cpp
    Image.cpp
    ImageFactory.cpp
    IntersectionTest.cpp
    ISSKeypoints.cpp
    KDTreeFlann.cpp
    Line3D.cpp
    LineSet.cpp
    LineSetFactory.cpp
    MeshBase.cpp
    Octree.cpp
    PointCloud.cpp
    PointCloudCluster.cpp
    PointCloudFactory.cpp
    PointCloudSegmentation.cpp
    PointCloudPlanarPatchDetection.cpp
    Qhull.cpp
    RGBDImage.cpp
    RGBDImageFactory.cpp
    SurfaceReconstructionAlphaShape.cpp
    SurfaceReconstructionBallPivoting.cpp
    SurfaceReconstructionPoisson.cpp
    TetraMesh.cpp
    TetraMeshFactory.cpp
    TriangleMesh.cpp
    TriangleMeshDeformation.cpp
    TriangleMeshFactory.cpp
    TriangleMeshSimplification.cpp
    TriangleMeshSubdivide.cpp
    VoxelGrid.cpp
    VoxelGridFactory.cpp
)

open3d_show_and_abort_on_warning(geometry)
open3d_set_global_properties(geometry)
open3d_set_open3d_lib_properties(geometry)
open3d_link_3rdparty_libraries(geometry)
