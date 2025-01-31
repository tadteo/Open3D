#include "open3d/geometry/CryoEMOctree.h"
#include "pybind/docstring.h"
#include "pybind/geometry/geometry_trampoline.h"

namespace open3d {
namespace geometry {

namespace py = pybind11;

void pybind_cryoem_octree(py::module &m) {
    // Bind the leaf node
    py::class_<CryoEMOctreeLeafNode,
               std::shared_ptr<CryoEMOctreeLeafNode>,
               OctreeLeafNode>(m, "CryoEMOctreeLeafNode")
        .def(py::init<>())
        .def_readwrite("density", &CryoEMOctreeLeafNode::density_)
        .def_readwrite("resolution", &CryoEMOctreeLeafNode::resolution_)
        .def("__repr__", [](const CryoEMOctreeLeafNode &n) {
            return std::string("CryoEMOctreeLeafNode(density=") +
                   std::to_string(n.density_) + ", resolution=" +
                   std::to_string(n.resolution_) + ")";
        });

    // Bind the internal node
    py::class_<CryoEMOctreeInternalNode,
               std::shared_ptr<CryoEMOctreeInternalNode>,
               OctreeInternalNode>(m, "CryoEMOctreeInternalNode")
        .def(py::init<>())
        .def_readwrite("density", &CryoEMOctreeInternalNode::density_)
        .def_readwrite("resolution", &CryoEMOctreeInternalNode::resolution_)
        .def("aggregate_children",
             &CryoEMOctreeInternalNode::AggregateChildren);

    // Bind the main octree class
    py::class_<CryoEMOctree,
               std::shared_ptr<CryoEMOctree>,
               Octree>(m, "CryoEMOctree")
        .def(py::init<size_t, const Eigen::Vector3d&, double>(),
             "max_depth"_a, "origin"_a = Eigen::Vector3d(0,0,0),
             "size"_a = 1.0)
        .def("insert_density_point",
             &CryoEMOctree::InsertDensityPoint,
             "Insert a point with density/resolution.",
             "point"_a, "density"_a, "resolution"_a)
        .def("aggregate_all_nodes",
             &CryoEMOctree::AggregateAllNodes,
             "Aggregate child data up the tree.")
        .def("__repr__", [](const CryoEMOctree &oct) {
            return std::string("CryoEMOctree with max_depth=") +
                   std::to_string(oct.max_depth_) +
                   ", origin=[" + std::to_string(oct.origin_(0)) + ", " +
                   std::to_string(oct.origin_(1)) + ", " +
                   std::to_string(oct.origin_(2)) + "], size=" +
                   std::to_string(oct.size_);
        });
}

}  // namespace geometry
}  // namespace open3d
