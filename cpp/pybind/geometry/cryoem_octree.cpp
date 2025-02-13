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
        .def("__repr__", [](const CryoEMOctreeLeafNode &n) {
            return std::string("CryoEMOctreeLeafNode(density=") +
                   std::to_string(n.density_) + ")";
        });

    // Bind the internal node
    py::class_<CryoEMOctreeInternalNode,
               std::shared_ptr<CryoEMOctreeInternalNode>,
               OctreeInternalNode>(m, "CryoEMOctreeInternalNode")
        .def(py::init<>())
        .def_readwrite("density", &CryoEMOctreeInternalNode::density_)
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
             "Insert a point with density.",
             "point"_a, "density"_a)
        .def("compress_node", &CryoEMOctree::CompressNode,
          "Compress an internal Cryo-EM node in place if eligible. "
          "The function updates the node pointer with a merged leaf node if compression applies.",
          py::arg("node"))
        .def("compress_octree", [](CryoEMOctree &octree, float tolerance) {
            int merge_count = 0;
            float avg_error = 0.0f;
            octree.CompressOctree(tolerance, merge_count, avg_error);
            // Return a tuple: (merge_count, avg_error)
            return py::make_tuple(merge_count, avg_error);
        }, "Compress the octree given a tolerance.")
        .def("compress_octree_adaptive", [](CryoEMOctree &octree, float base_tolerance) {
            int merge_count = 0;
            float avg_error = 0.0f;
            octree.CompressOctreeAdaptive(base_tolerance, merge_count, avg_error);
            // Return a tuple: (merge_count, avg_error)
            return py::make_tuple(merge_count, avg_error);
        }, "Compress the octree given a base tolerance.")
        .def("count_nodes", [](const CryoEMOctree &octree) {
            return octree.CountNodes();
        }, "Count the total number of nodes in the octree.")
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
