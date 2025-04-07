#include "open3d/geometry/CryoEMOctree.h"
#include "open3d/geometry/VoxelGrid.h"
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>
#include <cmath>           // for std::fabs
#include <json/json.h>     // For JSON handling (if needed)
#include "open3d/utility/Logging.h"
#include <numeric>  // for std::accumulate
#include <limits>   // for std::numeric_limits
#include <chrono>   // for std::chrono
#include <queue>
#include <unordered_map>
#include <sstream>
#include <iomanip>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

/*
 * This file contains the implementation of the CryoEMOctree and its associated nodes.
 */
namespace py = pybind11;
namespace open3d {
namespace geometry {

/**
 * @brief Attempts to compress an internal Cryo‑EM node in place, if all its immediate children are leaf nodes
 * or null. The compression aggregates the density values using a simple arithmetic mean,
 * and replaces the node with a new leaf node if eligible.
 *
 * This function modifies the tree in place: when called with a reference to a node pointer in the tree, if the node
 * qualifies for compression, its pointer is updated to point to a new CryoEMOctreeLeafNode with the aggregated value.
 *
 * @param node A reference to the node pointer to possibly compress.
 */
void CryoEMOctree::CompressNode(std::shared_ptr<OctreeNode>& node) {
    if (!node) {
        utility::LogInfo("Node is null");
        return;
    }

    // Check if the node is an internal Cryo‑EM node.
    auto internal = std::dynamic_pointer_cast<CryoEMOctreeInternalNode>(node);
    if (!internal) {
        utility::LogInfo("Node is not an internal node");
        return;
    }

    // Check that every non-null child is a Cryo‑EM leaf node.
    bool eligible = true;
    std::vector<std::shared_ptr<CryoEMOctreeLeafNode>> leaves;
    for (auto& child : internal->children_) {
        if (child) {
            auto leaf = std::dynamic_pointer_cast<CryoEMOctreeLeafNode>(child);
            if (!leaf) {
                eligible = false;
                break;
            }
            leaves.push_back(leaf);
        }
    }
    if (!eligible || leaves.empty()) {
        utility::LogInfo("Node is not eligible for compression");
        return;
    }
    // Aggregate the leaf data using arithmetic mean.
    float sum_density = 0.0f;
    for (const auto& leaf : leaves) {
        sum_density += leaf->density_;
    }
    
    float new_density = sum_density / static_cast<float>(leaves.size());
    
    // Construct a new leaf node representing the compressed branch.
    auto merged_leaf = std::make_shared<CryoEMOctreeLeafNode>();
    merged_leaf->density_ = new_density;
    
    // Update the node in place by replacing it with the merged leaf.
    node = merged_leaf;
}

//==============================================================================
// Implementation for CryoEMOctreeLeafNode
//==============================================================================

CryoEMOctreeLeafNode::CryoEMOctreeLeafNode() 
    : density_(0.0f) {}

std::shared_ptr<OctreeNode> CryoEMOctreeLeafNode::Clone() const {
    auto node = std::make_shared<CryoEMOctreeLeafNode>();
    node->density_ = density_;
    return node;
}

bool CryoEMOctreeLeafNode::ConvertToJsonValue(Json::Value &value) const {
    // Serialize the Cryo‑EM specific data.
    value["density"] = density_;
    return true;
}

bool CryoEMOctreeLeafNode::ConvertFromJsonValue(const Json::Value &value) {
    if (!value.isMember("density"))
        return false;
        
    density_ = value["density"].asFloat();
    return true;
}

bool CryoEMOctreeLeafNode::operator==(const OctreeLeafNode& other) const {
    const CryoEMOctreeLeafNode* other_leaf = dynamic_cast<const CryoEMOctreeLeafNode*>(&other);
    if (!other_leaf) return false;
    return (std::fabs(density_ - other_leaf->density_) < 1e-6f);
}

std::function<std::shared_ptr<OctreeLeafNode>()>
CryoEMOctreeLeafNode::GetInitFunction() {
    return []() -> std::shared_ptr<OctreeLeafNode> {
        return std::make_shared<CryoEMOctreeLeafNode>();
    };
}

std::function<void(std::shared_ptr<OctreeLeafNode>)>
CryoEMOctreeLeafNode::GetUpdateFunction(float density) {
    return [density](std::shared_ptr<OctreeLeafNode> node) -> void {
        if (auto leaf = std::dynamic_pointer_cast<CryoEMOctreeLeafNode>(node)) {
            leaf->density_ = density;  // Assuming density_ is a member of CryoEMOctreeLeafNode
        } else {
            utility::LogError("Internal error: leaf node must be CryoEMOctreeLeafNode");
        }
    };
}

//==============================================================================
// Implementation for CryoEMOctreeInternalNode
//==============================================================================

void CryoEMOctreeInternalNode::AggregateChildren() {
    double sum_density = 0.0;
    int count = 0;
    // Aggregate density from each child.
    for (auto &child : children_) {
        if (child) {
            auto leaf = std::dynamic_pointer_cast<CryoEMOctreeLeafNode>(child);
            if (leaf) {
                sum_density += leaf->density_;
                count++;
                continue;
            }
            auto inode = std::dynamic_pointer_cast<CryoEMOctreeInternalNode>(child);
            if (inode) {
                sum_density += inode->density_;
                count++;
            }
        }
    }
    if (count > 0) {
        density_ = static_cast<float>(sum_density / count);
    }
}

std::shared_ptr<OctreeNode> CryoEMOctreeInternalNode::Clone() const {
    auto node = std::make_shared<CryoEMOctreeInternalNode>();
    // Copy relevant data
    node->density_ = density_;
    
    // Clone children
    for (size_t i = 0; i < children_.size(); ++i) {
        if (children_[i]) {
            node->children_[i] = children_[i]->Clone();
        }
    }
    return node;
}

std::function<std::shared_ptr<OctreeInternalNode>()>
CryoEMOctreeInternalNode::GetInitFunction() {
    return []() -> std::shared_ptr<OctreeInternalNode> {
        return std::make_shared<CryoEMOctreeInternalNode>();
    };
}

std::function<void(std::shared_ptr<OctreeInternalNode>)>
CryoEMOctreeInternalNode::GetUpdateFunction(float density) {
    return [density](std::shared_ptr<OctreeInternalNode> node) -> void {
        if (auto internal = std::dynamic_pointer_cast<CryoEMOctreeInternalNode>(node)) {
            // Update internal node with the density or other information if needed
            internal->density_ = density;  // Assuming density_ exists in the internal node
            // If you need to aggregate children data, do it here
        } else {
            utility::LogError("Internal error: internal node must be CryoEMOctreeInternalNode");
        }
    };
}

//==============================================================================
// Implementation for CryoEMOctree
//==============================================================================

CryoEMOctree::CryoEMOctree(int max_depth, const Eigen::Vector3d &origin, double size)
    : Octree(max_depth, origin, size) {
    root_node_ = std::make_shared<CryoEMOctreeInternalNode>();
}

void CryoEMOctree::InsertDensityPoint(const Eigen::Vector3d &point, float density) {
    // Get the initialization and update functions for CryoEM-specific nodes
    auto cryoLeafInit = CryoEMOctreeLeafNode::GetInitFunction();
    auto cryoLeafUpdate = CryoEMOctreeLeafNode::GetUpdateFunction(density);
    auto cryoInternalInit = CryoEMOctreeInternalNode::GetInitFunction();
    auto cryoInternalUpdate = CryoEMOctreeInternalNode::GetUpdateFunction(density);
    
    // Use the proper version of IsPointInBound with all three arguments
    bool in_bounds = Octree::IsPointInBound(point, this->origin_, this->size_);
    if (!in_bounds) {
        utility::LogWarning("Point {} is outside octree bounds", point.transpose());
        return;
    }
    
    // Call the base class InsertPoint with proper function types
    this->InsertPoint(point, cryoLeafInit, cryoLeafUpdate, cryoInternalInit, cryoInternalUpdate);
}

void CryoEMOctree::InsertCryoEMSubtree(const Eigen::Vector3d &point,
                                        std::shared_ptr<CryoEMOctree> subtree) {
    if (!Octree::IsPointInBound(point, this->origin_, this->size_)) {
        utility::LogWarning("Insertion point {} is outside octree bounds", point.transpose());
        return;
    }

    // Prepare the lambda functions needed by the base InsertSubtree.
    // Here we use the CryoEM-specific initialization and update functions.
    auto cryoLeafInit = CryoEMOctreeLeafNode::GetInitFunction();
    auto cryoLeafUpdate = CryoEMOctreeLeafNode::GetUpdateFunction(0.0f); // Use 0.0f or an appropriate default
    auto cryoInternalInit = CryoEMOctreeInternalNode::GetInitFunction();
    auto cryoInternalUpdate = CryoEMOctreeInternalNode::GetUpdateFunction(0.0f);
    
    // Correct the call: pass the point, the entire subtree, and the four required function callbacks.
    this->InsertSubtree(point, subtree, cryoLeafInit, cryoLeafUpdate, cryoInternalInit, cryoInternalUpdate);
}


/**
 * @brief Helper function to recursively merge two CryoEMOctreeInternalNodes.
 * 
 * This function merges child nodes from src into dst, ensuring that data 
 * is combined correctly.
 */
void MergeInternalNodes(std::shared_ptr<CryoEMOctreeInternalNode> dst,
                        std::shared_ptr<CryoEMOctreeInternalNode> src) {
    for (size_t i = 0; i < 8; i++) {
        if (src->children_[i]) {
            if (!dst->children_[i]) {
                // If the destination node does not have this child, copy it directly
                dst->children_[i] = src->children_[i];
            } else {
                // If both have a child at this position, recursively merge them
                auto src_internal = std::dynamic_pointer_cast<CryoEMOctreeInternalNode>(src->children_[i]);
                auto dst_internal = std::dynamic_pointer_cast<CryoEMOctreeInternalNode>(dst->children_[i]);

                if (src_internal && dst_internal) {
                    MergeInternalNodes(dst_internal, src_internal);
                } else {
                    // Conflict: Keep one (e.g., max density or average)
                    auto src_leaf = std::dynamic_pointer_cast<CryoEMOctreeLeafNode>(src->children_[i]);
                    auto dst_leaf = std::dynamic_pointer_cast<CryoEMOctreeLeafNode>(dst->children_[i]);

                    if (src_leaf && dst_leaf) {
                        // Average density value (or choose max if needed)
                        dst_leaf->density_ = (dst_leaf->density_ + src_leaf->density_) / 2.0f;
                    } else if (src_leaf) {
                        dst->children_[i] = src_leaf;
                    }
                }
            }
        }
    }
}

void CryoEMOctree::SplitTreeGeneric() {
    // Implementation for splitting nodes can be added as needed.
}

void CryoEMOctree::AggregateSubtree(std::shared_ptr<OctreeNode> node) {
    auto internal = std::dynamic_pointer_cast<CryoEMOctreeInternalNode>(node);
    if (internal) {
        for (auto &child : internal->children_) {
            if (child) {
                AggregateSubtree(child);
            }
        }
        internal->AggregateChildren();
    }
}

//------------------------------------------------------------------------------
// Implementation for CryoEMOctree::CompressOctreeRecursive
//------------------------------------------------------------------------------
void CryoEMOctree::CompressOctreeRecursive(std::shared_ptr<OctreeNode> &node,
                                            float tolerance,
                                            int &merge_count,
                                            std::vector<float> &merge_errors,
                                            bool &changes) {
    if (!node) return;

    auto internal = std::dynamic_pointer_cast<OctreeInternalNode>(node);
    if (!internal) return;

    // Post‑order traversal.
    for (auto &child : internal->children_) {
        CompressOctreeRecursive(child, tolerance, merge_count, merge_errors, changes);
    }

    // Check if all non-null children are leaves.
    bool compressible = true;
    std::vector<std::shared_ptr<CryoEMOctreeLeafNode>> leaf_children;
    for (auto &child : internal->children_) {
        if (child) {
            auto leaf = std::dynamic_pointer_cast<CryoEMOctreeLeafNode>(child);
            if (!leaf) {
                compressible = false;
                break;
            }
            leaf_children.push_back(leaf);
        }
    }
    if (!compressible || leaf_children.empty()) return;

    // Determine density min/max and decide on merge eligibility.
    float min_density = std::numeric_limits<float>::max();
    float max_density = std::numeric_limits<float>::lowest();
    float sum_density = 0.0f;
    for (const auto &leaf : leaf_children) {
        float d = leaf->density_;
        if (d < min_density) { min_density = d; }
        if (d > max_density) { max_density = d; }
        sum_density += d;
    }
    if ((max_density - min_density) > tolerance) return;

    float avg_density = sum_density / static_cast<float>(leaf_children.size());
    float error_sum = 0.0f;
    for (const auto &leaf : leaf_children) {
        error_sum += std::fabs(leaf->density_ - avg_density);
    }
    float error_avg = error_sum / leaf_children.size();
    merge_errors.push_back(error_avg);

    // Compress the node.
    CompressNode(node);
    merge_count++;
}

//------------------------------------------------------------------------------
// Implementation for CryoEMOctree::CompressOctree
//------------------------------------------------------------------------------
void CryoEMOctree::CompressOctree(float tolerance, int &merge_count, float &avg_error) {
    merge_count = 0;
    std::vector<float> merge_errors;
    bool changes = true;
    int pass_count = 0;

    while (changes) {
        pass_count++;
        changes = false;
        CompressOctreeRecursive(root_node_, tolerance, merge_count, merge_errors, changes);
        
        auto counts = CountNodes();
        int current_node_count = counts.total_nodes;
        utility::LogInfo("[DEBUG] End of pass {} ; Total merges: {} ; Node count: {}",
                         pass_count, merge_count, current_node_count);
    }
    if (!merge_errors.empty()) {
        avg_error = std::accumulate(merge_errors.begin(), merge_errors.end(), 0.0f) / merge_errors.size();
    } else {
        avg_error = 0.0f;
    }
    utility::LogInfo("[DEBUG] Compression finished after {} passes. Final merge count: {} ; avg error: {}",
                     pass_count, merge_count, avg_error);
}

//////////////////////////
// Adaptive Compression
//////////////////////////

void CryoEMOctree::CompressOctreeAdaptive(float base_tolerance, int &merge_count, float &avg_error) {
    merge_count = 0;
    std::vector<float> merge_errors;
    bool changes = true;
    int pass_count = 0;
    
    auto counts = CountNodes();
    int current_node_count = counts.total_nodes;
    utility::LogInfo("[DEBUG] Node count before merging {}", current_node_count);
    
    while (changes) {
        pass_count++;
        changes = false;
        CompressOctreeRecursiveAdaptive(root_node_, base_tolerance, merge_count, merge_errors, changes);
        
        auto counts = CountNodes();
        current_node_count = counts.total_nodes;
        utility::LogInfo("[DEBUG] Adaptive pass {} ; Total merges: {} ; Node count: {}",
                         pass_count, merge_count, current_node_count);
    }
    
    if (!merge_errors.empty()) {
        avg_error = std::accumulate(merge_errors.begin(), merge_errors.end(), 0.0f) / merge_errors.size();
    } else {
        avg_error = 0.0f;
    }
    
    utility::LogInfo("[DEBUG] Adaptive compression finished after {} passes. Final merge count: {} ; avg error: {}",
                     pass_count, merge_count, avg_error);
}

void CryoEMOctree::CompressOctreeRecursiveAdaptive(std::shared_ptr<OctreeNode> &node,
                                                     float base_tolerance,
                                                     int &merge_count,
                                                     std::vector<float> &merge_errors,
                                                     bool &changes) {
    if (!node) return;

    auto internal = std::dynamic_pointer_cast<OctreeInternalNode>(node);
    if (!internal) return;

    // Post‑order traversal.
    for (auto &child : internal->children_) {
        CompressOctreeRecursiveAdaptive(child, base_tolerance, merge_count, merge_errors, changes);
    }

    // Check if all non-null children are leaves.
    bool compressible = true;
    std::vector<std::shared_ptr<CryoEMOctreeLeafNode>> leaf_children;
    for (auto &child : internal->children_) {
        if (child) {
            auto leaf = std::dynamic_pointer_cast<CryoEMOctreeLeafNode>(child);
            if (!leaf) {
                compressible = false;
                break;
            }
            leaf_children.push_back(leaf);
        }
    }
    if (!compressible || leaf_children.empty()) return;

    // Compute local density statistics.
    float sum_density = 0.0f;
    float min_density = std::numeric_limits<float>::max();
    float max_density = std::numeric_limits<float>::lowest();
    std::vector<float> densities;
    for (const auto &leaf : leaf_children) {
        float d = leaf->density_;
        densities.push_back(d);
        sum_density += d;
        min_density = std::min(min_density, d);
        max_density = std::max(max_density, d);
    }
    float avg_density = sum_density / static_cast<float>(leaf_children.size());

    // Compute local standard deviation.
    float variance = 0.0f;
    for (float d : densities) {
        variance += (d - avg_density) * (d - avg_density);
    }
    variance /= static_cast<float>(leaf_children.size());
    float sigma = std::sqrt(variance);

    // Compute adaptive tolerance.
    float adaptive_tolerance = base_tolerance / (1.0f + sigma);
    // Enforce a stricter tolerance in high-density regions.
    const float density_threshold = 1.0f; // adjust as necessary
    if (avg_density > density_threshold) {
        adaptive_tolerance = std::min(adaptive_tolerance, base_tolerance * 0.5f);
    }

    // If the range of densities exceeds the adaptive tolerance, do not compress.
    if ((max_density - min_density) > adaptive_tolerance) return;

    // Compute error to record merge quality.
    float error_sum = 0.0f;
    for (const auto &leaf : leaf_children) {
        error_sum += std::fabs(leaf->density_ - avg_density);
    }
    float error_avg = error_sum / static_cast<float>(leaf_children.size());
    merge_errors.push_back(error_avg);

    // Compress the node.
    CompressNode(node);
    merge_count++;
    changes = true;
}

// Helper function to recursively count nodes
static NodeCounts CountNodesRecursive(const std::shared_ptr<OctreeNode>& node, int current_depth = 0, int max_depth = 0) {
    if (!node)
        return {0, 0, 0};
        
    NodeCounts counts = {1, 0, 0};  // Start with 1 total node
    
    auto internal = std::dynamic_pointer_cast<OctreeInternalNode>(node);
    if (internal) {
        counts.internal_nodes = 1;  // This is an internal node
        for (const auto &child : internal->children_) {
            auto child_counts = CountNodesRecursive(child, current_depth + 1, max_depth);
            counts.total_nodes += child_counts.total_nodes;
            counts.internal_nodes += child_counts.internal_nodes;
            counts.leaf_nodes += child_counts.leaf_nodes;
        }
    } else {
        counts.leaf_nodes = 1;  // This is a leaf node
        if (current_depth < max_depth) {
            // Log information about this non-maximal leaf
            auto leaf = std::dynamic_pointer_cast<CryoEMOctreeLeafNode>(node);
            utility::LogInfo("Found non-maximal leaf at depth {}/{} with density {}",
                           current_depth, max_depth,
                           leaf ? leaf->density_ : 0.0f);
        }
    }
    
    return counts;
}

NodeCounts CryoEMOctree::CountNodes() const {
    // Now pass the max_depth_ from the octree to the recursive function
    auto counts = CountNodesRecursive(root_node_, 0, this->max_depth_);
    
    // Print node structure details
    utility::LogInfo("Root node exists: {}", root_node_ != nullptr);
    if (root_node_) {
        auto internal = std::dynamic_pointer_cast<OctreeInternalNode>(root_node_);
        if (internal) {
            int child_count = 0;
            for (const auto& child : internal->children_) {
                if (child) child_count++;
            }
            utility::LogInfo("Root is internal with {} non-null children", child_count);
        }
    }
    
    utility::LogInfo("Total nodes: {}, Internal nodes: {}, Leaf nodes: {}", 
                    counts.total_nodes, counts.internal_nodes, counts.leaf_nodes);
    
    return counts;
}

void CryoEMOctree::ConvertVoxelMapToOctree(
    const py::array_t<float>& density_array,
    double map_size,
    int target_tasks
    ) {

    // Get buffer and shape information from numpy array
    py::buffer_info buf = density_array.request();
    if (buf.ndim != 3) {
        utility::LogError("Expected 3D numpy array, but got {}D array", buf.ndim);
    }
    
    // int max_depth = this->max_depth_;

    // Extract shape information
    size_t nx = buf.shape[0];
    size_t ny = buf.shape[1];
    size_t nz = buf.shape[2];
    float* data_ptr = static_cast<float*>(buf.ptr);

    // Print the shape of the density array
    utility::LogInfo("Density array shape: {}x{}x{}", nx, ny, nz);

    // Compute voxel sizes in physical units (it should be the same for all the leaf nodes)
    double voxelSizeX = map_size / static_cast<double>(nx);
    double voxelSizeY = map_size / static_cast<double>(ny);
    double voxelSizeZ = map_size / static_cast<double>(nz);

    // Define grid size based on dimensions
    // typedef Eigen::Matrix<long double, 3, 1> Vector3ld;
    Eigen::Vector3d grid_size(nx, ny, nz);
    
    // Step 1: Use the existing origin and size
    Eigen::Vector3d map_origin = this->origin_;

    //Printing the origin and size
    utility::LogInfo("The origin of the map is: {}", map_origin.transpose());
    utility::LogInfo("The size of the map is: {}", map_size);
    utility::LogInfo("The voxel size of the map is: {} {} {}", voxelSizeX, voxelSizeY, voxelSizeZ);
    utility::LogInfo("The origin of the octree is: {}", this->origin_.transpose());
    utility::LogInfo("The size of the octree is: {}", this->size_);



    // --- Step 2: Subdivide the map into subtrees that align with the voxel grid ---
    int depthOfSubdivisions = std::log2(target_tasks)/std::log2(8);
    int numSubdivisions = std::pow(2, depthOfSubdivisions);
    int depthOfSubtrees = this->max_depth_ - depthOfSubdivisions;
    double subregion_size_length = this->size_ / numSubdivisions; 
    utility::LogInfo("numSubDivisions: {}, target_tasks: {}, that means the depth of the subtrees will be {}",
            numSubdivisions, target_tasks, depthOfSubtrees);

    
    
    utility::LogInfo("Subdividing map into {} subregions per axis, each with approximately {} voxels per side", 
                    numSubdivisions, subregion_size_length);

    // Create a vector to hold all the sub octrees.
    std::vector<std::shared_ptr<CryoEMOctree>> subregions_octree;
    for (int i = 0; i < numSubdivisions; ++i) {
        for (int j = 0; j < numSubdivisions; ++j) {
            for (int k = 0; k < numSubdivisions; ++k) {
                // Compute the origin phisical position for this subtree.
                Eigen::Vector3d origin(map_origin.x() + i * subregion_size_length, 
                                             map_origin.y() + j * subregion_size_length, 
                                             map_origin.z() + k * subregion_size_length);
                utility::LogInfo("Creating subtree at depth {} with origin {} and size {}",
                        depthOfSubtrees, 
                        origin.cast<double>().transpose(),
                        subregion_size_length);

                subregions_octree.push_back(std::make_shared<CryoEMOctree>(
                                    depthOfSubtrees, 
                                    origin,
                                    subregion_size_length));
            }
        }
    }

    // Start subdivision over the entire domain.
    std::atomic<size_t> points_inserted(0);
    
    // Step 3: Parallel voxel insertion
    auto start_time = std::chrono::high_resolution_clock::now();

    // Now use the fixed subtrees vector size for parallel voxel insertion.
    tbb::parallel_for(tbb::blocked_range<size_t>(0, subregions_octree.size()),
        [&](const tbb::blocked_range<size_t> &range) {
            for (size_t i = range.begin(); i < range.end(); i++) {
                //Get the origin index of the subtree
                Eigen::Vector3i subtree_origin_index = (subregions_octree[i]->origin_ / voxelSizeX).cast<int>();
                int side_length = static_cast<int>(subregions_octree[i]->size_ / voxelSizeX);

                for (int x = subtree_origin_index.x(); x < subtree_origin_index.x() + side_length; x++) {
                    for (int y = subtree_origin_index.y(); y < subtree_origin_index.y() + side_length; y++) {
                        for (int z = subtree_origin_index.z(); z < subtree_origin_index.z() + side_length; z++) {
                            // Check if the index is within the bounds of the density array
                            if (x >= 0 && x < static_cast<int>(nx) &&
                                y >= 0 && y < static_cast<int>(ny) &&
                                z >= 0 && z < static_cast<int>(nz)) {
                                float density = data_ptr[x * ny * nz + y * nz + z];
                                // Inserting the points (whe give the position at the center of the leaf to check position during insertion to avoid misalignments)
                                double pos_x = x * voxelSizeX + (voxelSizeX/2);
                                double pos_y = y * voxelSizeY + (voxelSizeY/2);
                                double pos_z = z * voxelSizeZ + (voxelSizeZ/2);
                                Eigen::Vector3d pos(pos_x, pos_y, pos_z);
                                points_inserted++;
                                subregions_octree[i]->InsertDensityPoint(pos, density);
                            }
                        }
                    }
                }
            }
        });

    auto end_time = std::chrono::high_resolution_clock::now();    
    auto merge_duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
    utility::LogInfo("Voxel insertion took {} seconds", merge_duration);
    utility::LogInfo("Points actually inserted: {}", points_inserted.load());
    

    // DEBUGGING
    //Count sum of all nodes in all subtrees
    utility::LogInfo("Counting nodes in all subtrees");
    int sum_nodes = 0;
    for (auto &subtree_data : subregions_octree) {
        auto counts = subtree_data->CountNodes();
        sum_nodes += counts.total_nodes;
    }
    utility::LogInfo("Sum of all nodes in all subtrees: {}", sum_nodes);

    // END DEBUGGING

    start_time = std::chrono::high_resolution_clock::now();

    // Step 4: Sequential merging of sub-octrees into a correct hierarchical structure
    auto merge_start_time = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < subregions_octree.size(); i++) {
        this->InsertCryoEMSubtree(subregions_octree[i]->origin_, subregions_octree[i]);
    }
    auto merge_end_time = std::chrono::high_resolution_clock::now();
    merge_duration = std::chrono::duration_cast<std::chrono::seconds>(merge_end_time - merge_start_time).count();
    open3d::utility::LogInfo("Tree reconstruction took {} seconds", merge_duration);

    //count nodes after merging
    auto counts = this->CountNodes();
    int node_count = counts.total_nodes;
    utility::LogInfo("Node count after tree reconstruction: {}", node_count);
}

std::shared_ptr<OctreeLeafNode> CryoEMOctree::ConvertInternalToLeaf(
        const std::shared_ptr<OctreeInternalNode>& internal) {
    if (!internal->children_.empty()) {
        // Look for the first non-null child
        for (const auto& child : internal->children_) {
            if (child) {
                // Try to cast it to a CryoEMOctreeLeafNode
                auto leaf = std::dynamic_pointer_cast<CryoEMOctreeLeafNode>(child);
                if (leaf) {
                    // Clone and return the leaf
                    return std::dynamic_pointer_cast<OctreeLeafNode>(leaf->Clone());
                }
            }
        }
    }
    
    // If we couldn't find a valid child to clone, create a new default leaf node
    return std::make_shared<CryoEMOctreeLeafNode>();
}

} // namespace geometry
} // namespace open3d
