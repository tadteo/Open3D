#include "open3d/geometry/CryoEMOctree.h"
#include <cmath>           // for std::fabs
#include <json/json.h>     // For JSON handling (if needed)
#include "open3d/utility/Logging.h"
#include <numeric>  // for std::accumulate
#include <limits>   // for std::numeric_limits

/*
 * This file contains the implementation of the CryoEMOctree and its associated nodes.
 */

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

std::shared_ptr<OctreeLeafNode> CryoEMOctreeLeafNode::Clone() const {
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
        if (auto cryo_node = std::dynamic_pointer_cast<CryoEMOctreeLeafNode>(node)) {
            cryo_node->density_ = density;
        } else {
            utility::LogError("Internal error: node must be CryoEMOctreeLeafNode");
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

std::shared_ptr<OctreeInternalNode> CryoEMOctreeInternalNode::Clone() const {
    auto node = std::make_shared<CryoEMOctreeInternalNode>();
    node->density_ = density_;
    return node;
}

//==============================================================================
// Implementation for CryoEMOctree
//==============================================================================

CryoEMOctree::CryoEMOctree(int max_depth, const Eigen::Vector3d &origin, double size)
    : Octree(max_depth, origin, size) {
    root_node_ = std::make_shared<CryoEMOctreeInternalNode>();
}

void CryoEMOctree::InsertDensityPoint(const Eigen::Vector3d &point, float density) {
    // Define an initializer for Cryo‑EM leaf nodes.
    auto cryoLeafInit = []() -> std::shared_ptr<OctreeLeafNode> {
        return std::make_shared<CryoEMOctreeLeafNode>();
    };

    // Define an updater for Cryo‑EM leaf nodes that stores the density.
    auto cryoLeafUpdate = [density](std::shared_ptr<OctreeLeafNode> node) {
        if (auto cryoLeaf = std::dynamic_pointer_cast<CryoEMOctreeLeafNode>(node)) {
            cryoLeaf->density_ = density;
        } else {
            utility::LogError("InsertDensityPoint: Node is not a CryoEMOctreeLeafNode.");
        }
    };

    // Define an initializer for Cryo‑EM internal nodes.
    auto cryoInternalInit = []() -> std::shared_ptr<OctreeInternalNode> {
        return std::make_shared<CryoEMOctreeInternalNode>();
    };

    // Define an updater for Cryo‑EM internal nodes (if needed).
    auto cryoInternalUpdate = [](std::shared_ptr<OctreeInternalNode> node) {
        // Optionally update aggregated parameters.
    };

    // Use the base class InsertPoint method with our lambdas.
    this->InsertPoint(point, cryoLeafInit, cryoLeafUpdate, cryoInternalInit, cryoInternalUpdate);
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
        
        int current_node_count = CountNodes();
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
    
    while (changes) {
        pass_count++;
        changes = false;
        CompressOctreeRecursiveAdaptive(root_node_, base_tolerance, merge_count, merge_errors, changes);
        
        int current_node_count = CountNodes();
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

// Helper function to recursively count nodes.
static int CountNodesRecursive(const std::shared_ptr<OctreeNode>& node) {
    if (!node)
        return 0;
    int count = 1;
    auto internal = std::dynamic_pointer_cast<OctreeInternalNode>(node);
    if (internal) {
        for (const auto &child : internal->children_) {
            count += CountNodesRecursive(child);
        }
    }
    return count;
}

int CryoEMOctree::CountNodes() const {
    return CountNodesRecursive(root_node_);
}

} // namespace geometry
} // namespace open3d
