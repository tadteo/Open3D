#include "open3d/geometry/CryoEMOctree.h"
#include <cmath>           // for std::fabs
#include <json/json.h>     // For JSON handling (if needed)
#include "open3d/utility/Logging.h"

/*
 * This file contains the implementation of the CryoEMOctree and its associated nodes.
 */

namespace open3d {
namespace geometry {

/**
 * @brief Attempts to compress an internal Cryo-EM node in place, if all its immediate children are leaf nodes
 * or null. The compression aggregates the density and resolution values using a weighted average (by resolution),
 * and replaces the node with a new leaf node if eligible.
 *
 * This function modifies the tree in place: when called with a reference to a node pointer in the tree, if the node
 * qualifies for compression, its pointer is updated to point to a new CryoEMOctreeLeafNode with the aggregated values.
 *
 * @param node A reference to the node pointer to possibly compress.
 */
void CryoEMOctree::CompressNode(std::shared_ptr<OctreeNode>& node) {
    if (!node) return;
    
    // Check if the node is an internal Cryo-EM node.
    auto internal = std::dynamic_pointer_cast<CryoEMOctreeInternalNode>(node);
    if (!internal) {
        // Node is already a leaf (or not an internal node), so nothing to compress.
        return;
    }
    
    // Check that every non-null child is a Cryo-EM leaf node.
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
        // The node is not eligible for compression.
        return;
    }
    
    // Aggregate the leaf data using a weighted average (using each leaf's resolution as its weight).
    float total_weight = 0.0f;
    float weighted_density_sum = 0.0f;
    float weighted_resolution_sum = 0.0f;
    for (const auto& leaf : leaves) {
        float weight = leaf->resolution_;
        total_weight += weight;
        weighted_density_sum += leaf->density_ * weight;
        weighted_resolution_sum += leaf->resolution_ * weight;
    }
    
    float new_density = 0.0f, new_resolution = 0.0f;
    if (total_weight > 0.0f) {
        new_density = weighted_density_sum / total_weight;
        new_resolution = weighted_resolution_sum / total_weight;
    } else {
        // If total weight is zero, fall back to the arithmetic mean.
        for (const auto& leaf : leaves) {
            new_density += leaf->density_;
            new_resolution += leaf->resolution_;
        }
        new_density /= static_cast<float>(leaves.size());
        new_resolution /= static_cast<float>(leaves.size());
    }
    
    // Construct a new leaf node representing the compressed branch.
    auto merged_leaf = std::make_shared<CryoEMOctreeLeafNode>();
    merged_leaf->density_ = new_density;
    merged_leaf->resolution_ = new_resolution;
    
    // Update the node in place by replacing it with the merged leaf.
    node = merged_leaf;
}

//==============================================================================
// Implementation for CryoEMOctreeLeafNode
//==============================================================================

CryoEMOctreeLeafNode::CryoEMOctreeLeafNode() 
    : density_(0.0f), resolution_(0.0f) {}

std::shared_ptr<OctreeLeafNode> CryoEMOctreeLeafNode::Clone() const {
    auto node = std::make_shared<CryoEMOctreeLeafNode>();
    node->density_ = density_;
    node->resolution_ = resolution_;
    // Additional base class data can be cloned here if needed.
    return node;
}

bool CryoEMOctreeLeafNode::ConvertToJsonValue(Json::Value &value) const {
    // Serialize the Cryo-EM specific data.
    value["density"] = density_;
    value["resolution"] = resolution_;
    // Serialize base class data here, if necessary.
    return true;
}

bool CryoEMOctreeLeafNode::ConvertFromJsonValue(const Json::Value &value) {
    // Ensure required fields are present.
    if (!value.isMember("density") || !value.isMember("resolution"))
        return false;
        
    density_ = value["density"].asFloat();
    resolution_ = value["resolution"].asFloat();
    // Deserialize base class data here if needed.
    return true;
}

bool CryoEMOctreeLeafNode::operator==(const OctreeLeafNode& other) const {
    // Attempt dynamic cast to compare Cryo-EM-specific fields.
    const CryoEMOctreeLeafNode* other_leaf = dynamic_cast<const CryoEMOctreeLeafNode*>(&other);
    if (!other_leaf) return false;
    return (std::fabs(density_ - other_leaf->density_) < 1e-6f &&
            std::fabs(resolution_ - other_leaf->resolution_) < 1e-6f);
}

std::function<std::shared_ptr<OctreeLeafNode>()> 
CryoEMOctreeLeafNode::GetInitFunction() {
    return []() -> std::shared_ptr<OctreeLeafNode> {
        return std::make_shared<CryoEMOctreeLeafNode>();
    };
}

std::function<void(std::shared_ptr<OctreeLeafNode>)>
CryoEMOctreeLeafNode::GetUpdateFunction(float density, float resolution) {
    return [density, resolution](std::shared_ptr<OctreeLeafNode> node) -> void {
        if (auto cryo_node = 
                std::dynamic_pointer_cast<CryoEMOctreeLeafNode>(node)) {
            cryo_node->density_ = density;
            cryo_node->resolution_ = resolution;
        } else {
            utility::LogError("Internal error: node must be CryoEMOctreeLeafNode");
        }
    };
}

//==============================================================================
// Implementation for CryoEMOctreeInternalNode
//==============================================================================

CryoEMOctreeInternalNode::CryoEMOctreeInternalNode() 
    : density_(0.0f), resolution_(0.0f) {}

void CryoEMOctreeInternalNode::AggregateChildren() {
    double sum_density = 0.0;
    double sum_resolution = 0.0;
    int count = 0;
    // Aggregate data from each child.
    for (auto &child : children_) {
        if (child) {
            auto leaf = std::dynamic_pointer_cast<CryoEMOctreeLeafNode>(child);
            if (leaf) {
                sum_density += leaf->density_;
                sum_resolution += leaf->resolution_;
                count++;
                continue;
            }
            auto inode = std::dynamic_pointer_cast<CryoEMOctreeInternalNode>(child);
            if (inode) {
                sum_density += inode->density_;
                sum_resolution += inode->resolution_;
                count++;
            }
        }
    }
    if (count > 0) {
        density_ = static_cast<float>(sum_density / count);
        resolution_ = static_cast<float>(sum_resolution / count);
    }
}

std::shared_ptr<OctreeInternalNode> CryoEMOctreeInternalNode::Clone() const {
    auto node = std::make_shared<CryoEMOctreeInternalNode>();
    node->density_ = density_;
    node->resolution_ = resolution_;
    // Note: Cloning of child nodes is typically handled at the tree level.
    return node;
}

//==============================================================================
// Implementation for CryoEMOctree
//==============================================================================

CryoEMOctree::CryoEMOctree(int max_depth, const Eigen::Vector3d &origin, double size)
    : Octree(max_depth, origin, size) {
    // By default, initialize the root as a CryoEM internal node.
    root_node_ = std::make_shared<CryoEMOctreeInternalNode>();
}

void CryoEMOctree::InsertDensityPoint(const Eigen::Vector3d &point, float density, float resolution) {
    // Lambda to create a new leaf node with cryo-EM data.
    auto leaf_init = [density, resolution]() -> std::shared_ptr<OctreeLeafNode> {
        auto leaf = std::make_shared<CryoEMOctreeLeafNode>();
        leaf->density_ = density;
        leaf->resolution_ = resolution;
        return leaf;
    };
    // Lambda to update an existing leaf node's cryo-EM data.
    auto leaf_update = [density, resolution](std::shared_ptr<OctreeLeafNode> node) {
        auto cryo_node = std::dynamic_pointer_cast<CryoEMOctreeLeafNode>(node);
        if (cryo_node) {
            cryo_node->density_ = density;
            cryo_node->resolution_ = resolution;
        }
    };
    // Use base class InsertPoint method.
    InsertPoint(point, leaf_init, leaf_update);
}


void CryoEMOctree::SplitTreeGeneric() {
    // Implementation for splitting nodes can be added as needed.
    // Currently, this method is not implemented.
}

void CryoEMOctree::AggregateSubtree(std::shared_ptr<OctreeNode> node) {
    // If the node is an internal CryoEM node, process its children recursively.
    auto internal = std::dynamic_pointer_cast<CryoEMOctreeInternalNode>(node);
    if (internal) {
        for (auto &child : internal->children_) {
            if (child) {
                AggregateSubtree(child);
            }
        }
        // After processing children, aggregate data up to the current node.
        internal->AggregateChildren();
    }
    // For leaf nodes, no aggregation is necessary.
}

} // namespace geometry
} // namespace open3d
