#pragma once

#include "open3d/geometry/Octree.h"

namespace open3d {
namespace geometry {

/// A derived octree class that stores Cryo-EM specific data.
class CryoEMOctreeLeafNode : public OctreeLeafNode {
public:
    // Define the smart pointer type
    using Ptr = std::shared_ptr<CryoEMOctreeLeafNode>;

    CryoEMOctreeLeafNode() : density_(0.0f), resolution_(0.0f) {}

    /// Custom fields
    float density_;
    float resolution_;

    // Overriding Clone function if you need duplication of node.
    std::shared_ptr<OctreeLeafNode> Clone() const override {
        auto node = std::make_shared<CryoEMOctreeLeafNode>();
        node->density_ = density_;
        node->resolution_ = resolution_;
        // Clone base node data if needed
        return node;
    }

    // You can override other methods (e.g. IsLeaf(), etc.) if needed.

    // Implement pure virtual functions
    bool ConvertToJsonValue(Json::Value &value) const override {
        // Implementation
        return true; // Example return value
    }

    bool ConvertFromJsonValue(const Json::Value &value) override {
        // Implementation
        return true; // Example return value
    }

    bool operator==(const OctreeLeafNode& other) const override {
        // Implementation
        return true; // Example return value
    }
};

class CryoEMOctreeInternalNode : public OctreeInternalNode {
public:
    using Ptr = std::shared_ptr<CryoEMOctreeInternalNode>;

    CryoEMOctreeInternalNode() : density_(0.0f), resolution_(0.0f) {}

    float density_;
    float resolution_;

    // Example: aggregate from children
    void AggregateChildren() {
        double sum_density = 0.0;
        double sum_resolution = 0.0;
        int count = 0;
        for (auto &child : children_) {
            if (child) {
                // If child is a CryoEMLeafNode
                auto leaf = std::dynamic_pointer_cast<CryoEMOctreeLeafNode>(child);
                if (leaf) {
                    sum_density += leaf->density_;
                    sum_resolution += leaf->resolution_;
                    count++;
                }
                // If child is a CryoEMInternalNode
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

    std::shared_ptr<OctreeInternalNode> Clone() const {
        auto node = std::make_shared<CryoEMOctreeInternalNode>();
        node->density_ = density_;
        node->resolution_ = resolution_;
        // children_ are typically cloned at the Octree level
        return node;
    }
};

class CryoEMOctree : public Octree {
public:
    using Ptr = std::shared_ptr<CryoEMOctree>;

    CryoEMOctree(int max_depth,
                 const Eigen::Vector3d &origin,
                 double size)
        : Octree(max_depth, origin, size) {
        // By default, we use a CryoEMOctreeInternalNode as root
        root_node_ = std::make_shared<CryoEMOctreeInternalNode>();
    }

    /// Example of specialized insertion
    void InsertDensityPoint(const Eigen::Vector3d &point,
                            float density,
                            float resolution) {
        auto leaf_init = [density, resolution]() {
            auto leaf = std::make_shared<CryoEMOctreeLeafNode>();
            leaf->density_ = density;
            leaf->resolution_ = resolution;
            return leaf;
        };
        auto leaf_update = [density, resolution](std::shared_ptr<OctreeLeafNode> node) {
            auto cryo_node = std::dynamic_pointer_cast<CryoEMOctreeLeafNode>(node);
            if (cryo_node) {
                cryo_node->density_ = density;
                cryo_node->resolution_ = resolution;
            }
        };
        InsertPoint(point, leaf_init, leaf_update);
    }

    // Example aggregator
    void AggregateAllNodes() {
        // If root_node_ is a CryoEMOctreeInternalNode, do a BFS/DFS
        AggregateSubtree(root_node_);
    }

private:
    void AggregateSubtree(std::shared_ptr<OctreeNode> node) {
        auto inode = std::dynamic_pointer_cast<CryoEMOctreeInternalNode>(node);
        if (inode) {
            // Recurse
            for (auto &child : inode->children_) {
                if (child) {
                    AggregateSubtree(child);
                }
            }
            // Then aggregate
            inode->AggregateChildren();
        }
        // If it's a leaf node, nothing to do
    }
};

} // namespace geometry
} // namespace open3d
