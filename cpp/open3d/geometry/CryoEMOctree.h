#pragma once

#include "open3d/geometry/Octree.h"
#include <memory>
#include <vector>
#include <functional>

namespace open3d {
namespace geometry {

/**
 * @brief A derived octree leaf node class that stores Cryo-EM specific data.
 */
class CryoEMOctreeLeafNode : public OctreeLeafNode {
public:
    /// Smart pointer type for CryoEMOctreeLeafNode.
    using Ptr = std::shared_ptr<CryoEMOctreeLeafNode>;

    /**
     * @brief Default constructor initializing density and resolution.
     */
    CryoEMOctreeLeafNode();

    /**
     * @brief Creates a deep copy of the leaf node.
     * @return A shared pointer to the cloned OctreeLeafNode.
     */
    std::shared_ptr<OctreeLeafNode> Clone() const override;

    /**
     * @brief Serializes the node data to a JSON value.
     * @param value JSON value to hold serialized data.
     * @return True if serialization is successful.
     */
    bool ConvertToJsonValue(Json::Value &value) const override;

    /**
     * @brief Deserializes the node data from a JSON value.
     * @param value JSON value containing the node data.
     * @return True if deserialization is successful.
     */
    bool ConvertFromJsonValue(const Json::Value &value) override;

    /**
     * @brief Compares this leaf node with another OctreeLeafNode.
     * @param other The other OctreeLeafNode to compare with.
     * @return True if nodes are equal, false otherwise.
     */
    bool operator==(const OctreeLeafNode& other) const override;

    /// Returns a lambda that creates a new CryoEMOctreeLeafNode.
    static std::function<std::shared_ptr<OctreeLeafNode>()> GetInitFunction();

    /// Returns a lambda that updates a CryoEMOctreeLeafNode with the given density and resolution.
    static std::function<void(std::shared_ptr<OctreeLeafNode>)>
    GetUpdateFunction(float density, float resolution);

    // Cryo-EM specific data fields.
    float density_;
    float resolution_;
};

/**
 * @brief A derived octree internal node class that stores aggregated Cryo-EM data.
 */
class CryoEMOctreeInternalNode : public OctreeInternalNode {
public:
    /// Smart pointer type for CryoEMOctreeInternalNode.
    using Ptr = std::shared_ptr<CryoEMOctreeInternalNode>;

    /**
     * @brief Default constructor initializing density and resolution.
     */
    CryoEMOctreeInternalNode();

    /**
     * @brief Aggregates density and resolution values from child nodes.
     */
    void AggregateChildren();

    /**
     * @brief Creates a deep copy of the internal node.
     * @return A shared pointer to the cloned OctreeInternalNode.
     */
    std::shared_ptr<OctreeInternalNode> Clone() const;

    // Aggregated Cryo-EM specific data fields.
    float density_;
    float resolution_;
};

/**
 * @brief An octree structure specialized for storing Cryo-EM data.
 */
class CryoEMOctree : public Octree {
public:
    /// Smart pointer type for CryoEMOctree.
    using Ptr = std::shared_ptr<CryoEMOctree>;

    /**
     * @brief Constructs a new CryoEMOctree.
     * @param max_depth Maximum depth of the octree.
     * @param origin Origin of the octree.
     * @param size Size of the octree.
     */
    CryoEMOctree(int max_depth, const Eigen::Vector3d &origin, double size);

    /**
     * @brief Inserts a point along with its cryo-EM density and resolution.
     * @param point The 3D point to be inserted.
     * @param density Density value at the point.
     * @param resolution Resolution value at the point.
     */
    void InsertDensityPoint(const Eigen::Vector3d &point, float density, float resolution);

    void CompressNode(std::shared_ptr<OctreeNode>& node);

    /**
     * @brief Splits the octree for further refinement.
     */
    void SplitTreeGeneric();

private:
    /**
     * @brief Recursively aggregates data in the subtree starting from the given node.
     * @param node The root node of the subtree.
     */
    void AggregateSubtree(std::shared_ptr<OctreeNode> node);
};

} // namespace geometry
} // namespace open3d
