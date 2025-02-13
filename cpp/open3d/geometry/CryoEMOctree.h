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
     * @brief Default constructor initializing density.
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

    /// Returns a lambda that updates a CryoEMOctreeLeafNode with the given density.
    static std::function<void(std::shared_ptr<OctreeLeafNode>)>
    GetUpdateFunction(float density);

    // Cryo-EM specific data field.
    float density_;
};

/**
 * @brief A derived octree internal node class that stores aggregated Cryo-EM data.
 */
class CryoEMOctreeInternalNode : public OctreeInternalNode {
public:
    /// Smart pointer type for CryoEMOctreeInternalNode.
    using Ptr = std::shared_ptr<CryoEMOctreeInternalNode>;

    /**
     * @brief Default constructor initializing density.
     */
    CryoEMOctreeInternalNode() : OctreeInternalNode() {}

    /**
     * @brief Aggregates density values from child nodes.
     */
    void AggregateChildren();

    /**
     * @brief Creates a deep copy of the internal node.
     * @return A shared pointer to the cloned OctreeInternalNode.
     */
    std::shared_ptr<OctreeInternalNode> Clone() const;

    // Aggregated Cryo-EM specific data field.
    float density_;
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
     * @brief Inserts a point along with its Cryo-EM density.
     * @param point The 3D point to be inserted.
     * @param density Density value at the point.
     */
    void InsertDensityPoint(const Eigen::Vector3d &point, float density);

    void CompressNode(std::shared_ptr<OctreeNode>& node);

    /**
     * @brief Splits the octree for further refinement.
     */
    void SplitTreeGeneric();

    /**
     * @brief Returns the total number of nodes in the octree.
     * @return The count of nodes.
     */
    int CountNodes() const;

    /**
     * @brief Recursively compresses the octree using a standard fixed tolerance.
     *
     * All internal nodes whose non-null children are leaves and whose children's densities
     * vary by no more than tolerance are replaced by a merged leaf node.
     *
     * @param tolerance Maximum allowed difference between children densities.
     * @param merge_count (Output) Total number of merges performed.
     * @param avg_error   (Output) Average error of the merges.
     */
    void CompressOctree(float tolerance, int &merge_count, float &avg_error);

    /**
     * @brief Recursively compresses the octree using an adaptive metric that takes into account the local density variation.
     *
     * Regions with higher average density (and hence higher information content) are compressed
     * less aggressively (i.e. with a lower effective tolerance) to preserve fine details.
     *
     * @param base_tolerance Base tolerance value used as the starting point for computing the adaptive tolerance.
     * @param merge_count (Output) Total number of merges performed.
     * @param avg_error   (Output) Average error of the merges.
     */
    void CompressOctreeAdaptive(float base_tolerance, int &merge_count, float &avg_error);

private:
    /**
     * @brief Helper for CompressOctree: recursively compress the tree with fixed tolerance.
     */
    void CompressOctreeRecursive(std::shared_ptr<OctreeNode> &node,
                                 float tolerance,
                                 int &merge_count,
                                 std::vector<float> &merge_errors,
                                 bool &changes);

    /**
     * @brief Helper for CompressOctreeAdaptive: recursively compress the tree using an adaptive tolerance based on local statistics.
     */
    void CompressOctreeRecursiveAdaptive(std::shared_ptr<OctreeNode> &node,
                                          float base_tolerance,
                                          int &merge_count,
                                          std::vector<float> &merge_errors,
                                          bool &changes);

    /**
     * @brief Recursively aggregates data in the subtree starting from the given node.
     * @param node The root node of the subtree.
     */
    void AggregateSubtree(std::shared_ptr<OctreeNode> node);
};

} // namespace geometry
} // namespace open3d
