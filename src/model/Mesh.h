/**
 * @file Mesh.h
 * @brief Mesh data structure wrapper for libigl
 */
#pragma once

#include <Eigen/Core>

/**
 * @class Mesh
 * @brief Mesh representation class with vertices, faces and colors. Furthermore
 * it stores an ID per mesh which is the index the mesh has in the renderer data
 * structure.
 */
class Mesh {
public:
  /**
   * @brief Create a new mesh.
   * @param V Vertices matrix of shape (N x 3)
   * @param F Faces matrix of shape (N x 3), note that the sequence per triangle
   * is important, as (1, 2, 3) and (3, 2, 1) are the same triangle, but with
   * inverted normals!
   */
  Mesh(Eigen::MatrixX3d V, Eigen::MatrixX3i F);

  /**
   * @brief Get the number of vertices.
   * @return const Eigen::Index number of vertices
   */
  const Eigen::Index numVertices() const;

  /**
   * @brief Get a copy of the vertex matrix.
   * @return const Eigen::MatrixXd non-modifiable vertex matrix
   */
  const Eigen::MatrixX3d getVertices() const;

  /**
   * @brief Get a copy of the face matrix.
   * @return const Eigen::MatrixXi non-modifiable face matrix
   */
  const Eigen::MatrixX3i getFaces() const;

  /**
   * @brief Get the number of faces.
   * @return const Eigen::Index number of faces
   */
  const Eigen::Index numFaces() const;

  /**
   * @brief Update the vertex positions of the vertices
   * @param V New vertex matrix of shape (N, 3), note that this function does
   * not check wether the new vertex positions are valid.
   */
  void updateVertices(Eigen::MatrixX3d V);

  /**
   * @brief Getter for the color of the mesh
   * @return Color of mesh
   */
  Eigen::Vector4f getColor() const;

  /**
   * @brief Set the color of the mesh
   * @param r Red contribution, should be in [0, 1]
   * @param g Green contribution, should be in [0, 1]
   * @param b Blue contribution, should be in [0, 1]
   * @param alpha Transparency, should be in [0, 1]
   */
  void updateColor(float r, float g, float b, float alpha);

  /**
   * @brief Get the internal ID of the mesh (renderer ID, not libigl)
   * @return size_t index of mesh
   */
  size_t getID();

  /**
   * @brief Set the internal ID of the mesh (should be set by renderer)
   * @param id new id
   */
  void setID(size_t id);

private:
  // @TODO: Currently stores the vertex positions directly, might be smart to
  // add an origin position just like in the PBS example projects.
  Eigen::MatrixX3d V; ///< Vertex matrix of shape (N x 3)
  Eigen::MatrixX3i F; ///< Face matrix of shape (N x 3)
  // @TODO: Update single color vector with color matrix (see libigl)
  Eigen::Vector4f color =
      Eigen::Vector4f(0.2f, 0.2f, 0.2f, 1.0f); ///< Mesh color

  size_t ID = -1; ///< ID set by the renderer, -1 means uninitialized
};