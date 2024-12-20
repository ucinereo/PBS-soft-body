/**
 * @file Mesh.h
 * @brief Mesh data structure wrapper for libigl
 */
#pragma once

#include "./accel/BVH.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <igl/readOBJ.h>

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
   * @param vertices Vertices matrix of shape (N x 3)
   * @param faces Faces matrix of shape (N x 3), note that the sequence per
   * triangle is important, as (1, 2, 3) and (3, 2, 1) are the same triangle,
   * but with inverted normals!
   */
  Mesh(Eigen::MatrixX3d &vertices, Eigen::MatrixX3i &faces)
      : m_vertices(vertices), m_initialVertices(vertices), m_faces(faces) {}

  /**
   * @brief Create a new mesh.
   * @param vertices Vertices matrix of shape (N x 3)
   * @param faces Faces matrix of shape (N x 3), note that the sequence per
   * triangle is important, as (1, 2, 3) and (3, 2, 1) are the same triangle,
   * but with inverted normals!
   */
  Mesh(Eigen::MatrixX3d &vertices, Eigen::MatrixX3i &faces,
       Eigen::Affine3d &toWorld)
      : m_faces(faces), m_toWorld(toWorld) {
    m_vertices = (toWorld * vertices.transpose()).transpose();
    m_initialVertices = (toWorld * vertices.transpose()).transpose();
  }

  /**
   * @brief Get the number of vertices.
   * @return const Eigen::Index number of vertices
   */
  Eigen::Index numVertices() const { return m_vertices.rows(); };

  /**
   * @brief Get the current vertex positions.
   * @return const Eigen::MatrixXd non-modifiable vertex matrix
   */
  const Eigen::MatrixX3d &getVertices() const { return m_vertices; };

  /**
   * @brief Get the initial vertex positions
   * @return const Eigen::MatrixXd non-modifiable vertex matrix
   */
  const Eigen::MatrixX3d &getInitialVertices() const {
    return m_initialVertices;
  };

  /**
   * @brief Get the the face matrix which index into the vertex matrix.
   * @return const Eigen::MatrixX3i non-modifiable face matrix
   */
  const Eigen::MatrixX3i &getFaces() const { return m_faces; };

  /**
   * @brief Get the tetrahedron matrix which index into the vertex matrix.
   * @return const Eigen::MatrixX4i non-modifiable tet matrix
   */
  const Eigen::MatrixX4i &getTets() const { return m_tets; };

  /**
   * @brief Get the number of faces.
   * @return Eigen::Index number of faces
   */
  Eigen::Index numFaces() const { return m_faces.rows(); };

  /**
   * @brief Update the vertex positions of the vertices
   * @param vertices New vertex matrix of shape (N, 3), note that this function
   * does not check whether the new vertex positions are valid.
   */
  void updateVertices(Eigen::MatrixX3d &vertices) { m_vertices = vertices; };

  /**
   * @brief Reset the vertex positions to their initial state
   */
  void resetVertices() { m_vertices = m_initialVertices; };

  /**
   * @brief Getter for the color of the mesh
   * @return Color of mesh
   */
  const Eigen::RowVector3d &getColor() const { return m_color; };

  /**
   * @brief Set the color of the mesh
   * @param r Red contribution, should be in [0, 1]
   * @param g Green contribution, should be in [0, 1]
   * @param b Blue contribution, should be in [0, 1]
   */
  void updateColor(double r, double g, double b) { m_color << r, g, b; };

  /**
   * @brief Upsample the mesh via subdivision
   * @param nSubdivs number of subdivisions
   */
  void upsample(int nSubdivs);

  /**
   * @brief Tetrahedralize the mesh
   */
  void tetrahedralize();

  /**
   * @brief Build a BVH for the mesh triangles to speed up intersection tests.
   * Only works for static objects.
   */
  void buildBVH(double slack);

  /**
   * @brief Get the triangles which are candidates for possible intersections
   */
  void query(Eigen::Vector3d &q, std::vector<Eigen::Index> &triangles) const;

  /**
   * @brief Get the internal ID of the mesh (renderer ID, not libigl)
   * @return size_t index of mesh
   */
  size_t getID() { return m_ID; };

  /**
   * @brief Set the internal ID of the mesh (should be set by renderer)
   * @param id new id
   */
  void setID(size_t ID) { m_ID = ID; };

  /**
   * @brief Create a floor mesh, consisting of two triangles, which spans the
   * coordinates
   * (-100, -100) to (100, 100)
   */
  static Mesh createFloor();

  /**
   * @brief Create a mesh from an .obj file, together with an object->world
   * transformation matrix
   * @param filename Path to the .obj file
   * @param toWorld object->world transformation matrix
   */
  static Mesh createOBJ(const std::string &filename, Eigen::Affine3d &toWorld);

private:
  Eigen::MatrixX3d m_vertices;        /// Stores the vertex positions
  Eigen::MatrixX3d m_initialVertices; /// Stores the initial vertex positions
  Eigen::MatrixX3i m_faces;           /// Stores the face indices
  Eigen::MatrixX4i m_tets;            /// Stores the tet indices

  BVH *m_bvh = nullptr; /// Stores the BVH if one gets built

  Eigen::Affine3d m_toWorld =
      Eigen::Affine3d::Identity(); /// Stores the object->world transformation
                                   /// matrix

  Eigen::RowVector3d m_color =
      Eigen::RowVector3d(0.2f, 0.2f, 0.2f); ///< Mesh color

  size_t m_ID = -1; ///< ID set by the renderer, -1 means uninitialized
};