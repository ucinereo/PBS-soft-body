/**
 * @file Mesh.h
 * @brief Mesh data structure wrapper for libigl
 */
#pragma once

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
   * @brief Get a copy of the vertex matrix.
   * @return const Eigen::MatrixXd non-modifiable vertex matrix
   */
  const Eigen::MatrixX3d &getVertices() const { return m_vertices; };

  /**
   * @brief Get a copy of the vertex matrix.
   * @return const Eigen::MatrixXd non-modifiable vertex matrix
   */
  const Eigen::MatrixX3d &getInitialVertices() const {
    return m_initialVertices;
  };

  /**
   * @brief Get a copy of the face matrix.
   * @return const Eigen::MatrixXi non-modifiable face matrix
   */
  const Eigen::MatrixX3i &getFaces() const { return m_faces; };

  /**
   * @brief Get the tetrahedra indices (matrix of shape (n x 4))
   * @return Tet index matrix
   */
  const Eigen::MatrixX4i &getTets() const { return m_tets; };

  /**
   * @brief Get the number of faces.
   * @return const Eigen::Index number of faces
   */
  Eigen::Index numFaces() const { return m_faces.rows(); };

  /**
   * @brief Update the vertex positions of the vertices
   * @param vertices New vertex matrix of shape (N, 3), note that this function
   * does not check wether the new vertex positions are valid.
   */
  void updateVertices(Eigen::MatrixX3d &vertices) { m_vertices = vertices; };

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

  void upsample(int nSubdivs);

  void tetrahedralize();

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

  static Mesh createCube(Eigen::Affine3d &toWorld) {
    Eigen::MatrixX3d V;
    Eigen::MatrixX3i F;

    igl::readOBJ("../assets/cube_1x.obj", V, F);

    return Mesh(V, F, toWorld);
  }

  static Mesh createDuck(Eigen::Affine3d &toWorld) {
    Eigen::MatrixX3d V;
    Eigen::MatrixX3i F;

    igl::readOBJ("../assets/rubber_duck.obj", V, F);

    return Mesh(V, F, toWorld);
  }

private:
  // @TODO: Currently stores the vertex positions directly, might be smart to
  // add an origin position just like in the PBS example projects.
  Eigen::MatrixX3d m_vertices;
  Eigen::MatrixX3d m_initialVertices;
  Eigen::MatrixX3i m_faces;
  Eigen::MatrixX4i m_tets;

  Eigen::Affine3d m_toWorld = Eigen::Affine3d::Identity();

  // @TODO: Update single color vector with color matrix (see libigl)
  Eigen::RowVector3d m_color =
      Eigen::RowVector3d(0.2f, 0.2f, 0.2f); ///< Mesh color

  size_t m_ID = -1; ///< ID set by the renderer, -1 means uninitialized
};