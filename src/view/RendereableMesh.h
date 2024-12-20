/**
 * @file RendereableMesh.h
 * @brief Header only data structure to store a renderable mesh for the
 * renderer.
 */

#pragma once

#include "../model/Mesh.h"
#include <Eigen/Core>

/**
 * @enum ShaderType
 * @brief Enum of different mesh types, currently consisting of Static and
 * Dynamic. This is used to find out which shader should be loaded to render
 * this object.
 */
enum class ShaderType {
  Static,
  Dynamic,
};

/**
 * @struct Renderable
 * @brief Wrapper to store renderable data with vertices, fragments and color
 * used in the renderer.
 */
struct Renderable {
  const ShaderType type;
  Eigen::MatrixXd m_vertices; ///< Vertex matrix of shape (N x 3)
  Eigen::MatrixXi m_faces;    ///< Face matrix of shape (N x 3)
  Eigen::RowVector3d m_color; ///< Color vector

  int m_iglViewerID; ///< Mesh id of libigl, to access the element in libigl

  /**
   * @brief Construct a new Renderable object
   * @param mesh The mesh which should be converted into a renderable
   * @param type The shader type which defines which shader should be loaded for
   * it
   */
  Renderable(Mesh &mesh, const ShaderType type) : type(type) {
    m_vertices = mesh.getVertices();
    m_faces = mesh.getFaces();
    m_color = mesh.getColor();
    m_iglViewerID = -1;
  };
};