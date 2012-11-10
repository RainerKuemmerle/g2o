// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// This file is part of g2o.
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with g2o.  If not, see <http://www.gnu.org/licenses/>.

#ifndef G2O_OPENGL_PRIMITIVES_H
#define G2O_OPENGL_PRIMITIVES_H

/** @addtogroup viewer libviewer **/
// @{

/** \file primitives.h
 * \brief draw primitives with OpenGL
 */

#include "opengl_wrapper.h"

#include "g2o/config.h"

#ifdef _MSC_VER
#  ifdef G2O_SHARED_LIBS
#    ifdef opengl_helper_EXPORTS
#      define G2O_OPENGL_API __declspec(dllexport)
#    else
#      define G2O_OPENGL_API __declspec(dllimport)
#    endif
#  else
#    define G2O_OPENGL_API
#  endif
#else
#  define G2O_OPENGL_API
#endif

namespace g2o {
namespace opengl {

/**
 * draw a box that is centered in the current coordinate frame
 * @param l length of the box (x dimension)
 * @param w width of the box (y dimension)
 * @param h height of the box (z dimension)
 */
void G2O_OPENGL_API drawBox(GLfloat l, GLfloat w, GLfloat h);

/**
 * draw a plane in x-y dimension with a height of zero
 * @param l length in x
 * @param w width in y
 */
void G2O_OPENGL_API drawPlane(GLfloat l, GLfloat w);

/**
 * draw a sphere whose center is in the origin of the current coordinate frame
 * @param radius the radius of the sphere
 */
void G2O_OPENGL_API drawSphere(GLfloat radius);

/**
 * draw a ellipsoid whose center is in the origin of the current coordinate frame
 * @param r1 radius along x axis
 * @param r2 radius along y axis
 * @param r3 radius along z axis
 */
void G2O_OPENGL_API drawEllipsoid(GLfloat r1, GLfloat r2, GLfloat r3);

/**
 * draw a cone
 */
void G2O_OPENGL_API drawCone(GLfloat radius, GLfloat height);

/**
 * draw a disk
 */
void G2O_OPENGL_API drawDisk(GLfloat radius);

/**
 * draw a (closed) cylinder
 * @param radius the radius of the cylinder
 * @param height the height of the cylinder
 */
void G2O_OPENGL_API drawCylinder(GLfloat radius, GLfloat height);

/**
 * draw a pyramid
 */
void G2O_OPENGL_API drawPyramid(GLfloat length, GLfloat height);

/**
 * draw a range ring
 * @param range the range (radius) of the partial ring
 * @param fov Field Of View of the range sensor
 * @param range_width specify how thick the ring should be drawn
 */
void G2O_OPENGL_API drawRangeRing(GLfloat range, GLfloat fov, GLfloat range_width = 0.05);

/**
 * draw a slice of a cylinder (approximated with slices_per_circle triangles for the complete circle)
 * @param radius the radius of the cylinder
 * @param height the height of the cylinder
 * @param fov the "fov" of the slice (om degree)
 * @param slices_per_circle the number of triangle used to approximate the fulle circle
 */
void G2O_OPENGL_API drawSlice(GLfloat radius, GLfloat height, GLfloat fov, int slices_per_circle = 32);

/**
 * draws a box used to represent a 6d pose
 */
void G2O_OPENGL_API drawPoseBox();

/**
 * draw a 2D arrow along the x axis with the given len
 */
void G2O_OPENGL_API drawArrow2D(float len, float head_width, float head_len);

/**
 * draw a point in the origin, having a size of pointSize
 */
void G2O_OPENGL_API drawPoint(float  pointSize);


// @}

#define POSE_VERTEX_COLOR 0.5f,0.5f,0.8f
#define POSE_PARAMETER_COLOR 0.5f,0.5f,0.8f
#define POSE_EDGE_COLOR 0.4f,0.4f,0.7f
#define POSE_EDGE_GHOST_COLOR 0.4f,0.4f,0.7f

#define LANDMARK_VERTEX_COLOR 0.8f,0.5f,0.3f
#define LANDMARK_EDGE_COLOR   0.7f,0.4f,0.2f
#define LANDMARK_EDGE_GHOST_COLOR   0.7f,0.4f,0.2f


} // end namespace
} // end namespace

#endif
