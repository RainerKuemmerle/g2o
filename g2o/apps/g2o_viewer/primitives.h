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

#ifndef G2O_PRIMITIVES_H
#define G2O_PRIMITIVES_H

/** @addtogroup viewer libviewer **/
// @{

/** \file primitives.h
 * \brief draw primitives with OpenGL
 */

#include "g2o_viewer_api.h"
#include <qgl.h>

namespace g2o {

/**
 * draw a box that is centered in the current coordinate frame
 * @param l length of the box (x dimension)
 * @param w width of the box (y dimension)
 * @param h height of the box (z dimension)
 */
void G2O_VIEWER_API drawBox(GLfloat l, GLfloat w, GLfloat h);

/**
 * draw a plane in x-y dimension with a height of zero
 * @param l length in x
 * @param w width in y
 */
void G2O_VIEWER_API drawPlane(GLfloat l, GLfloat w);

/**
 * draw a sphere whose center is in the origin of the current coordinate frame
 * @param radius the radius of the sphere
 */
void G2O_VIEWER_API drawSphere(GLfloat radius);

/**
 * draw a ellipsoid whose center is in the origin of the current coordinate frame
 * @param r1 radius along x axis
 * @param r2 radius along y axis
 * @param r3 radius along z axis
 */
void G2O_VIEWER_API drawEllipsoid(GLfloat r1, GLfloat r2, GLfloat r3);

/**
 * draw a cone
 */
void G2O_VIEWER_API drawCone(GLfloat radius, GLfloat height);

/**
 * draw a disk
 */
void G2O_VIEWER_API drawDisk(GLfloat radius);

/**
 * draw a (closed) cylinder
 * @param radius the radius of the cylinder
 * @param height the height of the cylinder
 */
void G2O_VIEWER_API drawCylinder(GLfloat radius, GLfloat height);

/**
 * draw a pyramid
 */
void G2O_VIEWER_API drawPyramid(GLfloat length, GLfloat height);

/**
 * draw a range ring
 * @param range the range (radius) of the partial ring
 * @param fov Field Of View of the range sensor
 * @param range_width specify how thick the ring should be drawn
 */
void G2O_VIEWER_API drawRangeRing(GLfloat range, GLfloat fov, GLfloat range_width = 0.05);

/**
 * draw a slice of a cylinder (approximated with slices_per_circle triangles for the complete circle)
 * @param radius the radius of the cylinder
 * @param height the height of the cylinder
 * @param fov the "fov" of the slice (om degree)
 * @param slices_per_circle the number of triangle used to approximate the fulle circle
 */
void G2O_VIEWER_API drawSlice(GLfloat radius, GLfloat height, GLfloat fov, int slices_per_circle = 32);

/**
 * draws a box used to represent a 6d pose
 */
void G2O_VIEWER_API drawPoseBox();

/**
 * Draws a 3D arrow along the positive Z axis.
 */
void G2O_VIEWER_API drawArrow(float length=1.0f, float radius=-1.0f, int nbSubdivisions=12);

/**
 * draw a 2D arrow along the x axis with the given len
 */
void G2O_VIEWER_API drawArrow2D(float len, float head_width, float head_len);

/**
 * Draws an XYZ axis, with a given len (default is 1.0).
 * 
 * The axis position and orientation matches the current modelView matrix state: three arrows (red,
 * green and blue) of length \p length are drawn along the positive X, Y and Z directions.
 */
void G2O_VIEWER_API drawAxis(float length = 1.f);

/**
 * Draws a grid in the XY plane, centered on (0,0,0) (defined in the current coordinate system).
 */
void G2O_VIEWER_API drawGrid(float size=1.0f, int nbSubdivisions=10);

// @}

} // end namespace

#endif
