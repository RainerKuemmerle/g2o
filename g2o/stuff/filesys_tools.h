// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/***************************************************************************
 *            filesysTools.h
 *
 *  Fr 02 Mär 2007 23:14:21 CET
 *  Copyright 2007 Rainer Kümmerle
 *  Email rk@raikue.net
 ****************************************************************************/

#ifndef G2O_FILESYS_TOOLS_H
#define G2O_FILESYS_TOOLS_H


/** @addtogroup utils **/
// @{

/** \file filesysTools.h
 * \brief utility functions for handling files, directory on Linux/Unix
 */

#include <string>
#include <vector>

namespace g2o {

/**
 * get filename extension (the part after the last .), e.g.
 * the extension of file.txt is txt
 */
std::string getFileExtension(const std::string& filename);

/**
 * get the filename without the extension.
 * file.txt -> file
 */
std::string getPureFilename(const std::string& filename);

/**
 * change the fileextension of a given filename.
 * Only if filename contains an extension, otherwise filename is returned.
 */
std::string changeFileExtension(const std::string& filename, const std::string& newExt, bool stripDot = false);

/**
 * return the basename of the given filename
 * /etc/fstab -> fstab
 */
std::string getBasename(const std::string& filename);

/**
 * return the directory of a given filename
 * /etc/fstab -> /etc
 */
std::string getDirname(const std::string& filename);

/**
 * check if file exists (note a directory is also a file)
 */
bool fileExists(const char* filename);

/**
 * return all files that match a given pattern, e.g., ~/blaa*.txt, /tmp/a*
 */
std::vector<std::string> getFilesByPattern(const char* pattern);

} // end namespace
// @}
#endif
