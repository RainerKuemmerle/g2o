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

#ifndef G2O_DL_WRAPPER_H
#define G2O_DL_WRAPPER_H

#include <vector>
#include <string>

#ifdef WINDOWS
#include <windows.h>
#endif

namespace g2o {

  /**
   * \brief Loading libraries during run-time
   */
  class DlWrapper
  {
    public:
      DlWrapper();
      virtual ~DlWrapper();

      /**
       * open all libs from a directory matching a specific pattern.
       * @return number of loaded libs
       */
      int openLibraries(const std::string& directory, const std::string& pattern = "");

      /**
       * open a specific library
       */
      bool openLibrary(const std::string& filename);

      /**
       * free all loaded libs, i.e., call dlclose()
       */
      void clear();

    protected:
# if defined (UNIX) || defined(CYGWIN)
      std::vector<void*> _handles;
#     elif defined (WINDOWS)
      std::vector<HMODULE> _handles;
#     endif
      std::vector<std::string> _filenames;

    private:
      /**
       * it's not allowed to draw a copy of the wrapper
       */
      DlWrapper(const DlWrapper& );
      DlWrapper& operator=(const DlWrapper& );
  };

}

#endif
