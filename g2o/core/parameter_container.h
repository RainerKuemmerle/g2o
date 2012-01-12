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

#ifndef G2O_GRAPH_PARAMETER_CONTAINER_HH_
#define G2O_GRAPH_PARAMETER_CONTAINER_HH_

#include <iosfwd>
#include <map>

namespace g2o {
    
    class Parameter;

    /**
     * \brief map id to parameters
     */
    class ParameterContainer : protected std::map<int, Parameter*> 
    {
    public:
      typedef std::map<int, Parameter*> BaseClass;

      /**
       * create a container for the parameters.
       * @param isMainStorage_ pointers to the parameters are owned by this container, i.e., freed in its constructor
       */
      ParameterContainer(bool isMainStorage_=true);
      virtual ~ParameterContainer();
      //! add parameter to the container
      bool addParameter(Parameter* p);
      //! return a parameter based on its ID
      Parameter* getParameter(int id);
      //! remove a parameter from the container, i.e., the user now owns the pointer
      Parameter* detachParameter(int id);
      //! read parameters from a stream
      virtual bool read(std::istream& is);
      //! write the data to a stream
      virtual bool write(std::ostream& os) const;
      bool isMainStorage() const {return _isMainStorage;}
      void clear();

      // stuff of the base class that should re-appear
      using BaseClass::size;

    protected:
      bool _isMainStorage;
    };

} // end namespace

#endif
