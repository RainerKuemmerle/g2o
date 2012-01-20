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

#ifndef G2O_GRAPH_PARAMETER_HH_
#define G2O_GRAPH_PARAMETER_HH_

#include <iosfwd>

#include "hyper_graph.h"

namespace g2o {

    class G2O_CORE_API Parameter : public HyperGraph::HyperGraphElement
    {
      public:
        Parameter();
        virtual ~Parameter() {};
        //! read the data from a stream
        virtual bool read(std::istream& is) = 0;
        //! write the data to a stream
        virtual bool write(std::ostream& os) const = 0;
        int id() const {return _id;}
        void setId(int id_);
        virtual HyperGraph::HyperGraphElementType elementType() const { return HyperGraph::HGET_PARAMETER;}
      protected:
        int _id;
    };

    typedef std::vector<Parameter*> ParameterVector;

} // end namespace

#endif
