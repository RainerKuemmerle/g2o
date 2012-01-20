#ifndef G2O_DATA_QUEUE_H
#define G2O_DATA_QUEUE_H

#include <map>
#include "g2o_types_data_api.h"

namespace g2o {

  class RobotData;

  /**
   * \brief a simple queue to store data and retrieve based on a timestamp
   */
  class G2O_TYPES_DATA_API DataQueue
  {
    public:
      typedef std::map<double, RobotData*>           Buffer;

    public:
      DataQueue();
      ~DataQueue();

      void add(RobotData* rd);

      RobotData* findClosestData(double timestamp) const;

      RobotData* before(double timestamp) const;
      RobotData* after(double timestamp) const;

      const Buffer& buffer() const {return _buffer;}

    protected:
      Buffer _buffer;
  };

} // end namespace

#endif
