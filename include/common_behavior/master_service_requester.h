// Copyright 2014 WUT
/*
 * robot.h
 *
 *  Created on: 26 sty 2014
 *      Author: konrad
 */

#ifndef MASTER_SERVICE_REQUESTER_H_
#define MASTER_SERVICE_REQUESTER_H_

#include "common_behavior/input_data.h"

#include <string>

#include "Eigen/Dense"

#include "rtt/RTT.hpp"

namespace common_behavior {

class MasterServiceRequester : public RTT::ServiceRequester {
 public:
  explicit MasterServiceRequester(RTT::TaskContext *owner) :
    RTT::ServiceRequester("master", owner),
    readPorts("readPorts")
{
    this->addOperationCaller(readPorts);
  }

  RTT::OperationCaller<void(InputData&)> readPorts;
};
}   // namespace common_behavior

#endif  // MASTER_SERVICE_REQUESTER_H_
