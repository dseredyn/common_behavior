/*
 Copyright (c) 2016, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef COMMON_BEHAVIOR_MASTER_SERVICE_REQUESTER_H_
#define COMMON_BEHAVIOR_MASTER_SERVICE_REQUESTER_H_

#include "common_behavior/input_data.h"
#include "common_behavior/buffer_info.h"
#include "common_behavior/abstract_behavior.h"
#include "common_behavior/abstract_predicate_list.h"

#include <string>

#include "rtt/RTT.hpp"

namespace common_behavior {

class MasterServiceRequester : public RTT::ServiceRequester {
 public:
  explicit MasterServiceRequester(RTT::TaskContext *owner)
    : RTT::ServiceRequester("master", owner)
    , initBuffersData("initBuffersData")
    , readBuffers("readBuffers")
    , writePorts("writePorts")
    , getDataSample("getDataSample")
    , getLowerInputBuffers("getLowerInputBuffers")
    , getUpperInputBuffers("getUpperInputBuffers")
    , getLowerOutputBuffers("getLowerOutputBuffers")
    , getUpperOutputBuffers("getUpperOutputBuffers")
    , getBehaviors("getBehaviors")
    , getInitialBehaviors("getInitialBehaviors")
    , getLatchedConnections("getLatchedConnections")
    , allocatePredicateList("allocatePredicateList")
    , calculatePredicates("calculatePredicates")
    , getPredicatesStr("getPredicatesStr")
    , allocateOutputScope("allocateOutputScope")
    , iterationEnd("iterationEnd")
  {
    this->addOperationCaller(initBuffersData);
    this->addOperationCaller(readBuffers);
    this->addOperationCaller(writePorts);
    this->addOperationCaller(getDataSample);

    this->addOperationCaller(getLowerInputBuffers);
    this->addOperationCaller(getUpperInputBuffers);
    this->addOperationCaller(getLowerOutputBuffers);
    this->addOperationCaller(getUpperOutputBuffers);

    this->addOperationCaller(getBehaviors);
    this->addOperationCaller(getInitialBehaviors);

    this->addOperationCaller(getLatchedConnections);

    this->addOperationCaller(allocatePredicateList);
    this->addOperationCaller(calculatePredicates);
    this->addOperationCaller(getPredicatesStr);

    this->addOperationCaller(allocateOutputScope);

    this->addOperationCaller(iterationEnd);
  }

  // OROCOS ports operations
  RTT::OperationCaller<void (InputDataPtr&)> initBuffersData;
  RTT::OperationCaller<void(InputDataPtr&)> readBuffers;
  RTT::OperationCaller<void (InputDataPtr&)> writePorts;
  RTT::OperationCaller<InputDataPtr()> getDataSample;

  // subsystem buffers
  RTT::OperationCaller<void(std::vector<InputBufferInfo >&)> getLowerInputBuffers;
  RTT::OperationCaller<void(std::vector<InputBufferInfo >&)> getUpperInputBuffers;
  RTT::OperationCaller<void(std::vector<OutputBufferInfo >&)> getLowerOutputBuffers;
  RTT::OperationCaller<void(std::vector<OutputBufferInfo >&)> getUpperOutputBuffers;

  // FSM parameters
  RTT::OperationCaller<std::vector<std::string >()> getBehaviors;
  RTT::OperationCaller<std::vector<std::string >()> getInitialBehaviors;

  RTT::OperationCaller<std::vector<std::pair<std::string, std::string > >() > getLatchedConnections;

  RTT::OperationCaller<PredicateListPtr() > allocatePredicateList;
  RTT::OperationCaller<void(const InputDataConstPtr&, const std::vector<RTT::TaskContext*>&, PredicateListPtr&) > calculatePredicates;
  RTT::OperationCaller<std::string(const PredicateListConstPtr&) > getPredicatesStr;

  RTT::OperationCaller<OutputScopeBasePtr() > allocateOutputScope;

  RTT::OperationCaller<void()> iterationEnd;
};
}   // namespace common_behavior

#endif  // COMMON_BEHAVIOR_MASTER_SERVICE_REQUESTER_H_

