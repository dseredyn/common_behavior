/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
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

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>
#include <rtt/extras/SlaveActivity.hpp>
#include <rtt/os/main.h>

#include <math.h>

#include <vector>
#include <string>

#include "common_behavior/abstract_behavior.h"
#include "common_behavior/abstract_state.h"
#include "common_behavior/input_data.h"
#include "common_behavior/master_service_requester.h"
#include "common_behavior/master_service.h"

using namespace RTT;

class DiagStateSwitch {
public:
    enum Reason {INVALID, INIT, STOP, ERROR};
    int id_;
    RTT::os::TimeService::nsecs time_;
    Reason reason_;
    common_behavior::PredicateListPtr pred_;

    static const std::string& getReasonStr(Reason r) {
        static const std::string inv("INV");
        static const std::string init("INIT");
        static const std::string stop("STOP");
        static const std::string err("ERR");
        switch (r) {
        case INIT:
            return init;
        case STOP:
            return stop;
        case ERROR:
            return err;
        }
        return inv;
    }

    const std::string& getReasonStr() const {
        return getReasonStr(reason_);
    }
};

class DiagStateSwitchHistory {
public:
    DiagStateSwitchHistory()
        : idx_(0)
    {}

    void addStateSwitch(int new_state_id, RTT::os::TimeService::nsecs time, DiagStateSwitch::Reason reason, common_behavior::PredicateListConstPtr pred = common_behavior::PredicateListConstPtr()) {
        if (h_.size() == 0) {
            return;
        }
        h_[idx_].id_ = new_state_id;
        h_[idx_].time_ = time;
        h_[idx_].reason_ = reason;
        if (pred) {
            *(h_[idx_].pred_) = *pred;
        }
        idx_ = (idx_+1) % h_.size();
    }

    void setSize(size_t size, RTT::OperationCaller<common_behavior::PredicateListPtr()> common_behavior::MasterServiceRequester::*func, common_behavior::MasterServiceRequester& a) {
        h_.resize(size);
        for (int i = 0; i < h_.size(); ++i) {
            h_[i].id_ = -1;
            h_[i].reason_ = DiagStateSwitch::INVALID;
            h_[i].pred_ = (a.*func)();
        }
        idx_ = 0;
    }

    bool getStateSwitchHistory(int idx, DiagStateSwitch &ss) const {
        if (idx >= h_.size() || h_.size() == 0) {
            return false;
        }
        int i = (idx_-idx-1+h_.size()*2) % h_.size();
        if (h_[i].reason_ == DiagStateSwitch::INVALID) {
            return false;
        }
        ss = h_[i];
        return true;
    }

private:

    std::vector<DiagStateSwitch> h_;
    int idx_;
};

class MasterComponent: public RTT::TaskContext {
public:
    explicit MasterComponent(const std::string &name);

    bool configureHook();

    bool startHook();

    void stopHook();

    void updateHook();

    std::string getDiag();

    bool addConmanScheme(RTT::TaskContext* scheme);

private:
    // parameters
    std::vector<std::string > state_names_;
    std::string initial_state_name_;

    std::vector<std::string > behavior_names_;

    RTT::OutputPort<uint32_t> port_status_test_out_;

    std::vector<std::shared_ptr<common_behavior::BehaviorBase > > behaviors_;

    std::vector<std::shared_ptr<common_behavior::StateBase > > states_;
    std::shared_ptr<common_behavior::StateBase > current_state_;

    // pointer to conman scheme TaskContext
    TaskContext *scheme_;

    // conman scheme operations
    RTT::OperationCaller<bool(const std::string &)> hasBlock_;
    RTT::OperationCaller<bool(int, const std::vector<std::string>&, const std::vector<std::string>&)> addGraphConfiguration_;
    RTT::OperationCaller<bool(int)> switchToConfiguration_;

    std::vector<TaskContext* > scheme_peers_;
    std::vector<std::vector<bool> > is_running_;

    bool state_switch_;

    RTT::base::DataObjectLockFree<DiagStateSwitchHistory > diag_ss_sync_;
    DiagStateSwitchHistory diag_ss_rt_;

    boost::shared_ptr<common_behavior::MasterServiceRequester > master_service_;

    boost::shared_ptr<common_behavior::InputData > in_data_;

    common_behavior::AbstractConditionCausePtr error_condition_;

    common_behavior::PredicateListPtr predicate_list_;

    RTT::Seconds last_exec_time_, last_exec_period_;
    RTT::os::TimeService::nsecs last_update_time_;

    int state_switch_history_length_;
};

MasterComponent::MasterComponent(const std::string &name)
    : TaskContext(name, PreOperational)
    , state_switch_history_length_(5)
{
    this->ports()->addPort("status_test_OUTPORT", port_status_test_out_);

    this->addOperation("getDiag", &MasterComponent::getDiag, this, RTT::ClientThread);

    this->addOperation("addConmanScheme", &MasterComponent::addConmanScheme, this, RTT::ClientThread);

    addProperty("state_switch_history_length", state_switch_history_length_);
}

std::string MasterComponent::getDiag() {
    std::ostringstream strs;

    strs << "<mcd>";
    strs << "<h>";

    DiagStateSwitchHistory ss;
    diag_ss_sync_.Get(ss);


    for (int i = 0; ; ++i) {
        DiagStateSwitch s;
        if (!ss.getStateSwitchHistory(i, s)) {
            break;
        }
        RTT::os::TimeService::Seconds switch_interval = RTT::nsecs_to_Seconds(last_update_time_ - s.time_);

        std::string err_str;
        if (s.pred_) {
            err_str = master_service_->getPredicatesStr(s.pred_);
        }
        std::string state_name;
        if (s.id_ >= 0) {
            state_name = states_[s.id_]->getShortStateName();
        }
        else {
            state_name = "INV_STATE";
        }
        strs << "<ss n=\"" << state_name << "\" r=\""
             << s.getReasonStr() << "\" t=\"" << switch_interval << "\" e=\""
             << err_str << "\" />";
    }

    strs << "</h>";

    strs << "<p>" << last_exec_period_ << "</p>";
    strs << "</mcd>";

    return strs.str();
}

bool MasterComponent::addConmanScheme(RTT::TaskContext* scheme) {
    scheme_ = scheme;
    return scheme_->setActivity( new RTT::extras::SlaveActivity(this->getActivity(), scheme_->engine()));
}

bool MasterComponent::configureHook() {
    Logger::In in("MasterComponent::configureHook");

    master_service_ = this->getProvider<common_behavior::MasterServiceRequester >("master");
    if (!master_service_) {
        RTT::log(RTT::Error) << "Unable to load common_behavior::MasterService" << RTT::endlog();
        return false;
    }

    predicate_list_ = master_service_->allocatePredicateList();
    if (!predicate_list_) {
        Logger::log() << Logger::Error << "could not allocate predicate list" << Logger::endl;
        return false;
    }

    state_names_ = master_service_->getStates();
    initial_state_name_ = master_service_->getInitialState();

    for (auto it = common_behavior::StateFactory::Instance()->getStates().begin();
        it != common_behavior::StateFactory::Instance()->getStates().end(); ++it)
    {
        Logger::log() << Logger::Info << "state: " << it->first << Logger::endl;
    }

    // retrieve states list
    for (int i = 0; i < state_names_.size(); ++i) {
        auto b_ptr = common_behavior::StateFactory::Instance()->Create( state_names_[i] );
        if (b_ptr) {
            states_.push_back(b_ptr);
        }
        else {
            Logger::log() << Logger::Error << "unknown state: " << state_names_[i] << Logger::endl;
            return false;
        }
    }

    // retrieve behavior names
    for (int i = 0; i < states_.size(); ++i) {
        const std::string& behavior_name = states_[i]->getBehaviorName();
        bool add = true;
        for (int j = 0; j < behavior_names_.size(); ++j) {
            if (behavior_names_[j] == behavior_name) {
                add = false;
                break;
            }
        }
        if (add) {
            behavior_names_.push_back(behavior_name);
        }
    }

    // retrieve behaviors list
    for (int i = 0; i < behavior_names_.size(); ++i) {
        auto b_ptr = common_behavior::BehaviorFactory::Instance()->Create( behavior_names_[i] );
        if (b_ptr) {
            behaviors_.push_back(b_ptr);
        }
        else {
            Logger::log() << Logger::Error << "unknown behavior: " << behavior_names_[i] << Logger::endl;
            return false;
        }
    }

    diag_ss_rt_.setSize(state_switch_history_length_, &common_behavior::MasterServiceRequester::allocatePredicateList, *master_service_);

    // select initial state
    for (int i = 0; i < states_.size(); ++i) {
        if (states_[i]->getStateName() == initial_state_name_) {
            current_state_ = states_[i];
            diag_ss_rt_.addStateSwitch(i, RTT::os::TimeService::Instance()->getNSecs(), DiagStateSwitch::INIT);
        }
    }

    diag_ss_sync_.data_sample(diag_ss_rt_);
    diag_ss_sync_.Set(diag_ss_rt_);

    if (!current_state_) {
        Logger::log() << Logger::Error << "unknown initial state: " << initial_state_name_ << Logger::endl;
        return false;
    }

    // get names of all components that are needed for all behaviors
    std::set<std::string > switchable_components;

    for (int i = 0; i < behaviors_.size(); ++i) {
        const std::vector<std::string >& comp_vec = behaviors_[i]->getRunningComponents();
        for (int j = 0; j < comp_vec.size(); ++j) {
            switchable_components.insert( comp_vec[j] );
        }
    }

    std::string switchable_components_str;
    for (std::set<std::string >::const_iterator it = switchable_components.begin(); it != switchable_components.end(); ++it) {
        switchable_components_str = switchable_components_str + (switchable_components_str.empty()?"":", ") + (*it);
    }
    Logger::log() << Logger::Info << "switchable components: " << switchable_components_str << Logger::endl;

    RTT::OperationInterfacePart *hasBlockOp = scheme_->getOperation("hasBlock");
    if (hasBlockOp == NULL) {
        Logger::log() << Logger::Error << "the peer " << scheme_->getName() << " has no matching operation hasBlock" << Logger::endl;
        return false;
    }

    hasBlock_ =  RTT::OperationCaller<bool(const std::string &)>(
        hasBlockOp, scheme_->engine());


    for (std::set<std::string >::const_iterator it = switchable_components.begin(); it != switchable_components.end(); ++it) {
        if (!hasBlock_( *it )) {
            Logger::log() << Logger::Error << "could not find a component \'" << (*it) << "\' in the scheme blocks list" << Logger::endl;
            return false;
        }
    }

    RTT::OperationInterfacePart *addGraphConfigurationOp = scheme_->getOperation("addGraphConfiguration");
    if (addGraphConfigurationOp == NULL) {
        Logger::log() << Logger::Error << "the peer " << scheme_->getName() << " has no matching operation addGraphConfiguration" << Logger::endl;
        return false;
    }

    addGraphConfiguration_ = RTT::OperationCaller<bool(int, const std::vector<std::string>&, const std::vector<std::string>&)>(
        addGraphConfigurationOp, scheme_->engine());

    RTT::OperationInterfacePart *switchToConfigurationOp = scheme_->getOperation("switchToConfiguration");
    if (switchToConfigurationOp == NULL) {
        Logger::log() << Logger::Error << "the peer " << scheme_->getName() << " has no matching operation switchToConfiguration" << Logger::endl;
        return false;
    }

    switchToConfiguration_ = RTT::OperationCaller<bool(int)>(
        switchToConfigurationOp, scheme_->engine());

    // add graph configuration for each behavior
    for (int i = 0; i < behaviors_.size(); ++i) {
        std::vector<std::string > vec_stopped;
        const std::vector<std::string >& vec_running = behaviors_[i]->getRunningComponents();
        for (std::set<std::string >::const_iterator ic = switchable_components.begin(); ic != switchable_components.end(); ++ic) {
            bool is_running = false;
            for (int ir = 0; ir < vec_running.size(); ++ir) {
                if ( (*ic) == vec_running[ir] ) {
                    is_running = true;
                    break;
                }
            }
            if (!is_running) {
                vec_stopped.push_back( *ic );
            }
        }
        addGraphConfiguration_(i, vec_stopped, vec_running);
    }

    // retrieve the vector of peers of conman scheme
    TaskContext::PeerList scheme_peers_names = scheme_->getPeerList();
    for (int pi = 0; pi < scheme_peers_names.size(); ++pi) {
        scheme_peers_.push_back( scheme_->getPeer(scheme_peers_names[pi]) );
    }

    in_data_ = master_service_->getDataSample();
    if (!in_data_) {
        RTT::log(RTT::Error) << "Unable to get InputData sample" << RTT::endlog();
        return false;
    }

    error_condition_ = master_service_->getErrorReasonSample();
    if (!error_condition_) {
        RTT::log(RTT::Warning) << "Error reason sample was set to NULL. Error condition diagnostics is disabled." << RTT::endlog();
    }

    return true;
}

bool MasterComponent::startHook() {
    state_switch_ = true;

    master_service_->initBuffers(in_data_);

    return true;
}

void MasterComponent::stopHook() {
}

void MasterComponent::updateHook() {

    // What time is it
    RTT::os::TimeService::nsecs now = RTT::os::TimeService::Instance()->getNSecs();
    RTT::os::TimeService::Seconds
        time = RTT::nsecs_to_Seconds(now),
        period = RTT::nsecs_to_Seconds(RTT::os::TimeService::Instance()->getNSecs(last_update_time_));
    
    // Store update time
    last_update_time_ = now;
      
    // Compute statistics describing how often update is being called
    last_exec_period_ = time - last_exec_time_;
    last_exec_time_ = time;

    master_service_->initBuffers(in_data_);
    master_service_->readIpcPorts(in_data_);
    master_service_->readInternalPorts(in_data_);

    master_service_->writePorts(in_data_);

    // get current behavior
    std::shared_ptr<common_behavior::BehaviorBase > current_behavior;
    for (int i = 0; i < behaviors_.size(); ++i) {
        if (current_state_->getBehaviorName() == behaviors_[i]->getName()) {
            current_behavior = behaviors_[i];
            break;
        }
    }

    master_service_->calculatePredicates(in_data_, scheme_peers_, current_state_->getStateName(), predicate_list_);

    //
    // check error condition
    //

    if (error_condition_) {
        error_condition_->clear();
    }

    bool pred_err = current_behavior->checkErrorCondition(predicate_list_);
    predicate_list_->IN_ERROR = pred_err;

    if (pred_err) {
        int next_state_index = -1;
        for (int i = 0; i < states_.size(); ++i) {
            if ( states_[i]->checkInitialCondition(predicate_list_) ) {
                if (next_state_index == -1) {
                    next_state_index = i;
                }
                else {
                    Logger::In in("MasterComponent::updateHook");
                    Logger::log() << Logger::Error << "two or more states have the same initial condition (err): current_state="
                        << current_state_->getStateName()
                        << ", states: " << states_[i]->getStateName() << ", " << states_[next_state_index]->getStateName() << ", predicates: " << master_service_->getPredicatesStr(predicate_list_)
                        << Logger::endl;
                    diag_ss_rt_.addStateSwitch(-1, now, DiagStateSwitch::ERROR, predicate_list_);
                    error();
                    return;
                }
            }
        }
        if (next_state_index == -1) {
            Logger::In in("MasterComponent::updateHook");
            Logger::log() << Logger::Error << "cannot switch to new state (initial condition, err): current_state="
                << current_state_->getStateName() << ", predicates: " << master_service_->getPredicatesStr(predicate_list_) << Logger::endl;
            diag_ss_rt_.addStateSwitch(-1, now, DiagStateSwitch::ERROR, predicate_list_);
            error();
            return;
        }
        else {
                Logger::log() << Logger::Info << "state_switch (error) from "
                    << current_state_->getStateName()
                    << " to " << states_[next_state_index]->getStateName()
                    << Logger::endl;

            current_state_ = states_[next_state_index];
            diag_ss_rt_.addStateSwitch(next_state_index, now, DiagStateSwitch::ERROR, predicate_list_);
            diag_ss_sync_.Set(diag_ss_rt_);

            state_switch_ = true;
        }
    }
    else {
        //
        // check stop condition
        //
        bool pred_stop = current_behavior->checkStopCondition(predicate_list_);

        if (pred_stop) {
            int next_state_index = -1;
            for (int i = 0; i < states_.size(); ++i) {
                if ( states_[i]->checkInitialCondition(predicate_list_) ) {
                    if (next_state_index == -1) {
                        next_state_index = i;
                    }
                    else {
                        Logger::In in("MasterComponent::updateHook");
                        Logger::log() << Logger::Error << "two or more states have the same initial condition (stop): current_state="
                            << current_state_->getStateName()
                            << ", states: " << states_[i]->getStateName() << ", " << states_[next_state_index]->getStateName() << ", predicates: " << master_service_->getPredicatesStr(predicate_list_)
                            << Logger::endl;
                        diag_ss_rt_.addStateSwitch(-1, now, DiagStateSwitch::STOP, predicate_list_);
                        error();
                        return;
                    }
                }
            }
            if (next_state_index == -1) {
                Logger::In in("MasterComponent::updateHook");
                Logger::log() << Logger::Error << "cannot switch to new state (initial condition, stop): current_state="
                    << current_state_->getStateName() << ", predicates: " << master_service_->getPredicatesStr(predicate_list_) << Logger::endl;
                diag_ss_rt_.addStateSwitch(-1, now, DiagStateSwitch::STOP, predicate_list_);
                error();
                return;
            }
            else {
                Logger::log() << Logger::Info << "state_switch (stop) from "
                    << current_state_->getStateName()
                    << " to " << states_[next_state_index]->getStateName()
                    << Logger::endl;

                current_state_ = states_[next_state_index];
                diag_ss_rt_.addStateSwitch(next_state_index, now, DiagStateSwitch::STOP, predicate_list_);
                diag_ss_sync_.Set(diag_ss_rt_);

                state_switch_ = true;
            }
        }
    }

    //
    // if the state has changed, reorganize the graph
    //
    if (state_switch_) {
        const std::string& behavior_name = current_state_->getBehaviorName();

        for (int i = 0; i < behaviors_.size(); ++i) {
            if (behaviors_[i]->getName() == behavior_name) {
                switchToConfiguration_(i);
                break;
            }
        }
        state_switch_ = false;
    }

    if (scheme_->getTaskState() != RTT::TaskContext::Running) {
        RTT::log(RTT::Error) << "Component is not in the running state: " << scheme_->getName() << RTT::endlog();
        error();
        return;
    }
    scheme_->update();

    // iterationEnd callback can be used by e.g. Gazebo simulator
    master_service_->iterationEnd();
/*
// TODO: determine if this is needed here
    master_service_->initBuffers(in_data_);
    if (!master_service_->readStatusPorts(in_data_)) {
        error();
    }
    master_service_->writeStatusPorts(in_data_);
*/
}

ORO_LIST_COMPONENT_TYPE(MasterComponent)

ORO_CREATE_COMPONENT_LIBRARY()

