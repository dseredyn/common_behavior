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

#ifndef __ABSTRACT_BEHAVIOR_H__
#define __ABSTRACT_BEHAVIOR_H__

#include "common_behavior/input_data.h"
#include "common_behavior/abstract_predicate_list.h"

#include <map>
#include <vector>
#include <string>

#include "rtt/RTT.hpp"

namespace common_behavior {

class AbstractConditionCause {
public:
    virtual bool isEqual(const AbstractConditionCause& c) const = 0;
    virtual bool orValue() const = 0;
    virtual void clear() = 0;
    virtual AbstractConditionCause& operator=(const AbstractConditionCause& arg) = 0;
};

bool operator==(const AbstractConditionCause& lhs, const AbstractConditionCause& rhs);

bool operator!=(const AbstractConditionCause& lhs, const AbstractConditionCause& rhs);

typedef boost::shared_ptr<AbstractConditionCause > AbstractConditionCausePtr;
typedef boost::shared_ptr<const AbstractConditionCause > AbstractConditionCauseConstPtr;

template <size_t N >
class ConditionCause : public AbstractConditionCause {
public:

    ConditionCause() {
        clear();
    }

    bool isEqual(const AbstractConditionCause& arg) const {
        const ConditionCause<N>* c = dynamic_cast<const ConditionCause<N>* >(&arg);
        if (!c) {
            return false;
        }
        for (int i = 0; i < N; ++i) {
            if (bitfield_[i] != c->bitfield_[i]) {
                return false;
            }
        }
        return true;
    }

    AbstractConditionCause& operator=(const AbstractConditionCause& arg) {
        const ConditionCause<N>* c = dynamic_cast<const ConditionCause<N>* >(&arg);
        if (this != c) {
            for (int i = 0; i < N; ++i) {
                bitfield_[i] = c->bitfield_[i];
            }
        }
    }

    bool orValue() const {
        for (int i = 0; i < N; ++i) {
            if (bitfield_[i]) {
                return true;
            }
        }
        return false;
    }

    void clear() {
        for (int i = 0; i < N; ++i) {
            bitfield_[i] = false;
        }
    }

    void setBit(size_t bit_idx, bool value) {
        bitfield_[bit_idx] = value;
    }

    bool getBit(int bit_idx) const {
        return bitfield_[bit_idx];
    }

private:
    boost::array<bool, N > bitfield_;
};

class OutputScopeBase {
public:
    virtual bool isCompatible(OutputScopeBase&) const = 0;

    virtual void add(OutputScopeBase&) = 0;

    virtual void substract(OutputScopeBase&) = 0;
};

typedef boost::shared_ptr<OutputScopeBase > OutputScopeBasePtr;
typedef boost::shared_ptr<const OutputScopeBase > OutputScopeBaseConstPtr;

class BehaviorBase {
public:

    virtual bool checkInitialCondition(const PredicateListConstPtr& pred_list) const = 0;
    virtual bool checkErrorCondition(const PredicateListConstPtr& pred_list) const = 0;
    virtual bool checkStopCondition(const PredicateListConstPtr& pred_list) const = 0;
    virtual OutputScopeBaseConstPtr getOutputScope() const = 0;

    bool isInitial() const {
        return is_initial_;
    }

    const std::string& getName() const {
        return name_;
    }

    const std::string& getShortName() const {
        return short_name_;
    }

    const std::vector<std::string >& getRunningComponents() const {
        return running_;
    }

    void addRunningComponent(const std::string& name) {
        running_.push_back(name);
    }

protected:
    explicit BehaviorBase(const std::string& name, const std::string& short_name)
        : name_(name)
        , short_name_(short_name)
        , is_initial_(false)
    { }

    bool is_initial_;

private:
    std::vector<std::string > running_;
    std::string name_;
    std::string short_name_;
};

class BehaviorFactory
{
private:
    map<string, function<BehaviorBase*(void)> > factoryFunctionRegistry;

    BehaviorFactory() {}

public:
    shared_ptr<BehaviorBase > Create(string name)
    {
        BehaviorBase * instance = nullptr;

        // find name in the registry and call factory method.
        auto it = factoryFunctionRegistry.find(name);
        if(it != factoryFunctionRegistry.end())
            instance = it->second();

        // wrap instance in a shared ptr and return
        if(instance != nullptr)
            return std::shared_ptr<BehaviorBase >(instance);
        else
            return nullptr;
    }

    void RegisterFactoryFunction(string name,
    function<BehaviorBase*(void)> classFactoryFunction)
    {
        // register the class factory function
        factoryFunctionRegistry[name] = classFactoryFunction;
    }

    const map<string, function<BehaviorBase*(void)> >& getBehaviors() const {
        return factoryFunctionRegistry;
    }

    static BehaviorFactory* Instance()
    {
        static BehaviorFactory factory;
        return &factory;
    }
};

template<class T>
class BehaviorRegistrar {
public:
    BehaviorRegistrar()
    {
        std::string name;
        {
            T sample;
            name = sample.getName();
        }

        std::cout << "BehaviorRegistrar: " << name << std::endl;
        // register the class factory function 
        BehaviorFactory::Instance()->RegisterFactoryFunction(name,
                [](void) -> BehaviorBase * { return new T();});
    }
};

#define LITERAL_registrar_behavior_(X) registrar_behavior_##X
#define EXPAND_registrar_behavior_(X) LITERAL_registrar_behavior_(X)
 
#define REGISTER_BEHAVIOR( STATE_CLASS ) static common_behavior::BehaviorRegistrar<STATE_CLASS > EXPAND_registrar_behavior_(__LINE__)

};  // namespace common_behavior

#endif  // __ABSTRACT_BEHAVIOR_H__

