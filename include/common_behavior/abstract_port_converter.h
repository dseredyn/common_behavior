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

#ifndef COMMON_BEHAVIOR_ABSTRACT_PORT_CONVERTER_H__
#define COMMON_BEHAVIOR_ABSTRACT_PORT_CONVERTER_H__

#include "common_behavior/input_data.h"
#include "common_behavior/abstract_predicate_list.h"

#include <map>
#include <vector>
#include <string>

#include "rtt/RTT.hpp"

namespace common_behavior {

class PortConverterBase {
public:
    virtual bool isCompatible(RTT::base::PortInterface *from, RTT::base::PortInterface *to) const = 0;
    virtual bool connectPorts(RTT::base::PortInterface *from, RTT::base::PortInterface *to, const RTT::ConnPolicy &cp) = 0;
};

template <typename Tfrom, typename Tto >
class PortConverter : public PortConverterBase {
public:

    virtual bool isCompatible(RTT::base::PortInterface *from, RTT::base::PortInterface *to) const {
        if (!from || !to) {
            return false;
        }

        RTT::OutputPort<Tfrom > *port_a = dynamic_cast<RTT::OutputPort<Tfrom >* >(from);
        RTT::InputPort<Tto > *port_b = dynamic_cast<RTT::InputPort<Tto >* >(to);

        if (!port_a || !port_b) {
            return false;
        }
        return true;
    }

    virtual bool connectPorts(RTT::base::PortInterface *from, RTT::base::PortInterface *to, const RTT::ConnPolicy &cp) {
        if (!from || !to) {
            return false;
        }

        RTT::OutputPort<Tfrom > *port_a = dynamic_cast<RTT::OutputPort<Tfrom >* >(from);
        RTT::InputPort<Tto > *port_b = dynamic_cast<RTT::InputPort<Tto >* >(to);

        if (!port_a || !port_b) {
            return false;
        }

        std::string port_b_name = port_b->getName();
        RTT::DataFlowInterface *dfi_b = port_b->getInterface();

        // from -> to_in >> convert >> to_in_out -> to
        new_port.reset( new RTT::InputPort<Tfrom >(port_b_name + "_conv_in") );
        new_port_out.reset( new RTT::OutputPort<Tto >(port_b_name + "_conv_in_out") );

        dfi_b->getOwner()->ports()->addEventPort( *new_port, boost::bind( &PortConverter<Tfrom, Tto >::event, this, _1 ) );
        dfi_b->getOwner()->ports()->addLocalPort( *new_port_out);

        if (!new_port_out->connectTo(to, cp)) {
            dfi_b->getOwner()->ports()->removePort(port_b_name + "_conv_in");
            dfi_b->getOwner()->ports()->removePort(port_b_name + "_conv_in_out");
            return false;
        }
    //    if (!dc_->connect(to + "_conv_in_out", to, cp)) {
    //        RTT::log(RTT::Info) << "could not connect: " << (to + "_conv_in_out") << ", " << (to) << RTT::endlog();
    //    }

        if (!port_a->connectTo(dynamic_cast<RTT::base::PortInterface*>(new_port.get()), cp)) {
            dfi_b->getOwner()->ports()->removePort(port_b_name + "_conv_in");
            dfi_b->getOwner()->ports()->removePort(port_b_name + "_conv_in_out");
            return false;
        }

    //    if (!dc_->connect(from, to + "_conv_in", cp)) {
    //        RTT::log(RTT::Info) << "could not connect: " << from << ", " << (to + "_conv_in") << RTT::endlog();
    //    }
    //    RTT::log(RTT::Info) << "added port conversion: " << from << ", " << to << RTT::endlog();
        return true;
    }

    virtual void convert(const Tfrom&, Tto&) const = 0;

protected:

private:
    void event(RTT::base::PortInterface *pi) {
        RTT::base::PortInterface *out = pi->getInterface()->getPort(pi->getName() + "_out");

        Tfrom val;
        Tto val_out;
        dynamic_cast<RTT::InputPort<Tfrom >* >(pi)->read(val);
        convert(val, val_out);
        dynamic_cast<RTT::OutputPort<Tto >* >(out)->write(val_out);
    }

    boost::shared_ptr<RTT::InputPort<Tfrom > > new_port;
    boost::shared_ptr<RTT::OutputPort<Tto > > new_port_out;
};

class PortConverterFactory
{
private:
    std::map<std::string, function<PortConverterBase*(void)> > factoryFunctionRegistry;

    PortConverterFactory() {}

public:
    shared_ptr<PortConverterBase > Create(std::string name)
    {
        PortConverterBase * instance = nullptr;

        // find name in the registry and call factory method.
        auto it = factoryFunctionRegistry.find(name);
        if(it != factoryFunctionRegistry.end())
            instance = it->second();

        // wrap instance in a shared ptr and return
        if(instance != nullptr)
            return std::shared_ptr<PortConverterBase >(instance);
        else
            return nullptr;
    }

    void RegisterFactoryFunction(std::string name,
    function<PortConverterBase*(void)> classFactoryFunction)
    {
        // register the class factory function
        factoryFunctionRegistry[name] = classFactoryFunction;
    }

    const std::map<std::string, function<PortConverterBase*(void)> >& getPortConverters() const {
        return factoryFunctionRegistry;
    }

    static PortConverterFactory* Instance()
    {
        static PortConverterFactory factory;
        return &factory;
    }
};

template<class T>
class PortConverterRegistrar {
public:
    PortConverterRegistrar(const std::string &name)
    {
        std::cout << "PortConverterRegistrar: " << name << std::endl;
        // register the class factory function 
        PortConverterFactory::Instance()->RegisterFactoryFunction(name,
                [](void) -> PortConverterBase * { return new T();});
    }
};

#define LITERAL_registrar_port_converter_(X) registrar_port_converter_##X
#define EXPAND_registrar_port_converter_(X) LITERAL_registrar_port_converter_(X)
 
#define REGISTER_PORT_CONVERTER( CONVERTER_CLASS ) static common_behavior::PortConverterRegistrar<CONVERTER_CLASS > EXPAND_registrar_port_converter_(__LINE__)(#CONVERTER_CLASS)

};  // namespace common_behavior

#endif  // COMMON_BEHAVIOR_ABSTRACT_PORT_CONVERTER_H__

