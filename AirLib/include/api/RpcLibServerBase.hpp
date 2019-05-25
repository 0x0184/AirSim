// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RpcLibServerBase_hpp
#define air_RpcLibServerBase_hpp

#include "common/Common.hpp"
#include "api/ApiServerBase.hpp"
#include "api/ApiProvider.hpp"

#include "api/RpcLibPort.hpp"

const extern uint16_t RpcLibPort;

namespace msr { namespace airlib {


class RpcLibServerBase : public ApiServerBase {
public:
<<<<<<< HEAD
    RpcLibServerBase(ApiProvider* api_provider, const std::string& server_address, uint16_t port = 39311);
=======
    RpcLibServerBase(ApiProvider* api_provider, const std::string& server_address, uint16_t port = RpcLibPort);
>>>>>>> 75a8f5bcf5d8408626db30f37d35adb2e5c33052
    virtual ~RpcLibServerBase() override;

    virtual void start(bool block, std::size_t thread_count) override;
    virtual void stop() override;

    class ApiNotSupported : public std::runtime_error {
    public:
        ApiNotSupported(const std::string& message)
            : std::runtime_error(message) {
        }
    };

protected:
    void* getServer() const;


    virtual VehicleApiBase* getVehicleApi(const std::string& vehicle_name)
    {
        auto* api = api_provider_->getVehicleApi(vehicle_name);
        if (api)
            return api;
        else
            throw ApiNotSupported("Vehicle API for '" + vehicle_name + 
                "' is not available. This could either because this is simulation-only API or this vehicle does not exist");
    }
    virtual VehicleSimApiBase* getVehicleSimApi(const std::string& vehicle_name)
    {
        auto* api = api_provider_->getVehicleSimApi(vehicle_name);
        if (api)
            return api;
        else
            throw ApiNotSupported("Vehicle Sim-API for '" + vehicle_name +
                "' is not available. This could either because this is not a simulation or this vehicle does not exist");
    }
    virtual WorldSimApiBase* getWorldSimApi()
    {
        auto* api = api_provider_->getWorldSimApi();
        if (api)
            return api;
        else
            throw ApiNotSupported("World-Sim API "
                "' is not available. This could be because this is not a simulation");
    }


private:
    ApiProvider* api_provider_;

    struct impl;
    std::unique_ptr<impl> pimpl_;
};


}} //namespace
#endif