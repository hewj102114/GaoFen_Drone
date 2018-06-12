// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#include "common/Common.hpp"
#include <thread>
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#undef check
#ifdef nil
#undef nil
#endif // nil
#include "rpc/client.h"
#include "vehicles/multirotor/api/MultirotorRpcLibAdapators.hpp"
STRICT_MODE_ON
#ifdef _MSC_VER
__pragma(warning( disable : 4239))
#endif			  


namespace msr { namespace airlib {


typedef msr::airlib_rpclib::MultirotorRpcLibAdapators MultirotorRpcLibAdapators;

MultirotorRpcLibClient::MultirotorRpcLibClient(const string&  ip_address, uint16_t port, uint timeout_ms)
    : RpcLibClientBase(ip_address, port, timeout_ms)
{
}

MultirotorRpcLibClient::~MultirotorRpcLibClient()
{}

bool MultirotorRpcLibClient::armDisarm(bool arm)
{
    return static_cast<rpc::client*>(getClient())->call("armDisarm", arm).as<bool>();
}
bool MultirotorRpcLibClient::takeoff(float max_wait_seconds)
{
    return static_cast<rpc::client*>(getClient())->call("takeoff", max_wait_seconds).as<bool>();
}
bool MultirotorRpcLibClient::land(float max_wait_seconds)
{
    return static_cast<rpc::client*>(getClient())->call("land", max_wait_seconds).as<bool>();
}
bool MultirotorRpcLibClient::moveByAngleThrottle(float pitch, float roll, float throttle, float yaw_rate, float duration)
{
    return static_cast<rpc::client*>(getClient())->call("moveByAngleThrottle", pitch, roll, throttle, yaw_rate, duration).as<bool>();
}
bool MultirotorRpcLibClient::hover()
{
    return static_cast<rpc::client*>(getClient())->call("hover").as<bool>();
}
GeoPoint MultirotorRpcLibClient::getGpsLocation()
{
    return static_cast<rpc::client*>(getClient())->call("getGpsLocation").as<MultirotorRpcLibAdapators::GeoPoint>().to();
}

BarometerData MultirotorRpcLibClient::getBarometerdata(float period)
{
	return static_cast<rpc::client*>(getClient())->call("getBarometerdata", period).as<MultirotorRpcLibAdapators::BarometerData>().to();
}

MagnetometerData MultirotorRpcLibClient::getMagnetometerdata(float period)
{
	return static_cast<rpc::client*>(getClient())->call("getMagnetometerdata", period).as<MultirotorRpcLibAdapators::MagnetometerData>().to();
}

ImuData  MultirotorRpcLibClient::getImudata(float period)
{
	return static_cast<rpc::client*>(getClient())->call("getImudata", period).as<MultirotorRpcLibAdapators::ImuData>().to();
}

}} //namespace

#endif
#endif
