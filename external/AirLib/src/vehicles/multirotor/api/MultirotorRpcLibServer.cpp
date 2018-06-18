// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "vehicles/multirotor/api/MultirotorRpcLibServer.hpp"


#include "common/Common.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "common/common_utils/MinWinDefines.hpp"
#undef NOUSER
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
//#undef check
#include "rpc/server.h"
#include "vehicles/multirotor/api/MultirotorRpcLibAdapators.hpp"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#define check(expr) (static_cast<void>((expr)))
STRICT_MODE_ON


namespace msr { namespace airlib {

typedef msr::airlib_rpclib::MultirotorRpcLibAdapators MultirotorRpcLibAdapators;

MultirotorRpcLibServer::MultirotorRpcLibServer(MultirotorApi* drone, string server_address, uint16_t port)
        : RpcLibServerBase(drone, server_address, port)
{
    (static_cast<rpc::server*>(getServer()))->
        bind("armDisarm", [&](bool arm) -> bool { return getDroneApi()->armDisarm(arm); });

    (static_cast<rpc::server*>(getServer()))->
        bind("takeoff", [&](float max_wait_seconds) -> bool { return getDroneApi()->takeoff(max_wait_seconds); });

    (static_cast<rpc::server*>(getServer()))->
        bind("land", [&](float max_wait_seconds) -> bool { return getDroneApi()->land(max_wait_seconds); });

    (static_cast<rpc::server*>(getServer()))->
        bind("moveByAngleThrottle", [&](float pitch, float roll, float throttle, float yaw_rate, float duration) ->
            bool { return getDroneApi()->moveByAngleThrottle(pitch, roll, throttle, yaw_rate, duration); });

    (static_cast<rpc::server*>(getServer()))->
        bind("hover", [&]() -> bool { return getDroneApi()->hover(); });

	(static_cast<rpc::server*>(getServer()))->
		bind("getGpsLocation", [&]() -> MultirotorRpcLibAdapators::GpsData {
		return MultirotorRpcLibAdapators::GpsData(getDroneApi()->getGpsLocation());
	});

	(static_cast<rpc::server*>(getServer()))->
		bind("getBarometerdata", [&]() -> MultirotorRpcLibAdapators::BarometerData {
		return MultirotorRpcLibAdapators::BarometerData(getDroneApi()->getBarometerdata());
	});
	
	(static_cast<rpc::server*>(getServer()))->
		bind("getMagnetometerdata", [&]() -> MultirotorRpcLibAdapators::MagnetometerData {
		return MultirotorRpcLibAdapators::MagnetometerData(getDroneApi()->getMagnetometerdata());
	});
	
	(static_cast<rpc::server*>(getServer()))->
		bind("getImudata", [&]() -> MultirotorRpcLibAdapators::ImuData {
		return MultirotorRpcLibAdapators::ImuData(getDroneApi()->getImudata());
	});

}

//required for pimpl
MultirotorRpcLibServer::~MultirotorRpcLibServer()
{
}

MultirotorApi* MultirotorRpcLibServer::getDroneApi()
{
    return static_cast<MultirotorApi*>(RpcLibServerBase::getVehicleApi());
}

}} //namespace


#endif
#endif
