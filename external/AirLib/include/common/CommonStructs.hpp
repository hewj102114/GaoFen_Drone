// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_CommonStructs_hpp
#define msr_airlib_CommonStructs_hpp

#include "common/Common.hpp"
#include <ostream>

namespace msr { namespace airlib {

//velocity
struct Twist {
    Vector3r linear, angular;

    Twist()
    {}

    Twist(const Vector3r& linear_val, const Vector3r& angular_val)
        : linear(linear_val), angular(angular_val)
    {
    }

    static const Twist zero()
    {
        static const Twist zero_twist(Vector3r::Zero(), Vector3r::Zero());
        return zero_twist;
    }
};

//force & torque
struct Wrench {
    Vector3r force, torque;

    Wrench()
    {}

    Wrench(const Vector3r& force_val, const Vector3r& torque_val)
        : force(force_val), torque(torque_val)
    {
    }

    //support basic arithmatic 
    Wrench operator+(const Wrench& other) const
    {
        Wrench result;
        result.force = this->force + other.force;
        result.torque = this->torque + other.torque;
        return result;
    }
    Wrench operator+=(const Wrench& other)
    {
        force += other.force;
        torque += other.torque;
        return *this;
    }
    Wrench operator-(const Wrench& other) const
    {
        Wrench result;
        result.force = this->force - other.force;
        result.torque = this->torque - other.torque;
        return result;
    }
    Wrench operator-=(const Wrench& other)
    {
        force -= other.force;
        torque -= other.torque;
        return *this;
    }

    static const Wrench zero()
    {
        static const Wrench zero_wrench(Vector3r::Zero(), Vector3r::Zero());
        return zero_wrench;
    }
};

struct Momentums {
    Vector3r linear;
    Vector3r angular;

    Momentums()
    {}

    Momentums(const Vector3r& linear_val, const Vector3r& angular_val)
        : linear(linear_val), angular(angular_val)
    {
    }

    static const Momentums zero()
    {
        static const Momentums zero_val(Vector3r::Zero(), Vector3r::Zero());
        return zero_val;
    }
};

struct Accelerations {
    Vector3r linear;
    Vector3r angular;

    Accelerations()
    {}

    Accelerations(const Vector3r& linear_val, const Vector3r& angular_val)
        : linear(linear_val), angular(angular_val)
    {
    }

    static const Accelerations zero()
    {
        static const Accelerations zero_val(Vector3r::Zero(), Vector3r::Zero());
        return zero_val;
    }
};

struct PoseWithCovariance {
    VectorMath::Pose pose;
    vector<real_T> covariance;	//36 elements, 6x6 matrix

    PoseWithCovariance()
        : covariance(36, 0)
    {}
};

struct PowerSupply {
    vector<real_T> voltage, current;
};

struct TwistWithCovariance {
    Twist twist;
    vector<real_T> covariance;	//36 elements, 6x6 matrix

    TwistWithCovariance()
        : covariance(36, 0)
    {}
};

struct Joystick {
    vector<float> axes;
    vector<int> buttons;
};

struct Odometry {
    PoseWithCovariance pose;
    TwistWithCovariance twist;
};

struct BarometerData {
	float altitude;    //meters
	float pressure;    //Pascal
	msr::airlib::TTimePoint time_stamp;

	BarometerData()
	{}

	BarometerData(float altitude_val, float pressure_val, msr::airlib::TTimePoint time_stamp_val)
	{
		set(altitude_val, pressure_val, time_stamp_val);
	}

	void set(float altitude_val, float pressure_val, msr::airlib::TTimePoint time_stamp_val)
	{
		altitude = altitude_val, pressure = pressure_val; time_stamp = time_stamp_val;
	}

	friend std::ostream& operator<<(std::ostream &os, BarometerData const &g) {
		return os << "[" << g.altitude << ", " << g.pressure << "]";
	}

	std::string to_string()
	{
		return std::to_string(altitude) + string(", ") + std::to_string(pressure);
	}
};

struct ImuData {
	Vector3r angular_velocity;
	Vector3r linear_acceleration;
	msr::airlib::TTimePoint time_stamp;

	ImuData()
	{}

	ImuData(Vector3r angular_velocity_val, Vector3r linear_acceleration_val, msr::airlib::TTimePoint time_stamp_val)
	{
		set(angular_velocity_val, linear_acceleration_val, time_stamp_val);
	}

	void set(Vector3r angular_velocity_val, Vector3r linear_acceleration_val, msr::airlib::TTimePoint time_stamp_val)
	{
		angular_velocity = angular_velocity_val;  linear_acceleration = linear_acceleration_val;  time_stamp = time_stamp_val;
	}

	/*
	friend std::ostream& operator<<(std::ostream &os, ImuData const &g) {
	//return os << "[" << g.angular_velocity.x << ", " << g.angular_velocity.y << ", " << g.angular_velocity.z << g.linear_acceleration.x << ", " << g.linear_acceleration.y << ", " << g.linear_acceleration.z << "]";
	return  os << "[" << g.angular_velocity.data << ", " << g.linear_acceleration.data << "]";
	}

	std::string to_string()
	{
	//return std::to_string(angular_velocity.x) + string(", ") + std::to_string(angular_velocity.y) + string(", ") + std::to_string(angular_velocity.z) + std::to_string(linear_acceleration.x) + string(", ") + std::to_string(linear_acceleration.y) + string(", ") + std::to_string(linear_acceleration.z);
	return std::to_string(angular_velocity.data) + string(", ") + std::to_string(linear_acceleration.data);
	}
	*/
};

struct MagnetometerData {
	Vector3r magnetic_field_body; //in Gauss
	msr::airlib::TTimePoint time_stamp;

	MagnetometerData()
	{}
	MagnetometerData(Vector3r magnetic_field_body_val, msr::airlib::TTimePoint time_stamp_val)
	{
		set(magnetic_field_body_val, time_stamp_val);
	}
	void set(Vector3r magnetic_field_body_val, msr::airlib::TTimePoint time_stamp_val)
	{
		magnetic_field_body = magnetic_field_body_val; time_stamp = time_stamp_val;
	}
	/*
	friend std::ostream& operator<<(std::ostream &os, MagnetometerData const &g) {
	return os << "[" << g.magnetic_field_body.data << "]";
	}

	std::string to_string()
	{
	//return std::to_string(magnetic_field_body.data) + string(", ") + std::to_string(magnetic_field_body(2)) + string(", ") + std::to_string(magnetic_field_body(3));
	return std::to_string(magnetic_field_body.data);
	}
	*/

};

struct GpsData {
	double latitude = 0, longitude = 0;
	float altitude = 0;
	msr::airlib::TTimePoint time_stamp;


	GpsData()
	{}

	GpsData(double latitude_val, double longitude_val, float altitude_val, msr::airlib::TTimePoint time_stamp_val)
	{
		set(latitude_val, longitude_val, altitude_val, time_stamp_val);
	}

	void set(double latitude_val, double longitude_val, float altitude_val, msr::airlib::TTimePoint time_stamp_val)
	{
		latitude = latitude_val, longitude = longitude_val; altitude = altitude_val; time_stamp = time_stamp_val;
	}

	friend std::ostream& operator<<(std::ostream &os, GpsData const &g) {
		return os << "[" << g.latitude << ", " << g.longitude << ", " << g.altitude << "]";
	}

	std::string to_string()
	{
		return std::to_string(latitude) + string(", ") + std::to_string(longitude) + string(", ") + std::to_string(altitude);
	}
};

struct GeoPoint {
    double latitude = 0, longitude = 0;
    float altitude = 0;

    GeoPoint()
    {}

    GeoPoint(double latitude_val, double longitude_val, float altitude_val)
    {
        set(latitude_val, longitude_val, altitude_val);
    }

    void set(double latitude_val, double longitude_val, float altitude_val)
    {
        latitude = latitude_val, longitude = longitude_val; altitude = altitude_val;
    }

    friend std::ostream& operator<<(std::ostream &os, GeoPoint const &g) { 
        return os << "[" << g.latitude << ", " << g.longitude << ", " << g.altitude << "]";
    }

    std::string to_string()
    {
        return std::to_string(latitude) + string(", ") + std::to_string(longitude) + string(", ") + std::to_string(altitude);
    }
};

struct HomeGeoPoint {
    GeoPoint home_point;
    double lat_rad, lon_rad;
    double cos_lat, sin_lat;

    HomeGeoPoint()
    {}
    HomeGeoPoint(const GeoPoint& home_point_val)
    {
        initialize(home_point_val);
    }
    void initialize(const GeoPoint& home_point_val)
    {
        home_point = home_point_val;
        lat_rad = Utils::degreesToRadians(home_point.latitude);
        lon_rad = Utils::degreesToRadians(home_point.longitude);
        cos_lat = cos(lat_rad);
        sin_lat = sin(lat_rad);
    }
};

struct CollisionInfo {
    bool has_collided = false;
    Vector3r normal = Vector3r::Zero();
    Vector3r impact_point = Vector3r::Zero();
    Vector3r position = Vector3r::Zero();
    real_T penetration_depth = 0;
    TTimePoint time_stamp = 0;
    unsigned int collision_count = 0;
    std::string object_name;
    int object_id = -1;

    CollisionInfo()
    {}

    CollisionInfo(bool has_collided_val, const Vector3r& normal_val, 
        const Vector3r& impact_point_val, const Vector3r& position_val, 
        real_T penetration_depth_val, TTimePoint time_stamp_val,
        const std::string& object_name_val, int object_id_val)
        : has_collided(has_collided_val), normal(normal_val),
        impact_point(impact_point_val), position(position_val),
        penetration_depth(penetration_depth_val), time_stamp(time_stamp_val),
        object_name(object_name_val), object_id(object_id_val)
    {
    }
};

struct CameraInfo {
    Pose pose;
    float fov;

    CameraInfo()
    {}

    CameraInfo(const Pose& pose_val, float fov_val)
        : pose(pose_val), fov(fov_val)
    {
    }
};

struct CollisionResponseInfo {
    unsigned int collision_count_raw = 0;
    unsigned int collision_count_non_resting = 0;
    TTimePoint collision_time_stamp = 0;
};

struct GeoPose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Quaternionr orientation;
    GeoPoint position;
};



}} //namespace
#endif
