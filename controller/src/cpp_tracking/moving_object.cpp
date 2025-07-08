#include "moving_object.hpp"
#include <iostream>
#include <utility>
#include <point.h>
#include <xtensor/xarray.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xnorm.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xio.hpp>

class Camera {
public:
    // Add camera fields as needed
};

// ChainData, Chronoloc, Vector implementations
Chronoloc::Chronoloc(const std::shared_ptr<Point>& point, double when, const std::shared_ptr<Rectangle>& bounds)
    : point(point), when(when), bounds(bounds) {
    if (!point->is3D) {
        this->point = std::make_shared<Point>(point->x, point->y, DEFAULT_EDGE_LENGTH);
    }
}

Vector::Vector(const std::shared_ptr<Camera>& camera, const std::shared_ptr<Point>& point, double when)
    : camera(camera), point(point), last_seen(when) {
    if (!point->is3D) {
        this->point = std::make_shared<Point>(point->x, point->y, DEFAULT_EDGE_LENGTH);
    }
}

std::string Vector::repr() const {
    // Stub for __repr__
    return "Vector: ...";
}

int MovingObject::gid_counter = 0;
std::mutex MovingObject::gid_lock;

MovingObject::MovingObject(const std::map<std::string, std::string>& info, double when, const std::shared_ptr<Camera>& camera)
    : chain_data(nullptr), size(), tracking_radius(DEFAULT_TRACKING_RADIUS), shift_type(1), project_to_map(false),
      rotation_from_velocity(false), first_seen(when), last_seen(), camera(camera), info(info),
      category("object"), boundingBox(nullptr), boundingBoxPixels(nullptr), confidence(), oid(), gid(),
      frameCount(1), location(), vectors(), rotation(), intersected(false) {
    // Copy info
    auto it = info.find("category");
    if (it != info.end()) category = it->second;
    it = info.find("id");
    if (it != info.end()) oid = it->second;
    // Add more field extraction as needed
}

MovingObject::~MovingObject() {}

void MovingObject::setGID(const std::string& gid) {
    chain_data = std::make_shared<ChainData>();
    this->gid = gid;
    this->first_seen = this->when();
}

void MovingObject::setPrevious(const std::shared_ptr<MovingObject>& otherObj) {
    if (!otherObj->location.empty()) {
        this->location = {this->location.empty() ? nullptr : this->location[0]};
        for (size_t i = 0; i < otherObj->location.size() && i < LOCATION_LIMIT - 1; ++i) {
            this->location.push_back(otherObj->location[i]);
        }
    }
    this->chain_data = otherObj->chain_data;
    this->gid = otherObj->gid;
    this->first_seen = otherObj->first_seen;
    this->frameCount = otherObj->frameCount + 1;
    if (this->chain_data && this->chain_data->publishedLocations.size() > LOCATION_LIMIT) {
        this->chain_data->publishedLocations.resize(LOCATION_LIMIT);
    }
}

std::optional<Eigen::Quaterniond> rotationToTarget(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    Eigen::Vector3d v1_norm = v1.normalized();
    Eigen::Vector3d v2_norm = v2.normalized();
    double dot = v1_norm.dot(v2_norm);

    // If vectors are nearly parallel
    if (dot > 1.0 - 1e-6) {
        return Eigen::Quaterniond::Identity();
    }
    // If vectors are nearly opposite
    if (dot < -1.0 + 1e-6) {
        // Find an orthogonal vector for the axis
        Eigen::Vector3d axis = Eigen::Vector3d::UnitX().cross(v1_norm);
        if (axis.norm() < 1e-6)
            axis = Eigen::Vector3d::UnitY().cross(v1_norm);
        axis.normalize();
        return Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, axis));
    }
    // General case
    Eigen::Vector3d axis = v1_norm.cross(v2_norm);
    double s = std::sqrt((1.0 + dot) * 2.0);
    double invs = 1.0 / s;
    Eigen::Quaterniond q;
    q.w() = s * 0.5;
    q.x() = axis.x() * invs;
    q.y() = axis.y() * invs;
    q.z() = axis.z() * invs;
    q.normalize();
    return q;
}

Eigen::Vector3d normalize(const Eigen::Vector3d& vector) {
    double magnitude = vector.norm();
    if (magnitude == 0.0) {
        return vector;
    }
    return vector / magnitude;
}

void MovingObject::inferRotationFromVelocity()
{
    if (rotation_from_velocity && velocity)
    {
        // velocity is assumed to be a struct/class with x, y, z fields
        xt::xarray<double> vel = {velocity->x, velocity->y, velocity->z};
        double speed = xt::linalg::norm(vel)();
        if (speed > SPEED_THRESHOLD)
        {
            xt::xarray<double> velocity = normalize(vel);
            xt::xarray<double> direction = {1.0, 0.0, 0.0};
            this->rotation = rotationToTarget(velocity, direction);
        }
    }
    else if (this->rotation.empty())
    {
        this->rotation = {0.0, 0.0, 0.0, 1.0};
    }
}
