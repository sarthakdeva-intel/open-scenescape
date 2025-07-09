#include "rv/tracking/TrackManager.hpp"
#include "rv/tracking/MultipleObjectTracker.hpp"
#include <iostream>
#include <numeric>
#include <algorithm>
#include <uuid/uuid.h> // For UUID generation
#include <ctime>
#include "rv/tracking/moving_object.hpp"
#include "rv/tracking/tracking.hpp"
#include "rv/tracking/ilabs_tracking.hpp"

IntelLabsTracking::IntelLabsTracking(float max_unreliable_time, float non_measurement_time_dynamic, float non_measurement_time_static)
{
    // Tracker config
    rv::tracking::TrackManagerConfig tracker_config;
    tracker_config.mDefaultProcessNoise = 1e-4;
    tracker_config.mDefaultMeasurementNoise = 2e-1;
    tracker_config.mInitStateCovariance = 1;
    tracker_config.mMotionModels = {rv::tracking::MotionModel::CV, rv::tracking::MotionModel::CA, rv::tracking::MotionModel::CTRV};

    if (checkValidTimeParameters(max_unreliable_time, non_measurement_time_dynamic, non_measurement_time_static)) {
        tracker_config.mMaxUnreliableTime = max_unreliable_time;
        tracker_config.mNonMeasurementTimeDynamic = non_measurement_time_dynamic;
        tracker_config.mNonMeasurementTimeStatic = non_measurement_time_static;
    } else {
        std::cerr << "The time-based parameters need to be positive and less than 10 seconds. Initiating the tracker with the default values of the time-based parameters." << std::endl;
        tracker_config.mMaxUnreliableTime = MAX_UNRELIABLE_TIME;
        tracker_config.mNonMeasurementTimeDynamic = NON_MEASUREMENT_TIME_DYNAMIC;
        tracker_config.mNonMeasurementTimeStatic = NON_MEASUREMENT_TIME_STATIC;
    }
    tracker = std::make_unique<rv::tracking::MultipleObjectTracker>(tracker_config);
    tracker->updateTrackerParams(ref_camera_frame_rate);
}

bool IntelLabsTracking::checkValidTimeParameters(double max_unreliable_time, double non_measurement_time_dynamic, double non_measurement_time_static) {
    return (max_unreliable_time > 0 && max_unreliable_time < 10 &&
            non_measurement_time_dynamic > 0 && non_measurement_time_dynamic < 10 &&
            non_measurement_time_static > 0 && non_measurement_time_static < 10);
}

std::vector<double> IntelLabsTracking::rvClassification(double confidence) {
    return {confidence, 1.0 - confidence};
}

rv::tracking::TrackedObject IntelLabsTracking::toRVObject(const std::shared_ptr<MovingObject>& sscape_object) {
    // Generate UUID
    uuid_t uuid;
    char uuid_str[37];
    uuid_generate(uuid);
    uuid_unparse(uuid, uuid_str);
    // Use oid as unique identifier if available
    std::string object_id = sscape_object->oid.empty() ? std::string(uuid_str) : sscape_object->oid;

    rv::tracking::TrackedObject rv_object;
    auto pt = sscape_object->sceneLoc();
    rv_object.x = pt ? pt->x() : 0.0;
    rv_object.y = pt ? pt->y() : 0.0;
    rv_object.z = pt ? pt->z() : 0.0;
    auto size = sscape_object->size.empty() ? std::vector<double>{DEFAULT_EDGE_LENGTH, DEFAULT_EDGE_LENGTH, DEFAULT_EDGE_LENGTH} : sscape_object->size;
    rv_object.length = size[0];
    rv_object.width = size[1];
    rv_object.height = size[2];
    rv_object.yaw = sscape_object->rotation.empty() ? 0.0 : sscape_object->rotation[1];
    double conf = sscape_object->confidence.has_value() ? sscape_object->confidence.value() : 0.5;
    auto cls = rvClassification(conf);
    rv_object.classification = Eigen::VectorXd::Map(cls.data(), cls.size());
    rv_object.attributes["info"] = object_id;
    return rv_object;
}

void IntelLabsTracking::updateTracks(const std::vector<std::shared_ptr<MovingObject>>& objects, std::chrono::system_clock::time_point timestamp) {
    std::vector<rv::tracking::TrackedObject> rv_objects;
    for (const auto& obj : objects) {
        rv_objects.push_back(toRVObject(obj));
    }
    double tracking_radius = DEFAULT_TRACKING_RADIUS;
    if (!objects.empty()) {
        double sum = 0.0;
        for (const auto& x : objects) sum += x->tracking_radius;
        tracking_radius = sum / objects.size();
    }
    tracker->track(rv_objects, timestamp, rv::tracking::DistanceType::Euclidean, tracking_radius);
}

std::shared_ptr<MovingObject> IntelLabsTracking::fromTrackedObject(const rv::tracking::TrackedObject& tracked_object, const std::vector<std::shared_ptr<MovingObject>>& objects) {
    std::string uuid = tracked_object.attributes.at("info");
    std::shared_ptr<MovingObject> sscape_object = nullptr;
    for (const auto& obj : objects) {
        if (uuid == obj->oid) {
            sscape_object = obj;
            break;
        }
    }
    if (!sscape_object) {
        for (const auto& obj : this->_objects) {
            if (uuid == obj->oid) {
                return obj;
            }
        }
    }
    if (!sscape_object) return nullptr;
    if (!sscape_object->location.empty() && sscape_object->location[0])
        sscape_object->location[0]->point = std::make_shared<Point>(tracked_object.x, tracked_object.y, tracked_object.z);
    // velocity is a vector of shared_ptr<Vector>, but for now just skip or implement as needed
    // sscape_object->velocity = ...
    // Fix: tracked_object.id is int, setId expects string
    sscape_object->setId(std::to_string(tracked_object.id));
    bool found = false;
    for (const auto& obj : this->_objects) {
        if (obj->getId() == sscape_object->getId()) {
            found = true;
            sscape_object->setPrevious(obj);
            sscape_object->inferRotationFromVelocity();
            break;
        }
    }
    if (!found) {
        sscape_object->setGID(uuid);
    }
    // UUIDManager assignID and pruneInactiveTracks are not implemented, so skip
    return sscape_object;
}

std::vector<std::shared_ptr<MovingObject>> IntelLabsTracking::mergeAlreadyTrackedObjects(const std::vector<std::shared_ptr<MovingObject>>& tracks) {
    double now = get_epoch_time();
    std::vector<std::shared_ptr<MovingObject>> result;
    std::map<std::string, std::pair<std::shared_ptr<MovingObject>, std::shared_ptr<MovingObject>>> existing_tracks;
    std::map<std::string, std::shared_ptr<MovingObject>> new_tracks;
    std::map<std::string, std::shared_ptr<MovingObject>> non_existing_tracks;
    for (const auto& new_obj : tracks) {
        bool found = false;
        for (const auto& existing_obj : this->already_tracked_objects_) {
            if (new_obj->oid == existing_obj->oid) {
                found = true;
                existing_tracks[new_obj->oid] = std::make_pair(new_obj, existing_obj);
                break;
            }
        }
        if (!found) {
            new_tracks[new_obj->oid] = new_obj;
        }
    }
    for (const auto& existing_obj : this->already_tracked_objects_) {
        if (existing_tracks.find(existing_obj->oid) == existing_tracks.end()) {
            non_existing_tracks[existing_obj->oid] = existing_obj;
        }
    }
    for (const auto& kv : existing_tracks) {
        auto new_obj = kv.second.first;
        auto old_obj = kv.second.second;
        new_obj->setPrevious(old_obj);
        new_obj->inferRotationFromVelocity();
        new_obj->last_seen = now;
        result.push_back(new_obj);
    }
    for (const auto& kv : new_tracks) {
        auto obj = kv.second;
        obj->setGID(obj->oid);
        obj->last_seen = now;
        result.push_back(obj);
    }
    for (const auto& kv : non_existing_tracks) {
        auto obj = kv.second;
        if (obj->last_seen.has_value() && (now - obj->last_seen.value() < MAX_UNRELIABLE_TIME)) {
            result.push_back(obj);
        }
    }
    return result;
}

void IntelLabsTracking::trackCategory(const std::vector<std::shared_ptr<MovingObject>>& objects,
                                      double when,
                                      const std::vector<std::shared_ptr<MovingObject>>& already_tracked_objects) {
    // Convert 'when' (double) to std::chrono::system_clock::time_point
    auto tp = std::chrono::system_clock::from_time_t(static_cast<time_t>(when));
    updateTracks(objects, tp);
    // Fix: Use correct method name getReliableTracks
    auto tracked_objects = tracker->getReliableTracks();

    std::vector<std::string> tracked_ids;
    for (const auto& obj : tracked_objects) {
        tracked_ids.push_back(std::to_string(obj.id));
    }
    uuid_manager.pruneInactiveTracks(tracked_ids);
    std::vector<std::shared_ptr<MovingObject>> tracks_from_detections;
    for (const auto& tracked_object : tracked_objects) {
        auto obj = fromTrackedObject(tracked_object, objects);
        if (obj) tracks_from_detections.push_back(obj);
    }
    this->already_tracked_objects_ = mergeAlreadyTrackedObjects(already_tracked_objects);
    this->_objects = tracks_from_detections;
    this->_objects.insert(this->_objects.end(), this->already_tracked_objects_.begin(), this->already_tracked_objects_.end());
}

float ref_camera_frame_rate = 30.0f;
