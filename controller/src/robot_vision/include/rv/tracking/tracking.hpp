#pragma once
#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <map>
#include <vector>
#include <string>
#include <functional>
#include <memory>
#include <atomic>
#include "moving_object.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

class UUIDManager {
public:
    std::unordered_map<std::string, std::pair<std::string, std::optional<float>>> active_ids_;
    int unique_id_count = 0;
    void connectDatabase() { std::cout << "UUIDManager: connectDatabase() called" << std::endl; }
    void pruneInactiveTracks(const std::vector<std::string>& tracked_ids) {
    std::unordered_map<std::string, std::pair<std::string, std::optional<float>>> new_active_ids;
    std::vector<std::string> inactive_tracks;
    for (const auto& kv : active_ids_) {
        if (std::find(tracked_ids.begin(), tracked_ids.end(), kv.first) != tracked_ids.end()) {
            new_active_ids[kv.first] = kv.second;
        } else {
            inactive_tracks.push_back(kv.first);
        }
    }
    active_ids_ = new_active_ids;
    for (const auto& track_id : inactive_tracks) {
        // If similarity is None (nullopt), increment unique_id_count_
        if (!active_ids_[track_id].second.has_value()) {
            unique_id_count++;
        }
    }
}
};

// Constants
constexpr int TYPE_1 = 1;
constexpr float MAX_UNRELIABLE_TIME = 0.3333f;
constexpr float NON_MEASUREMENT_TIME_DYNAMIC = 0.2666f;
constexpr float NON_MEASUREMENT_TIME_STATIC = 0.5333f;

// Object class factory
struct ObjectClassInfo {
    std::function<std::shared_ptr<MovingObject>(const std::string&, double, const std::string&)> creator;
    double scale = 1.0;
    double x_size = DEFAULT_EDGE_LENGTH;
    double y_size = DEFAULT_EDGE_LENGTH;
    double z_size = DEFAULT_EDGE_LENGTH;
    double tracking_radius = DEFAULT_TRACKING_RADIUS;
    bool project_to_map = false;
    int shift_type = TYPE_1;
    bool rotation_from_velocity = false;
};

extern std::map<std::string, ObjectClassInfo> object_classes;

class Tracking {
public:
    Tracking();
    virtual ~Tracking();
    int getUniqueIDCount(const std::string& category);
    void trackObjects(const std::vector<std::shared_ptr<MovingObject>>& objects,
                     const std::vector<std::shared_ptr<MovingObject>>& already_tracked_objects,
                     double when,
                     const std::vector<std::string>& categories,
                     float ref_camera_frame_rate,
                     float max_unreliable_time,
                     float non_measurement_time_dynamic,
                     float non_measurement_time_static);
    void updateRefCameraFrameRate(float ref_camera_frame_rate, const std::string& category);
    void createTrackers(const std::vector<std::string>& categories,
                        float max_unreliable_time,
                        float non_measurement_time_dynamic,
                        float non_measurement_time_static);
    virtual void updateObjectClasses(const std::vector<std::map<std::string, pybind11::object>>& assets);
    virtual void trackCategory(const std::vector<std::shared_ptr<MovingObject>>& objects,
                              double when,
                              const std::vector<std::shared_ptr<MovingObject>>& tracks);
    std::vector<std::shared_ptr<MovingObject>> currentObjects(const std::string& category = "");
    void run();
    void waitForComplete();
    void join();
    static std::shared_ptr<MovingObject> createObject(const std::string& sensorType,
                                                     const std::string& info,
                                                     double when,
                                                     const std::string& sensor);
    std::map<std::string, std::vector<std::shared_ptr<MovingObject>>> groupObjects(const std::vector<std::shared_ptr<MovingObject>>& objects);

    // Threading and queue
    std::thread worker_thread;
    std::queue<std::tuple<std::vector<std::shared_ptr<MovingObject>>, double, std::vector<std::shared_ptr<MovingObject>>>> queue;
    std::mutex queue_mutex;
    std::condition_variable queue_cv;
    std::atomic<bool> stop_flag{false};

    // Data members
    std::map<std::string, std::shared_ptr<Tracking>> trackers;
    std::vector<std::shared_ptr<MovingObject>> _objects;
    std::vector<std::shared_ptr<MovingObject>> curObjects;
    std::vector<std::shared_ptr<MovingObject>> already_tracked_objects;
    UUIDManager uuid_manager;
    float ref_camera_frame_rate = 0.0f;

protected:
    // Remove parameter names to avoid unused variable warnings
    virtual std::shared_ptr<Tracking> createTrackerInstance(float, float, float) {
        // By default, create base Tracking
        return std::make_shared<Tracking>();
    }
};
