#include "rv/tracking/tracking.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// Factory method for MovingObject subclasses
std::shared_ptr<MovingObject> MovingObject::createSubclass(const std::string& category) {
    // Fix: Use correct constructor signature for ATagObject and MovingObject
    if (category == "apriltag") {
        std::map<std::string, std::string> info;
        double when = 0.0;
        std::shared_ptr<Sensor> sensor = nullptr;
        return std::make_shared<ATagObject>(info, when, sensor);
    }
    std::map<std::string, std::string> info;
    double when = 0.0;
    std::shared_ptr<Camera> camera = nullptr;
    return std::make_shared<MovingObject>(info, when, camera);
}

Tracking::Tracking() {
    // Example: register apriltag object class
    // In real use, this should be more dynamic
    // ...existing code...
}

Tracking::~Tracking() {
    stop_flag = true;
    if (worker_thread.joinable()) worker_thread.join();
}

int Tracking::getUniqueIDCount(const std::string& category) {
    auto it = trackers.find(category);
    if (it != trackers.end()) {
        return it->second->uuid_manager.unique_id_count;
    }
    std::cout << "No tracker for category: " << category << std::endl;
    return 0;
}

void Tracking::trackObjects(const std::vector<std::shared_ptr<MovingObject>>& objects,
                           const std::vector<std::shared_ptr<MovingObject>>& already_tracked_objects,
                           double when,
                           const std::vector<std::string>& categories,
                           float ref_camera_frame_rate,
                           float max_unreliable_time,
                           float non_measurement_time_dynamic,
                           float non_measurement_time_static) {
    createTrackers(categories, max_unreliable_time, non_measurement_time_dynamic, non_measurement_time_static);
    std::vector<std::string> cats = categories;
    if (cats.empty()) {
        for (const auto& kv : trackers) cats.push_back(kv.first);
    }
    for (const auto& category : cats) {
        updateRefCameraFrameRate(ref_camera_frame_rate, category);
        auto& tracker = trackers[category];
        std::unique_lock<std::mutex> lock(tracker->queue_mutex);
        if (!tracker->queue.empty()) {
            std::cout << "Tracker work queue is not empty: " << category << ", size: " << tracker->queue.size() << std::endl;
            continue;
        }
        std::vector<std::shared_ptr<MovingObject>> new_objects;
        for (const auto& obj : objects) {
            if (obj->category == category) new_objects.push_back(obj);
        }
        tracker->queue.push({new_objects, when, already_tracked_objects});
        tracker->queue_cv.notify_one();
    }
}

void Tracking::updateRefCameraFrameRate(float ref_camera_frame_rate, const std::string& category) {
    if (trackers.count(category) && trackers[category]->ref_camera_frame_rate != ref_camera_frame_rate) {
        trackers[category]->ref_camera_frame_rate = ref_camera_frame_rate;
        // trackers[category]->tracker.update_tracker_params(ref_camera_frame_rate); // Stub
    }
}

void Tracking::createTrackers(const std::vector<std::string>& categories,
                             float max_unreliable_time,
                             float non_measurement_time_dynamic,
                             float non_measurement_time_static) {
    for (const auto& category : categories) {
        if (!trackers.count(category)) {
            // Use the factory method to create the correct subclass
            auto tracker = createTrackerInstance(max_unreliable_time, non_measurement_time_dynamic, non_measurement_time_static);
            trackers[category] = tracker;
            tracker->worker_thread = std::thread(&Tracking::run, tracker.get());
        }
    }
}

// Global object_classes map
std::map<std::string, ObjectClassInfo> object_classes = {
    {"apriltag", ObjectClassInfo{
        [](const std::string& info_str, double when, const std::string& sensor_str) {
            (void)sensor_str; // Silence unused warning
            std::map<std::string, std::string> info;
            if (!info_str.empty()) info["id"] = info_str;
            std::shared_ptr<Sensor> sensor = nullptr; // TODO: convert sensor_str if needed
            return std::make_shared<ATagObject>(info, when, sensor);
        },
        1.0f, // scale
        DEFAULT_EDGE_LENGTH, // x_size
        DEFAULT_EDGE_LENGTH, // y_size
        DEFAULT_EDGE_LENGTH, // z_size
        DEFAULT_TRACKING_RADIUS, // tracking_radius
        false, // project_to_map
        TYPE_1, // shift_type
        false // rotation_from_velocity
    }}
};

void Tracking::updateObjectClasses(const std::vector<std::map<std::string, pybind11::object>>& assets) {
    std::vector<std::string> remaining_object_class_names;
    for (const auto& kv : object_classes) {
        remaining_object_class_names.push_back(kv.first);
    }
    for (const auto& asset : assets) {
        std::string category = asset.at("name").cast<std::string>();
        auto found = std::find(remaining_object_class_names.begin(), remaining_object_class_names.end(), category);
        if (found != remaining_object_class_names.end()) {
            remaining_object_class_names.erase(found);
        }
        ObjectClassInfo info = object_classes.count(category) ? object_classes[category] : ObjectClassInfo{};
        // Set class creator
        if (category == "apriltag") {
            info.creator = [](const std::string& info_str, double when, const std::string& sensor_str) {
                (void)sensor_str;
                std::map<std::string, std::string> info;
                if (!info_str.empty()) info["id"] = info_str;
                std::shared_ptr<Sensor> sensor = nullptr;
                return std::make_shared<ATagObject>(info, when, sensor);
            };
        } else {
            info.creator = [](const std::string& info_str, double when, const std::string& sensor_str) {
                (void)sensor_str;
                std::map<std::string, std::string> info;
                if (!info_str.empty()) info["id"] = info_str;
                std::shared_ptr<Camera> camera = nullptr;
                return std::make_shared<MovingObject>(info, when, camera);
            };
        }
        // Set parameters from asset, using correct types
        if (asset.count("scale")) info.scale = asset.at("scale").cast<double>();
        if (asset.count("x_size")) info.x_size = asset.at("x_size").cast<double>();
        if (asset.count("y_size")) info.y_size = asset.at("y_size").cast<double>();
        if (asset.count("z_size")) info.z_size = asset.at("z_size").cast<double>();
        if (asset.count("tracking_radius")) info.tracking_radius = asset.at("tracking_radius").cast<double>();
        if (asset.count("project_to_map")) info.project_to_map = asset.at("project_to_map").cast<bool>();
        if (asset.count("shift_type")) info.shift_type = asset.at("shift_type").cast<int>();
        if (asset.count("rotation_from_velocity")) info.rotation_from_velocity = asset.at("rotation_from_velocity").cast<bool>();
        object_classes[category] = info;
    }
    for (const auto& category : remaining_object_class_names) {
        object_classes.erase(category);
    }
}

void Tracking::trackCategory(const std::vector<std::shared_ptr<MovingObject>>& objects,
                            double when,
                            const std::vector<std::shared_ptr<MovingObject>>& tracks) {
    (void)objects;
    (void)when;
    (void)tracks;
    // You must implement in your subclass
    std::cout << "trackCategory() not implemented" << std::endl;
}

std::vector<std::shared_ptr<MovingObject>> Tracking::currentObjects(const std::string& category) {
    std::vector<std::string> categories;
    if (category.empty()) {
        for (const auto& kv : trackers) categories.push_back(kv.first);
    } else {
        categories.push_back(category);
    }
    std::vector<std::shared_ptr<MovingObject>> cur_objects;
    for (const auto& cat : categories) {
        if (trackers.count(cat)) {
            auto& tracker = trackers[cat];
            cur_objects.insert(cur_objects.end(), tracker->curObjects.begin(), tracker->curObjects.end());
        }
    }
    if (category.empty()) {
        // Optionally group objects
        // groupObjects(cur_objects);
    }
    return cur_objects;
}

void Tracking::run() {
    uuid_manager.connectDatabase();
    while (!stop_flag) {
        std::unique_lock<std::mutex> lock(queue_mutex);
        queue_cv.wait(lock, [this]{ return !queue.empty() || stop_flag; });
        if (stop_flag) break;
        auto [objects, when, already_tracked_objects] = queue.front();
        queue.pop();
        lock.unlock();
        if (objects.empty()) continue;
        trackCategory(objects, when, already_tracked_objects);
        curObjects = _objects;
        curObjects.insert(curObjects.end(), this->already_tracked_objects.begin(), this->already_tracked_objects.end());
    }
}

void Tracking::waitForComplete() {
    std::unique_lock<std::mutex> lock(queue_mutex);
    while (!queue.empty()) {
        lock.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        lock.lock();
    }
}

void Tracking::join() {
    for (auto& kv : trackers) {
        auto& tracker = kv.second;
        {
            std::unique_lock<std::mutex> lock(tracker->queue_mutex);
            tracker->queue.push(std::make_tuple(std::vector<std::shared_ptr<MovingObject>>{}, 0.0, std::vector<std::shared_ptr<MovingObject>>{}));
            tracker->queue_cv.notify_one();
        }
        tracker->waitForComplete();
        if (tracker->worker_thread.joinable()) tracker->worker_thread.join();
    }
}

std::shared_ptr<MovingObject> Tracking::createObject(const std::string& sensorType,
                                                    const std::string& info_str,
                                                    double when,
                                                    const std::string& sensor_str) {
    (void)sensor_str; // Silence unused warning
    auto it = object_classes.find(sensorType);
    if (it != object_classes.end()) {
        const ObjectClassInfo& oclass = it->second;
        std::map<std::string, std::string> info;
        if (!info_str.empty()) info["id"] = info_str;
        if (sensorType == "apriltag") {
            std::shared_ptr<Sensor> sensor = nullptr; // TODO: convert sensor_str if needed
            auto mobj = std::static_pointer_cast<MovingObject>(std::make_shared<ATagObject>(info, when, sensor));
            mobj->asset_scale = oclass.scale;
            mobj->size = {oclass.x_size, oclass.y_size, oclass.z_size};
            mobj->tracking_radius = oclass.tracking_radius;
            mobj->project_to_map = oclass.project_to_map;
            mobj->shift_type = oclass.shift_type;
            mobj->rotation_from_velocity = oclass.rotation_from_velocity;
            return mobj;
        } else {
            std::shared_ptr<Camera> camera = nullptr; // TODO: convert sensor_str if needed
            auto mobj = std::make_shared<MovingObject>(info, when, camera);
            mobj->asset_scale = oclass.scale;
            mobj->size = {oclass.x_size, oclass.y_size, oclass.z_size};
            mobj->tracking_radius = oclass.tracking_radius;
            mobj->project_to_map = oclass.project_to_map;
            mobj->shift_type = oclass.shift_type;
            mobj->rotation_from_velocity = oclass.rotation_from_velocity;
            return mobj;
        }
    } else {
        std::map<std::string, std::string> info;
        if (!info_str.empty()) info["id"] = info_str;
        std::shared_ptr<Camera> camera = nullptr;
        return std::make_shared<MovingObject>(info, when, camera);
    }
}

std::map<std::string, std::vector<std::shared_ptr<MovingObject>>> Tracking::groupObjects(const std::vector<std::shared_ptr<MovingObject>>& objects) {
    std::map<std::string, std::vector<std::shared_ptr<MovingObject>>> ogroups;
    for (const auto& obj : objects) {
        std::string otype = obj->category;
        ogroups[otype].push_back(obj);
    }
    return ogroups;
}
