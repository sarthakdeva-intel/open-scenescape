#pragma once

#include "tracking.hpp"
#include "rv/tracking/TrackedObject.hpp"
#include "rv/tracking/TrackManager.hpp"
#include "rv/tracking/MultipleObjectTracker.hpp"
#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <map>

inline double get_epoch_time() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

class IntelLabsTracking : public Tracking {
public:
    IntelLabsTracking(float max_unreliable_time, float non_measurement_time_dynamic, float non_measurement_time_static);
    bool checkValidTimeParameters(double max_unreliable_time, double non_measurement_time_dynamic, double non_measurement_time_static);
    std::vector<double> rvClassification(double confidence = 1.0);
    std::unique_ptr<rv::tracking::MultipleObjectTracker> tracker;
    rv::tracking::TrackedObject toRVObject(const std::shared_ptr<MovingObject>& sscape_object);
    void updateTracks(const std::vector<std::shared_ptr<MovingObject>>& objects, std::chrono::system_clock::time_point timestamp);
    std::shared_ptr<MovingObject> fromTrackedObject(const rv::tracking::TrackedObject& tracked_object, const std::vector<std::shared_ptr<MovingObject>>& objects);
    std::vector<std::shared_ptr<MovingObject>> mergeAlreadyTrackedObjects(const std::vector<std::shared_ptr<MovingObject>>& tracks);
    void trackCategory(const std::vector<std::shared_ptr<MovingObject>>& objects, double when, const std::vector<std::shared_ptr<MovingObject>>& already_tracked_objects) override;
protected:
    using Tracking::_objects;
    std::vector<std::shared_ptr<MovingObject>> already_tracked_objects_;
    std::shared_ptr<Tracking> createTrackerInstance(float max_unreliable_time, float non_measurement_time_dynamic, float non_measurement_time_static) override {
        return std::make_shared<IntelLabsTracking>(max_unreliable_time, non_measurement_time_dynamic, non_measurement_time_static);
    }
};
