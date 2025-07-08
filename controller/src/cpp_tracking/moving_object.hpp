#ifndef CONTROLLER_CPP_TRACKING_MOVING_OBJECT_HPP
#define CONTROLLER_CPP_TRACKING_MOVING_OBJECT_HPP

#include <string>
#include <memory>
#include <vector>
#include <map>
#include <mutex>
#include <optional>
#include "sensor.hpp"
// Add includes for geometry, Point, Rectangle, etc. as needed

constexpr double DEFAULT_EDGE_LENGTH = 1.0;
constexpr double DEFAULT_TRACKING_RADIUS = 2.0;
constexpr int LOCATION_LIMIT = 20;
constexpr double SPEED_THRESHOLD = 0.1;
constexpr double APRILTAG_HOVER_DISTANCE = 0.5;

// Forward declarations for geometry types
class Point;
class Rectangle;
class Camera;
class Scene;

struct ChainData {
    std::map<std::string, std::string> regions;
    std::vector<std::shared_ptr<Point>> publishedLocations;
    std::map<std::string, std::string> sensors;
};

class Chronoloc {
public:
    std::shared_ptr<Point> point;
    double when; // Use double for timestamp
    std::shared_ptr<Rectangle> bounds;
    Chronoloc(const std::shared_ptr<Point>& point, double when, const std::shared_ptr<Rectangle>& bounds);
};

class Vector {
public:
    std::shared_ptr<Camera> camera;
    std::shared_ptr<Point> point;
    double last_seen;
    Vector(const std::shared_ptr<Camera>& camera, const std::shared_ptr<Point>& point, double when);
    std::string repr() const;
};

class MovingObject {
public:
    // Fields
    std::shared_ptr<ChainData> chain_data;
    std::vector<double> size;
    double tracking_radius;
    int shift_type=1;
    bool project_to_map;
    // map_triangle_mesh, map_translation, map_rotation: stubs for now
    bool rotation_from_velocity;
    double first_seen;
    std::optional<double> last_seen;
    std::shared_ptr<Camera> camera;
    std::map<std::string, std::string> info;
    std::string category;
    std::shared_ptr<Rectangle> boundingBox;
    std::shared_ptr<Rectangle> boundingBoxPixels;
    std::optional<double> confidence;
    std::string oid;
    std::optional<std::string> gid;
    int frameCount;
    // velocity: stub for now
    std::vector<std::shared_ptr<Chronoloc>> location;
    std::vector<std::shared_ptr<Vector>> vectors;
    std::vector<double> rotation;
    bool intersected;
    // reidVector: stub for now
    // adjusted: stub for now

    std::optional<double> baseAngle;

    MovingObject() : baseAngle(std::nullopt) {}

    void setBaseAngle(double angle) {
        baseAngle = angle;
    }
    bool hasBaseAngle() const {
        return baseAngle.has_value();
    }

    static int gid_counter;
    static std::mutex gid_lock;

    MovingObject(const std::map<std::string, std::string>& info, double when, const std::shared_ptr<Camera>& camera);
    virtual ~MovingObject();

    void setGID(const std::string& gid);
    void setPrevious(const std::shared_ptr<MovingObject>& otherObj);
    void inferRotationFromVelocity();
    std::shared_ptr<Point> camLoc() const;
    virtual void mapObjectDetectionToWorld(const std::map<std::string, std::string>& info, double when, const std::shared_ptr<Camera>& camera);
    std::shared_ptr<Point> sceneLoc();
    void projectBounds();
    double when() const;
    virtual std::string repr() const;
    static std::shared_ptr<MovingObject> createSubclass(const std::string& category);
    // displayIntersections, dump, load: stubs for now

    // Add ID field and getter
private:
    std::string id_;
public:
    void setId(const std::string& id) { id_ = id; }
    const std::string& getId() const { return id_; }
};

class ATagObject : public MovingObject {
public:
    std::string tag_id;
    ATagObject(const std::map<std::string, std::string>& info, double when, const std::shared_ptr<Sensor>& sensor);
    void mapObjectDetectionToWorld(const std::map<std::string, std::string>& info, double when, const std::shared_ptr<Sensor>& sensor);
    std::string repr() const override;
};

#endif // CONTROLLER_CPP_TRACKING_MOVING_OBJECT_HPP
