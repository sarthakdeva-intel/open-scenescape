/*
 * Copyright (C) 2024 Intel Corporation
 *
 * This software and the related documents are Intel copyrighted materials,
 * and your use of them is governed by the express license under which they
 * were provided to you ("License"). Unless the License provides otherwise,
 * you may not use, modify, copy, publish, distribute, disclose or transmit
 * this software or the related documents without Intel's prior written permission.
 *
 * This software and the related documents are provided as is, with no express
 * or implied warranties, other than those that are expressly stated in the License.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <rv/tracking/ilabs_tracking.hpp>
#include <rv/tracking/TrackedObject.hpp>

namespace py = pybind11;

// Helper: Expose updateObjectClasses with py::object values
py::object updateObjectClasses_py(Tracking& self, const std::vector<std::map<std::string, py::object>>& assets) {
    self.updateObjectClasses(assets);
    return py::none();
}

PYBIND11_MODULE(cpp_tracking, m) {
    // Expose constants
    m.attr("DEFAULT_EDGE_LENGTH") = DEFAULT_EDGE_LENGTH;
    m.attr("DEFAULT_TRACKING_RADIUS") = DEFAULT_TRACKING_RADIUS;
    m.attr("MAX_UNRELIABLE_TIME") = MAX_UNRELIABLE_TIME;
    m.attr("NON_MEASUREMENT_TIME_DYNAMIC") = NON_MEASUREMENT_TIME_DYNAMIC;
    m.attr("NON_MEASUREMENT_TIME_STATIC") = NON_MEASUREMENT_TIME_STATIC;
    m.attr("TYPE_1") = TYPE_1;

    // Expose Tracking base class before IntelLabsTracking
    py::class_<Tracking, std::shared_ptr<Tracking>>(m, "Tracking")
        .def(py::init<>())
        .def("updateObjectClasses", &updateObjectClasses_py)
        ;

    py::class_<IntelLabsTracking, Tracking, std::shared_ptr<IntelLabsTracking>>(m, "IntelLabsTracking")
        .def(py::init<float, float, float>(),
             py::arg("max_unreliable_time"),
             py::arg("non_measurement_time_dynamic"),
             py::arg("non_measurement_time_static"))
        .def("checkValidTimeParameters", &IntelLabsTracking::checkValidTimeParameters,
             py::arg("max_unreliable_time"),
             py::arg("non_measurement_time_dynamic"),
             py::arg("non_measurement_time_static"))
        .def("rvClassification", &IntelLabsTracking::rvClassification,
             py::arg("confidence") = 1.0)
        .def("toRVObject", &IntelLabsTracking::toRVObject)
        .def("updateTracks", &IntelLabsTracking::updateTracks, py::call_guard<py::gil_scoped_release>())
        .def("fromTrackedObject", &IntelLabsTracking::fromTrackedObject)
        .def("mergeAlreadyTrackedObjects", &IntelLabsTracking::mergeAlreadyTrackedObjects)
        .def("trackCategory", &IntelLabsTracking::trackCategory,
             py::arg("objects"), py::arg("when"), py::arg("already_tracked_objects"),
             py::call_guard<py::gil_scoped_release>())
        .def("trackObjects", &IntelLabsTracking::trackObjects, py::call_guard<py::gil_scoped_release>())
        .def("currentObjects", &IntelLabsTracking::currentObjects,
             py::arg("category") = std::string())
        .def("waitForComplete", &IntelLabsTracking::waitForComplete, py::call_guard<py::gil_scoped_release>())
        .def("join", &IntelLabsTracking::join, py::call_guard<py::gil_scoped_release>())
        .def_static("createObject", &Tracking::createObject,
            py::arg("sensorType"), py::arg("info"), py::arg("when"), py::arg("sensor"))
        .def("updateObjectClasses", &updateObjectClasses_py)
        ;
}
