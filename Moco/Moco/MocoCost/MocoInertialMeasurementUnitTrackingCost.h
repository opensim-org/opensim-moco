#ifndef MOCO_MOCOINTERNALMEASUREMENTUNITTRACKINGCOST_H
#define MOCO_MOCOINTERNALMEASUREMENTUNITTRACKINGCOST_H

/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoInertialMeasurementUnitTrackingCost.h                    *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Common/TimeSeriesTable.h>
#include "MocoCost.h"

namespace OpenSim {

class MocoInertialMeasurementUnitTrackingCost : public MocoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoInertialMeasurementUnitTrackingCost,
        MocoCost);
public:
    MocoInertialMeasurementUnitTrackingCost() {
        constructProperties();
    }
    void addInertialMeasurementUnit(std::string name,
            std::string bodyPath, SimTK::Vec3 locationInBody,
            TimeSeriesTable_<SimTK::Vec4> orientation,
            TimeSeriesTable acceleration) {
        set_name(std::move(name));
        set_body_path(std::move(bodyPath));
        set_location_in_body(std::move(locationInBody));
        m_orientation = orientation;
        m_acceleration = acceleration;
    }
private:
    OpenSim_DECLARE_PROPERTY(name, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(body_path, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(location_in_body, SimTK::Vec3, "TODO");
    void constructProperties() {
        constructProperty_name("");
        constructProperty_body_path("");
        constructProperty_location_in_body(SimTK::Vec3(0));
    }
    TimeSeriesTable_<SimTK::Vec4> m_orientation;
    TimeSeriesTable m_acceleration;
};

} // namespace OpenSim

#endif // MOCO_MOCOINTERNALMEASUREMENTUNITTRACKINGCOST_H
