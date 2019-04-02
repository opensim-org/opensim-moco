#ifndef MOCO_FREEPOINTBODYACTUATOR_H
#define MOCO_FREEPOINTBODYACTUATOR_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: FreePointBodyActuator.h                                      *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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

#include <OpenSim/Simulation/Model/Actuator.h>
#include "../osimMocoDLL.h"

namespace OpenSim {

class OSIMMOCO_API FreePointBodyActuator : public Actuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(FreePointBodyActuator, Actuator);
public:
    OpenSim_DECLARE_PROPERTY(point_is_global, bool,
        "Interpret point in Ground frame if true; otherwise, body frame. "
        "Default: false.");
    OpenSim_DECLARE_PROPERTY(spatial_force_is_global, bool,
        "Interpret axis in Ground frame if true; otherwise, body's frame. "
        "Default: true.");
    OpenSim_DECLARE_SOCKET(body, Body,
        "The body on which to apply the spatial force.");

    FreePointBodyActuator();
    explicit FreePointBodyActuator(const Body& body,
            bool pointIsGlobal = false,
            bool spatialForceIsGlobal = true);

    /// Set the 'point_is_global' property that determines whether the point is
    /// specified in inertial coordinates or in the body's local coordinates.
    void setPointForceIsGlobal(bool isGlobal)
    {   set_point_is_global(isGlobal); }
    /// Return the current value of the 'point_is_global' property.
    bool getPointIsGlobal() const
    {   return get_point_is_global(); }

    /// Set the 'spatial_force_is_global' property that determines how to
    /// interpret the 'axis' vector; if not global (Ground frame) it is in 
    /// body's frame.
    void setSpatialForceIsGlobal(bool isGlobal)
    {   set_spatial_force_is_global(isGlobal); }
    /// Return the current value of the 'spatial_force_is_global' property.
    bool getSpatialForceIsGlobal() const
    {   return get_spatial_force_is_global(); }

    /// Set the body to which this actuator applies spatial forces.
    void setBody(const Body& body);
    const Body& getBody() const;
    /// Set the body name to which this actuator applies spatial forces.
    void setBodyName(const std::string& name);
    const std::string& getBodyName() const;

    //SimTK::Vec3 getPoint(const SimTK::State& s) const;
private:
    void constructProperties();

    void computeForce(const SimTK::State& state,
            SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
            SimTK::Vector& mobilityForces) const override;

    int numControls() const override { return 9; }
    double getPower(const SimTK::State& s) const override;

}; // class FreePointBodyActuator

} // namespace OpenSim

#endif // MOCO_FREEPOINTBODYACTUATOR_H