/* -------------------------------------------------------------------------- *
 * OpenSim Moco: FreePointBodyActuator.cpp                                    *
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

#include <OpenSim/Simulation/Model/Model.h>
#include "FreePointBodyActuator.h"

using namespace OpenSim;

FreePointBodyActuator::FreePointBodyActuator() {
    constructProperties();
}

FreePointBodyActuator::FreePointBodyActuator(const Body& body,
        bool pointIsGlobal,
        bool spatialForceIsGlobal) : FreePointBodyActuator() {

    connectSocket_body(body);
    set_point_is_global(pointIsGlobal);
    set_spatial_force_is_global(spatialForceIsGlobal);
}

void FreePointBodyActuator::setBodyName(const std::string& name) {
    updSocket<Body>("body").setConnecteePath(name);
}

const std::string& FreePointBodyActuator::getBodyName() const {
    return getSocket<Body>("body").getConnecteePath();
}

void FreePointBodyActuator::setBody(const Body& body) {
    connectSocket_body(body);
}

const Body& FreePointBodyActuator::getBody() const {
    return getConnectee<Body>("body");
}

void FreePointBodyActuator::constructProperties() {
    constructProperty_point_is_global(false);
    constructProperty_spatial_force_is_global(true);
}

void FreePointBodyActuator::computeForce(const SimTK::State& s,
        SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
        SimTK::Vector& generalizedForces) const {
    if (!_model) return;

    const bool spatialForceIsGlobal = getSpatialForceIsGlobal();

    const Body& body = getBody();

    // get the control signals
    const SimTK::Vector bodyForceAndPointVals = getControls(s);
    
    int count = 0;

    SimTK::Vec3 torqueVec;
    for (int i = 0; i < 3; ++i) {
        if (m_enabled_controls[i]) {
            torqueVec[i] = m_control_max_vals[i] * bodyForceAndPointVals[count];
            ++count;
        } else {
            torqueVec[i] = 0;
        }
    }

    SimTK::Vec3 forceVec;
    for (int i = 0; i < 3; ++i) {
        if (m_enabled_controls[i + 3]) {
            forceVec[i] = 
                    m_control_max_vals[i + 3] * bodyForceAndPointVals[count];
            ++count;
        } else {
            forceVec[i] = 0;
        }
    }

    SimTK::Vec3 pointVec;
    for (int i = 0; i < 3; ++i) {
        if (m_enabled_controls[i + 6]) {
            pointVec[i] = 
                    m_control_max_vals[i + 6] * bodyForceAndPointVals[count];
            ++count;
        } else {
            pointVec[i] = 0;
        }
    }

    // if the user has given the spatialForces in body frame, transform them to
    // global (ground) frame
    if (!spatialForceIsGlobal){
        torqueVec = body.expressVectorInGround(s, torqueVec);
        forceVec = body.expressVectorInGround(s, forceVec);
    }

    // if the point of applying force is not in body frame (which is the default 
    // case) transform it to body frame
    if (get_point_is_global()) {
        pointVec = getModel().getGround().
                findStationLocationInAnotherFrame(s, pointVec, body);
    }

    applyTorque(s, body, torqueVec, bodyForces);
    applyForceToPoint(s, body, pointVec, forceVec, bodyForces);

}

double FreePointBodyActuator::getPower(const SimTK::State& s) const {
    OPENSIM_THROW_FRMOBJ(Exception, "Not implemented.");
    //    const Body& body = getBody();
    //
    //    const SimTK::MobilizedBody& body_mb = body.getMobilizedBody();
    //    SimTK::SpatialVec bodySpatialVelocities = body_mb.getBodyVelocity(s);
    //
    //    SimTK::Vector bodyVelocityVec(6);
    //    bodyVelocityVec[0] = bodySpatialVelocities[0][0];
    //    bodyVelocityVec[1] = bodySpatialVelocities[0][1];
    //    bodyVelocityVec[2] = bodySpatialVelocities[0][2];
    //    bodyVelocityVec[3] = bodySpatialVelocities[1][0];
    //    bodyVelocityVec[4] = bodySpatialVelocities[1][1];
    //    bodyVelocityVec[5] = bodySpatialVelocities[1][2];
    //
    //    const SimTK::Vector bodyForceAndPointVals = getControls(s);
    //    const SimTK::Vector bodyForceVals(6,
    //            bodyForceAndPointVals.getContiguousScalarData(), true);
    //
    //    // TODO: this is incorrect since the point can move along the body. How to
    //    // get the velocity of the point controls?
    //    double power = ~bodyForceVals * bodyVelocityVec;
    //
    //    return power;
}
