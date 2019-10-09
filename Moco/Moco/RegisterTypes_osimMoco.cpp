/* -------------------------------------------------------------------------- *
 * OpenSim Moco: RegisterTypes_osimMoco.cpp                                   *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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
#include "RegisterTypes_osimMoco.h"

#include "Common/TableProcessor.h"
#include "Components/AccelerationMotion.h"
#include "Components/ActivationCoordinateActuator.h"
#include "Components/DeGrooteFregly2016Muscle.h"
#include "Components/DiscreteForces.h"
#include "Components/PositionMotion.h"
#include "Components/SmoothSphereHalfSpaceForce.h"
#include "Components/StationPlaneContactForce.h"
#include "Components/MultivariatePolynomialFunction.h"
#ifdef MOCO_WITH_TROPTER
#    include "InverseMuscleSolver/GlobalStaticOptimization.h"
#    include "InverseMuscleSolver/INDYGO.h"
#endif
#include "MocoBounds.h"
#include "MocoCasADiSolver/MocoCasADiSolver.h"
#include "MocoControlBoundConstraint.h"
#include "MocoGoal/MocoControlGoal.h"
#include "MocoGoal/MocoControlTrackingGoal.h"
#include "MocoGoal/MocoGoal.h"
#include "MocoGoal/MocoInitialActivationGoal.h"
#include "MocoGoal/MocoJointReactionGoal.h"
#include "MocoGoal/MocoMarkerFinalGoal.h"
#include "MocoGoal/MocoMarkerTrackingGoal.h"
#include "MocoGoal/MocoOrientationTrackingGoal.h"
#include "MocoGoal/MocoPeriodicityGoal.h"
#include "MocoGoal/MocoStateTrackingGoal.h"
#include "MocoGoal/MocoSumSquaredStateGoal.h"
#include "MocoGoal/MocoTranslationTrackingGoal.h"
#include "MocoInverse.h"
#include "MocoParameter.h"
#include "MocoProblem.h"
#include "MocoStudy.h"
#include "MocoTrack.h"
#include "MocoTropterSolver.h"
#include "MocoWeightSet.h"
#include "ModelOperators.h"
#include <exception>
#include <iostream>

#include <OpenSim/Simulation/MarkersReference.h>

using namespace OpenSim;

static osimMocoInstantiator instantiator;

OSIMMOCO_API void RegisterTypes_osimMoco() {
    try {
        Object::registerType(MocoFinalTimeGoal());
        Object::registerType(MocoWeight());
        Object::registerType(MocoWeightSet());
        Object::registerType(MocoStateTrackingGoal());
        Object::registerType(MocoMarkerTrackingGoal());
        Object::registerType(MocoMarkerFinalGoal());
        Object::registerType(MocoControlGoal());
        Object::registerType(MocoSumSquaredStateGoal());
        Object::registerType(MocoControlTrackingGoal());
        Object::registerType(MocoInitialActivationGoal());
        Object::registerType(MocoJointReactionGoal());
        Object::registerType(MocoOrientationTrackingGoal());
        Object::registerType(MocoTranslationTrackingGoal());
        Object::registerType(MocoPeriodicityGoalPair());
        Object::registerType(MocoPeriodicityGoal());
        Object::registerType(MocoBounds());
        Object::registerType(MocoInitialBounds());
        Object::registerType(MocoFinalBounds());
        Object::registerType(MocoVariableInfo());
        Object::registerType(MocoParameter());
        Object::registerType(MocoPhase());
        Object::registerType(MocoProblem());
        Object::registerType(MocoStudy());

        Object::registerType(MocoInverse());
        Object::registerType(MocoTrack());

        Object::registerType(MocoTropterSolver());

        Object::registerType(MocoControlBoundConstraint());

        Object::registerType(MocoCasADiSolver());

        Object::registerType(ActivationCoordinateActuator());

#ifdef MOCO_WITH_TROPTER
        Object::registerType(GlobalStaticOptimization());
        Object::registerType(INDYGO());
#endif

        Object::registerType(TableProcessor());

        Object::registerType(TabOpLowPassFilter());
        Object::registerType(TabOpUseAbsoluteStateNames());

        Object::registerType(ModelProcessor());
        Object::registerType(ModOpReplaceMusclesWithDeGrooteFregly2016());
        Object::registerType(ModOpIgnoreActivationDynamics());
        Object::registerType(ModOpIgnoreTendonCompliance());
        Object::registerType(ModOpAddReserves());
        Object::registerType(ModOpAddExternalLoads());
        Object::registerType(ModOpIgnorePassiveFiberForcesDGF());
        Object::registerType(ModOpScaleActiveFiberForceCurveWidthDGF());
        Object::registerType(ModOpReplaceJointsWithWelds());
        Object::registerType(ModOpScaleMaxIsometricForce());

        Object::registerType(AckermannVanDenBogert2010Force());
        Object::registerType(MeyerFregly2016Force());
        Object::registerType(EspositoMiller2018Force());
        Object::registerType(PositionMotion());
        Object::registerType(DeGrooteFregly2016Muscle());
        Object::registerType(SmoothSphereHalfSpaceForce());
        Object::registerType(MultivariatePolynomialFunction());

        Object::registerType(DiscreteForces());
        Object::registerType(AccelerationMotion());

        // TODO: Move to osimSimulation.
        Object::registerType(MarkersReference());
        Object::registerType(MarkerWeight());
        Object::registerType(Set<MarkerWeight>());
    } catch (const std::exception& e) {
        std::cerr << "ERROR during osimMoco Object registration:\n"
                  << e.what() << std::endl;
    }
}

osimMocoInstantiator::osimMocoInstantiator() { registerDllClasses(); }

void osimMocoInstantiator::registerDllClasses() { RegisterTypes_osimMoco(); }
