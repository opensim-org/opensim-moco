#ifndef MOCO_BHARGAVA2004METABOLICS_H
#define MOCO_BHARGAVA2004METABOLICS_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: Bhargava2004Metabolics.h                                     *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Antoine Falisse                                                 *
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

#include "../osimMocoDLL.h"
#include <unordered_map>

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/Probe.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Common/Set.h>

namespace OpenSim {

class Muscle;

// Helper classes defined below.
class Bhargava2004MuscleSmoothMetabolicsProbe_MetabolicMuscleParameter;
class Bhargava2004MuscleSmoothMetabolicsProbe_MetabolicMuscleParameterSet;

class OSIMMOCO_API Bhargava2004MuscleSmoothMetabolicsProbe : public Probe {
OpenSim_DECLARE_CONCRETE_OBJECT(Bhargava2004MuscleSmoothMetabolicsProbe, Probe);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(activation_rate_on,
        bool,
        "Specify whether activation heat rate is to be calculated (true/false).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(maintenance_rate_on,
        bool,
        "Specify whether maintenance heat rate is to be calculated (true/false).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(shortening_rate_on,
        bool,
        "Specify whether shortening heat rate is to be calculated (true/false).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(basal_rate_on,
        bool,
        "Specify whether basal heat rate is to be calculated (true/false).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(mechanical_work_rate_on,
        bool,
        "Specify whether mechanical work rate is to be calculated (true/false).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(enforce_minimum_heat_rate_per_muscle,
        bool,
        "Specify whether the total heat rate for a muscle will be clamped to a "
        "minimum value of 1.0 W/kg (true/false).");

    /** Default curve shown in doxygen. **/
    OpenSim_DECLARE_PROPERTY(normalized_fiber_length_dependence_on_maintenance_rate,
        PiecewiseLinearFunction,
        "Contains a PiecewiseLinearFunction object that describes the "
        "normalized fiber length dependence on maintenance rate.");

    /** Disabled by default. **/
    OpenSim_DECLARE_PROPERTY(use_force_dependent_shortening_prop_constant,
        bool,
        "Specify whether to use a force dependent shortening proportionality "
        "constant (true/false).");

    /** Default value = 1.2. **/
    OpenSim_DECLARE_PROPERTY(basal_coefficient,
        double,
        "Basal metabolic coefficient.");

    /** Default value = 1.0. **/
    OpenSim_DECLARE_PROPERTY(basal_exponent,
        double,
        "Basal metabolic exponent.");

    /** Default value = 1.0. **/
    OpenSim_DECLARE_PROPERTY(muscle_effort_scaling_factor,
        double,
        "Scale the excitation and activation values used by the probe to "
        "compensate for solutions with excessive coactivation (e.g., when a "
        "suboptimal tracking strategy is used).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(include_negative_mechanical_work,
        bool,
        "Specify whether negative mechanical work will be included in Wdot "
        "(true/false).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(forbid_negative_total_power,
        bool,
        "Specify whether the total power for each muscle must remain positive "
        "(true/false).");

    /** Default value = true **/
    OpenSim_DECLARE_PROPERTY(report_total_metabolics_only,
        bool,
        "If set to false, the individual muscle metabolics, basal rate, and "
        "total summation will be reported. If set to true, only the total "
        "summation will be reported.");

    OpenSim_DECLARE_UNNAMED_PROPERTY(
        Bhargava2004MuscleSmoothMetabolicsProbe_MetabolicMuscleParameterSet,
        "A set containing, for each muscle, the parameters "
        "required to calculate muscle metabolic power.");

//=============================================================================
// PUBLIC METHODS
//=============================================================================
    /** MuscleMap typedef */
    typedef std::map
       <std::string,
       Bhargava2004MuscleSmoothMetabolicsProbe_MetabolicMuscleParameter*>
       MuscleMap;

    //--------------------------------------------------------------------------
    // Constructor(s) and Setup
    //--------------------------------------------------------------------------
    /** Default constructor */
    Bhargava2004MuscleSmoothMetabolicsProbe();

    /** Convenience constructor */
    Bhargava2004MuscleSmoothMetabolicsProbe(
        const bool activation_rate_on,
        const bool maintenance_rate_on,
        const bool shortening_rate_on,
        const bool basal_rate_on,
        const bool work_rate_on);



    //-----------------------------------------------------------------------------
    // Computation
    //-----------------------------------------------------------------------------
    /** Compute muscle metabolic power. */
    virtual SimTK::Vector computeProbeInputs(const SimTK::State& state) const override;

    /** Returns the number of probe inputs in the vector returned by computeProbeInputs(). */
    int getNumProbeInputs() const override;

    /** Returns the column labels of the probe values for reporting.
        Currently uses the Probe name as the column label, so be sure
        to name your probe appropriately!*/
    virtual OpenSim::Array<std::string> getProbeOutputLabels() const override;



    //-----------------------------------------------------------------------------
    /** @name     Bhargava2004MuscleSmoothMetabolicsProbe Interface
    These accessor methods are to be used when setting up a new muscle
    metabolic analysis from the API. The basic operation is as follows:
    @code
    Bhargava2004MuscleSmoothMetabolicsProbe* myProbe new Bhargava2004MuscleSmoothMetabolicsProbe(...);
    model.addProbe(myProbe);
    myProbe->addMuscle("muscleName1", ... );
    myProbe->addMuscle("muscleName2", ... );
    myProbe->addMuscle("muscleName3", ... );
    myProbe->useProvidedMass("muscleName1", 1.2);         // muscle1 mass = 1.2 kg
    myProbe->useCalculatedMass("muscleName2");            // muscle2 mass is based on muscle properties (below)
    myProbe->setDensity("muscleName2", 1100.0);           // muscle2 density is 1100 kg/m^3
    myProbe->setSpecificTension("muscleName2", 0.26e6);   // muscle2 specific tension is 0.26e6 Pa
    myProbe->removeMuscle("muscleName3");
    myProbe->setOperation("integrate")           // See OpenSim::Probe for other operations
    @endcode
    @note It is important to first add the metabolic probe to the model before
    calling any other methods that may modify its properties. This is because
    some methods (e.g. addMuscle() or useCalculatedMass) may require information
    about the muscles to successfully execute, and this information can only be
    obtained if the metabolic probe is already 'connected' to the model.
    */
    // Get the number of muscles being analyzed in the metabolic analysis. */
    const int getNumMetabolicMuscles() const;

    /** Add a muscle and its parameters so that it can be included in the metabolic analysis. */
    void addMuscle(const std::string& muscleName,
        double ratio_slow_twitch_fibers,
        double activation_constant_slow_twitch,
        double activation_constant_fast_twitch,
        double maintenance_constant_slow_twitch,
        double maintenance_constant_fast_twitch);

    /** Add a muscle and its parameters so that it can be included in the metabolic analysis. */
    void addMuscle(const std::string& muscleName,
        double ratio_slow_twitch_fibers,
        double activation_constant_slow_twitch,
        double activation_constant_fast_twitch,
        double maintenance_constant_slow_twitch,
        double maintenance_constant_fast_twitch,
        double muscle_mass);

    /** Remove a muscle from the metabolic analysis. */
    void removeMuscle(const std::string& muscleName);

    /** %Set an existing muscle to use a provided muscle mass. */
    void useProvidedMass(const std::string& muscleName, double providedMass);

    /** %Set an existing muscle to calculate its own mass. */
    void useCalculatedMass(const std::string& muscleName);

    /** Get whether the muscle mass is being explicitly provided.
       True means that it is using the property 'provided_muscle_mass'
       False means that the muscle mass is being calculated from muscle properties. */
    bool isUsingProvidedMass(const std::string& muscleName);

    /** Get the muscle mass used in the metabolic analysis. The value
        returned will depend on if the muscle mass is explicitly provided
        (i.e. isUsingProvidedMass = true), or if it is being automatically
        calculated from muscle data already present in the model
        (i.e. isUsingProvidedMass = true). */
    const double getMuscleMass(const std::string& muscleName) const;

    /** Get the ratio of slow twitch fibers for an existing muscle. */
    const double getRatioSlowTwitchFibers(const std::string& muscleName) const;

    /** %Set the ratio of slow twitch fibers for an existing muscle. */
    void setRatioSlowTwitchFibers(const std::string& muscleName, const double& ratio);

    /** Get the density for an existing muscle (kg/m^3). */
    const double getDensity(const std::string& muscleName) const;

    /** %Set the density for an existing muscle (kg/m^3). */
    void setDensity(const std::string& muscleName, const double& density);

    /** Get the specific tension for an existing muscle (Pascals (N/m^2)). */
    const double getSpecificTension(const std::string& muscleName) const;

    /** %Set the specific tension for an existing muscle (Pascals (N/m^2)). */
    void setSpecificTension(const std::string& muscleName, const double& specificTension);

    /** Get the activation constant for slow twitch fibers for an existing muscle. */
    const double getActivationConstantSlowTwitch(const std::string& muscleName) const;

    /** %Set the activation constant for slow twitch fibers for an existing muscle. */
    void setActivationConstantSlowTwitch(const std::string& muscleName, const double& c);

    /** Get the activation constant for fast twitch fibers for an existing muscle. */
    const double getActivationConstantFastTwitch(const std::string& muscleName) const;

    /** %Set the activation constant for fast twitch fibers for an existing muscle. */
    void setActivationConstantFastTwitch(const std::string& muscleName, const double& c);

    /** Get the maintenance constant for slow twitch fibers for an existing muscle. */
    const double getMaintenanceConstantSlowTwitch(const std::string& muscleName) const;

    /** %Set the maintenance constant for slow twitch fibers for an existing muscle. */
    void setMaintenanceConstantSlowTwitch(const std::string& muscleName, const double& c);

    /** Get the maintenance constant for fast twitch fibers for an existing muscle. */
    const double getMaintenanceConstantFastTwitch(const std::string& muscleName) const;

    /** %Set the maintenance constant for fast twitch fibers for an existing muscle. */
    void setMaintenanceConstantFastTwitch(const std::string& muscleName, const double& c);




//==============================================================================
// PRIVATE
//==============================================================================
private:
    //--------------------------------------------------------------------------
    // Data
    //--------------------------------------------------------------------------
    MuscleMap _muscleMap;


    //--------------------------------------------------------------------------
    // ModelComponent Interface
    //--------------------------------------------------------------------------
    void extendConnectToModel(Model& aModel) override;
    void connectIndividualMetabolicMuscle(Model& aModel,
        Bhargava2004MuscleSmoothMetabolicsProbe_MetabolicMuscleParameter& mm);

    void setNull();
    void constructProperties();


    //--------------------------------------------------------------------------
    // MetabolicMuscleParameter Private Interface
    //--------------------------------------------------------------------------
    // Get const MetabolicMuscleParameter from the MuscleMap using a string accessor.
    const Bhargava2004MuscleSmoothMetabolicsProbe_MetabolicMuscleParameter*
        getMetabolicParameters(const std::string& muscleName) const;

    // Get writable MetabolicMuscleParameter from the MuscleMap using a string accessor.
    Bhargava2004MuscleSmoothMetabolicsProbe_MetabolicMuscleParameter*
        updMetabolicParameters(const std::string& muscleName);

//=============================================================================
};  // END of class Bhargava2004MuscleSmoothMetabolicsProbe
//=============================================================================

//==============================================================================
//==============================================================================
//          Bhargava2004MuscleSmoothMetabolicsProbe_MetabolicMuscleParameter
//==============================================================================

/**
 * Documentation for this class has been provided with the documentation for the
 * Bhargava2004MuscleSmoothMetabolicsProbe class.
 *
 * @see Bhargava2004MuscleSmoothMetabolicsProbe
 */

class OSIMMOCO_API
    Bhargava2004MuscleSmoothMetabolicsProbe_MetabolicMuscleParameter
    : public Object
{
    OpenSim_DECLARE_CONCRETE_OBJECT(
        Bhargava2004MuscleSmoothMetabolicsProbe_MetabolicMuscleParameter, Object);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(specific_tension, double,
        "The specific tension of the muscle (Pascals (N/m^2)).");

    OpenSim_DECLARE_PROPERTY(density, double,
        "The density of the muscle (kg/m^3).");

    OpenSim_DECLARE_PROPERTY(ratio_slow_twitch_fibers, double,
        "Ratio of slow twitch fibers in the muscle (must be between 0 and 1).");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(use_provided_muscle_mass, bool,
        "An optional flag that allows the user to explicitly specify a muscle mass. "
        "If set to true, the 'provided_muscle_mass' property must be specified.");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(provided_muscle_mass, double,
        "The user specified muscle mass (kg).");

    OpenSim_DECLARE_PROPERTY(activation_constant_slow_twitch, double,
        "Activation constant for slow twitch fibers (W/kg).");

    OpenSim_DECLARE_PROPERTY(activation_constant_fast_twitch, double,
        "Activation constant for fast twitch fibers (W/kg).");

    OpenSim_DECLARE_PROPERTY(maintenance_constant_slow_twitch, double,
        "Maintenance constant for slow twitch fibers (W/kg).");

    OpenSim_DECLARE_PROPERTY(maintenance_constant_fast_twitch, double,
        "Maintenance constant for fast twitch fibers (W/kg).");

//=============================================================================
// DATA
//=============================================================================
// These private member variables are kept here because they apply to
// a single muscle, but are not set in this class -- rather, they are
// set by the probes that own them.
protected:
    Muscle* _musc;          // Internal pointer to the muscle that corresponds
                            // to these parameters.
    double _muscMass;       // The mass of the muscle (depends on if
                            // <use_provided_muscle_mass> is true or false.



//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // Constructor(s)
    //--------------------------------------------------------------------------
    Bhargava2004MuscleSmoothMetabolicsProbe_MetabolicMuscleParameter();

    Bhargava2004MuscleSmoothMetabolicsProbe_MetabolicMuscleParameter(
            const std::string& muscleName,
            double ratio_slow_twitch_fibers,
            double muscle_mass = SimTK::NaN);

    Bhargava2004MuscleSmoothMetabolicsProbe_MetabolicMuscleParameter(
        const std::string& muscleName,
        double ratio_slow_twitch_fibers,
        double activation_constant_slow_twitch,
        double activation_constant_fast_twitch,
        double maintenance_constant_slow_twitch,
        double maintenance_constant_fast_twitch,
        double muscle_mass = SimTK::NaN);


    // Uses default (compiler-generated) destructor, copy constructor, copy
    // assignment operator.


    //--------------------------------------------------------------------------
    // Muscle mass
    //--------------------------------------------------------------------------
    const double getMuscleMass() const      { return _muscMass; }
    void setMuscleMass();



    //--------------------------------------------------------------------------
    // Internal muscle pointer
    //--------------------------------------------------------------------------
    const Muscle* getMuscle() const         { return _musc; }
    void setMuscle(Muscle* m)               { _musc = m; }



private:
    //--------------------------------------------------------------------------
    // Object Interface
    //--------------------------------------------------------------------------
    void setNull();
    void constructProperties();

//=============================================================================
};  // END of class MetabolicMuscleParameter
//=============================================================================





//==============================================================================
//==============================================================================
//==============================================================================
//                          MetabolicMuscleParameterSet
//==============================================================================
/**
 * MetabolicMuscleParameterSet is a class that holds the set of
 * MetabolicMuscleParameters for each muscle.
 */
class OSIMMOCO_API
    Bhargava2004MuscleSmoothMetabolicsProbe_MetabolicMuscleParameterSet
    : public Set<Bhargava2004MuscleSmoothMetabolicsProbe_MetabolicMuscleParameter>
{
    OpenSim_DECLARE_CONCRETE_OBJECT(
        Bhargava2004MuscleSmoothMetabolicsProbe_MetabolicMuscleParameterSet,
        Set<Bhargava2004MuscleSmoothMetabolicsProbe_MetabolicMuscleParameter>);

public:
    Bhargava2004MuscleSmoothMetabolicsProbe_MetabolicMuscleParameterSet()
    {  }


//=============================================================================
};  // END of class MetabolicMuscleParameterSet
//=============================================================================


}; //namespace
//=============================================================================
//=============================================================================

#endif // MOCO_BHARGAVA2004METABOLICS_H