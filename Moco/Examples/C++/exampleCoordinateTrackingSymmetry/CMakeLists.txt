set(EXAMPLE_MOCO_COORDINATE_TRACKING_SYMMETRY_FILES
      gait10dof18musc_MusclePolynomials_withPassiveForces.osim
      gait10dof18musc_MusclePolynomials_withoutPassiveForces.osim
      referenceCoordinate.sto)

MocoAddExampleCXX(NAME exampleCoordinateTrackingSymmetry
                  RESOURCES "${EXAMPLE_MOCO_COORDINATE_TRACKING_SYMMETRY_FILES}")
