set(EXAMPLE_MOCO_PREDICTIVE_TORQUE_ENDPOINTS_FILES
      gait_2D_contact_torque_bounds_noPelvisRes.osim
      IK_reference_radian.sto
      exampleTrackingTorqueEndpoints_solution.sto)

MocoAddExampleCXX(NAME examplePredictiveTorqueEndpoints
                  RESOURCES "${EXAMPLE_MOCO_PREDICTIVE_TORQUE_ENDPOINTS_FILES}")
