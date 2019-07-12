set(EXAMPLE_MOCO_TRACKING_TORQUE_ENDPOINTS_FILES
      gait_2D_contact_torque_bounds_noPelvisRes.osim
      IK_reference_radian.sto)

MocoAddExampleCXX(NAME exampleTrackingTorqueEndpoints
                  RESOURCES "${EXAMPLE_MOCO_TRACKING_TORQUE_ENDPOINTS_FILES}")
