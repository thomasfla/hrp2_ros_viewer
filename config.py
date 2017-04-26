from IPython import embed
MESH_PATH = '/opt/openrobots/share'
HRP2_URDF_MODEL = "/opt/openrobots/share/hrp2_14_description/urdf/hrp2_14_reduced.urdf"
READ_FREEFLYER_FROM_AA_SIGNAL = True

CONTACT_FORCE_ARROW_RADIUS = 0.01
CONTACT_FORCE_ARROW_SCALE = 0.005
CONTACT_FORCE_ARROW_COLOR = (1, 0, 0, 1);

DISPLAY_COP_RL      = False
DISPLAY_COP         = False
DISPLAY_COM         = False
DISPLAY_FEET_FORCES = False

DISPLAY_MOCAP_CHEST = True

TOPIC_FORCES_RF = "estimator_contactWrenchRightSole"
TOPIC_FORCES_LF = "estimator_contactWrenchLeftSole"

TOPIC_MOCAP_CHEST = "/evart/hrp2_14_chest/hrp2_14_chest"

TOPIC_FF    = "floatingBase_pos"
TOPIC_CoP_R = "inv_dyn_cop_R"
TOPIC_CoP_L = "inv_dyn_cop_L"
TOPIC_CoP   = "inv_dyn_cop"
TOPIC_CoM   = "inv_dyn_come"
