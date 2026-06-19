# Processed SMPL full-body motion frame for the SONIC deploy "pose" (streamed-motion) stream.
#
# Field layout mirrors the gear_sonic pico_manager pose message
# (gear_sonic/scripts/pico_manager_thread_server.py PoseStreamer). One message = one motion frame.
#
# The GearSonicInterface node buffers a sliding window of these frames and forwards them to the
# SONIC deploy stack over ZMQ on the "pose" topic (streamed-motion mode). The deploy's
# streamed_motion_merger requires smpl_pose, smpl_joints, body_quat and frame_index.

std_msgs/Header header

# Monotonically increasing frame counter. The deploy's streamed_motion_merger orders/merges
# frames by this index, so it MUST strictly increase across a motion.
int64 frame_index

# SMPL body pose: 21 joints x 3, axis-angle (radians), row-major [joint][xyz].
float64[63] smpl_pose

# SMPL joint positions: 24 joints x 3, meters, local/root frame, row-major [joint][xyz].
float64[72] smpl_joints

# Global body (root) orientation.
# NOTE: geometry_msgs/Quaternion is (x, y, z, w); the node forwards it to the deploy as
# scalar-first (w, x, y, z), matching the gear_sonic "body_quat_w" field.
geometry_msgs/Quaternion body_quat

# Optional auxiliary body joint position/velocity targets (29 DoF). Fill with zeros if unused;
# the deploy primarily tracks the SMPL pose. Kept for protocol-v2 compatibility.
float64[29] joint_pos
float64[29] joint_vel
