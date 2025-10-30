#!/usr/bin/env python3

import time
import numpy as np

import rclpy
from rclpy.node import Node

# (kept to preserve your structure; not used)
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, RobotState
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

# ============================================================
# UDP listener globals (kept to preserve structure; not used here)
hand_pos = None
larm_pos = None
uarm_pos = None
hand_rot = None
larm_rot = None
uarm_rot = None
hips_rot = None

# ============================================================
# Helpers / config (your values)
SHOULDER_ANCHOR = np.array([0.045, 0.32, 1.526], dtype=float)

def remap_watch_to_base(p):
    """Map watch (x,y,z) -> robot base (x,y,z): (z, -x, y)."""
    x, y, z = map(float, p)
    return np.array([z, -x, y], dtype=float)


class MoveLeftArmClient(Node):
    def __init__(self):
        super().__init__('move_left_drumstick_tip_client')

        # --- Load robot model for Pinocchio
        urdf_path = "/home/thibaut/Documents/Bimanual_Robot/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_description/urdf/robots/bimanualrobot.urdf"
        self.robot: RobotWrapper = RobotWrapper.BuildFromURDF(urdf_path, [])
        self.model: pin.Model = self.robot.model
        self.data: pin.Data = self.model.createData()

        # Frames we care about
        self.F_ELBOW = "leftarm_wrist_1_link"   # "elbow" location in your naming
        self.F_WRIST = "leftarm_wrist_2_link"   # wrist

        # Joints we allow to move for IK
        self.IK_JOINTS = [
            "leftarm_shoulder_pan_joint",
            "leftarm_shoulder_lift_joint",
            "leftarm_elbow_joint",
            "leftarm_wrist_1_joint",
        ]

        # Resolve frame and joint indices
        self.fid_elbow = self.model.getFrameId(self.F_ELBOW)
        self.fid_wrist = self.model.getFrameId(self.F_WRIST)

        self.idx_q_vars = [self.model.joints[self.model.getJointId(n)].idx_q for n in self.IK_JOINTS]
        self.idx_v_vars = [self.model.joints[self.model.getJointId(n)].idx_v for n in self.IK_JOINTS]
        self.lb = np.array([self.model.lowerPositionLimit[i] for i in self.idx_q_vars])
        self.ub = np.array([self.model.upperPositionLimit[i] for i in self.idx_q_vars])

        # Measure L1/L2 from the actual model (robust)
        self.L1, self.L2 = self._measure_link_lengths()
        self.get_logger().info(f"L1={self.L1:.4f} m, L2={self.L2:.4f} m")

        # Run one demo solve
        self.demo_once()

    # ----------------- geometry helpers -----------------
    def _measure_link_lengths(self):
        """Measure shoulder→elbow (L1) and elbow→wrist (L2) from model neutral."""
        q0 = pin.neutral(self.model)
        pin.forwardKinematics(self.model, self.data, q0)
        pin.updateFramePlacements(self.model, self.data)

        # Shoulder reference frame: use upper arm link if available
        shoulder_ref = "leftarm_forearm_link" # or "leftarm_upper_arm_link"
        if not self.model.existFrame(shoulder_ref):
            shoulder_ref = "leftarm_shoulder_link"
        fid_sh = self.model.getFrameId(shoulder_ref)

        S = self.data.oMf[fid_sh].translation.copy()
        E = self.data.oMf[self.fid_elbow].translation.copy()
        W = self.data.oMf[self.fid_wrist].translation.copy()
        L1 = float(np.linalg.norm(E - S))
        L2 = float(np.linalg.norm(W - E))
        # L12 = float(np.linalg.norm(W - S))
        # print(L1, L2, L12)
        return L1, L2

    def _targets_from_wearable(self, uarm_xyz, larm_xyz, hand_xyz):
        """Build elbow and wrist targets in BASE from wearable points using L1/L2 and anchor."""
        sh = remap_watch_to_base(uarm_xyz)
        el = remap_watch_to_base(larm_xyz)
        wr = remap_watch_to_base(hand_xyz)

        u1 = el - sh; u1 /= (np.linalg.norm(u1) + 1e-9)
        u2 = wr - el; u2 /= (np.linalg.norm(u2) + 1e-9)

        tgt_E = SHOULDER_ANCHOR + self.L1 * u1
        tgt_W = tgt_E + self.L2 * u2
        return tgt_E, tgt_W

    # ----------------- IK (Gauss-Newton LM) -----------------
    def _full_q_from_vars(self, q_vars):
        q = pin.neutral(self.model)
        for v, i in zip(q_vars, self.idx_q_vars):
            q[i] = float(v)
        return q

    def _positions_and_jacobians(self, q):
        # Update kinematics
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        pE = self.data.oMf[self.fid_elbow].translation.copy()
        pW = self.data.oMf[self.fid_wrist].translation.copy()

        # 6xN Jacobians in WORLD; take linear (bottom 3 rows)
        JE6 = pin.computeFrameJacobian(self.model, self.data, q, self.fid_elbow, pin.ReferenceFrame.WORLD)
        JW6 = pin.computeFrameJacobian(self.model, self.data, q, self.fid_wrist, pin.ReferenceFrame.WORLD)
        JE = JE6[3:6, :]
        JW = JW6[3:6, :]

        # Keep only the columns for our IK joints (idx_v)
        JEv = JE[:, self.idx_v_vars]
        JWv = JW[:, self.idx_v_vars]
        return pE, pW, JEv, JWv

    def solve_ik_elbow_wrist(self, target_E, target_W, x0=None, max_it=80, lm_lambda=1e-4, tol=1e-5):
        if x0 is None:
            x = np.zeros(len(self.idx_q_vars))
        else:
            x = np.array(x0, dtype=float)

        for it in range(max_it):
            q = self._full_q_from_vars(x)
            pE, pW, JE, JW = self._positions_and_jacobians(q)

            r = np.hstack([(pE - target_E), (pW - target_W)])  # 6
            if np.linalg.norm(r) < tol:
                return True, x

            J = np.vstack([JE, JW])  # 6 x nv_sel
            H = J.T @ J + lm_lambda * np.eye(J.shape[1])
            g = J.T @ r
            try:
                dx = -np.linalg.solve(H, g)
            except np.linalg.LinAlgError:
                return False, x

            # Basic clamped update
            xn = np.clip(x + dx, self.lb, self.ub)

            # Simple backtracking if residual not improving
            qn = self._full_q_from_vars(xn)
            pEn, pWn, *_ = self._positions_and_jacobians(qn)
            rn = np.hstack([(pEn - target_E), (pWn - target_W)])
            if np.linalg.norm(rn) < np.linalg.norm(r):
                x = xn
            else:
                # shrink step
                x = np.clip(x + 0.5 * dx, self.lb, self.ub)

        return False, x

    # ----------------- Demo -----------------
    def demo_once(self):
        # One test triple (use your real sample if you want)
        uarm =  (-0.169548898935318, 0.43098410964012146,  0.018846701830625534)
        larm =  (-0.42159175872802734, 0.41295209527015686, -0.04238429665565491)
        hand =  (-0.6245307326316833, 0.40523090958595276, -0.1269783228635788)

        tgtE, tgtW = self._targets_from_wearable(uarm, larm, hand)
        self.get_logger().info(f"Targets:\n  elbow {np.round(tgtE,3)}\n  wrist {np.round(tgtW,3)}")

        ok, qvars = self.solve_ik_elbow_wrist(tgtE, tgtW, x0=None)
        if not ok:
            self.get_logger().warn("IK did not converge.")
        else:
            # Report solution
            for jn, v in zip(self.IK_JOINTS, qvars):
                self.get_logger().info(f"{jn:30s} = {v:+.6f} rad")

            # Check achieved errors
            q_sol = self._full_q_from_vars(qvars)
            pin.forwardKinematics(self.model, self.data, q_sol)
            pin.updateFramePlacements(self.model, self.data)
            pE = self.data.oMf[self.fid_elbow].translation
            pW = self.data.oMf[self.fid_wrist].translation
            eE = np.linalg.norm(pE - tgtE)
            eW = np.linalg.norm(pW - tgtW)
            self.get_logger().info(f"Achieved errors: elbow={eE*1000:.2f} mm, wrist={eW*1000:.2f} mm")


# ============================================================
if __name__ == "__main__":
    rclpy.init()
    node = MoveLeftArmClient()
    # No spin: this script just runs one IK solve and exits.
    node.destroy_node()
    rclpy.shutdown()
