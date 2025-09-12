#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, AllowedCollisionMatrix

ARM_LINKS = [
    "j2n6s200_link_base",
    "j2n6s200_link_1",
    "j2n6s200_link_2",
    "j2n6s200_link_3",
    "j2n6s200_link_4",
    "j2n6s200_link_5",
    "j2n6s200_link_6",
    "j2n6s200_end_effector",
    "j2n6s200_link_finger_1",
    "j2n6s200_link_finger_2",
    "j2n6s200_link_finger_tip_1",
    "j2n6s200_link_finger_tip_2",
]

ACCESSORY_LINKS = [
    "forkHandle","forkTine","forkTip",
    "cameraMount","camera_link","uncalibrated_camera_link","uncalibrated_camera_bottom_screw_frame",
    "enclosureBottom","enclosureTop",
    "FT","FTSensor","FTMount","FTArmMount",
    "frontStabilizer","nano","nanoMount",
]

class RelaxCollisions(Node):
    def __init__(self):
        super().__init__("relax_collisions")
        self.pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.timer = self.create_timer(0.5, self._tick)
        self.done = False

    def _tick(self):
        if self.done:
            return
        self.done = True
        # Build a symmetric ACM over the union of accessories and arm links.
        # Allow collisions only between accessories <-> arm; everything else stays False.
        all_links = ACCESSORY_LINKS + ARM_LINKS
        split = len(ACCESSORY_LINKS)
        acm = AllowedCollisionMatrix()
        acm.entry_names = list(all_links)
        for i, _ in enumerate(all_links):
            row = [False] * len(all_links)
            if i < split:
                # accessory row: allow with all arm columns
                for j in range(split, len(all_links)):
                    row[j] = True
            else:
                # arm row: allow with all accessory columns
                for j in range(0, split):
                    row[j] = True
            acm.entry_values.append(row)
        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = acm
        self.pub.publish(scene)
        self.get_logger().info("Relaxed collisions between arm and accessories")
        self.create_timer(0.5, lambda: rclpy.shutdown())

def main():
    rclpy.init()
    node = RelaxCollisions()
    rclpy.spin(node)

if __name__ == "__main__":
    main()


