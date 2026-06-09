#!/usr/bin/env python3
"""
Phase 2/3 — Multi-block spawner for fr3_delivery_sim.
Spawns a red, green, and blue cube at random, non-overlapping positions.
"""

import random
import math
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
from ros_gz_interfaces.msg import EntityFactory
from geometry_msgs.msg import Pose

X_MIN, X_MAX =  0.35,  0.60
Y_MIN, Y_MAX = -0.20,  0.20
CUBE_HALF_HEIGHT = 0.025

def make_cube_sdf(name: str, r: float, g: float, b: float) -> str:
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static> <link name="link">
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>4.2e-6</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>4.2e-6</iyy><iyz>0</iyz>
          <izz>4.2e-6</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r*0.9} {g*0.9} {b*0.9} 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

class ObjectSpawner(Node):
    SPAWN_SERVICE = '/world/empty/create'

    def __init__(self):
        super().__init__('object_spawner')
        self.client = self.create_client(SpawnEntity, self.SPAWN_SERVICE)

        self.get_logger().info('Waiting for Gazebo spawn service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'  {self.SPAWN_SERVICE} not ready yet, retrying...')

        self.get_logger().info('Spawn service found — spawning cubes.')
        self._spawn_cubes()

    def _spawn_cubes(self):
        cubes = [
            ('red_cube',   1.0, 0.0, 0.0),
            ('green_cube', 0.0, 1.0, 0.0),
            ('blue_cube',  0.0, 0.0, 1.0)
        ]
        spawned_positions = []

        for name, r, g, b in cubes:
            # Distance check to ensure blocks don't spawn inside each other
            while True:
                x = random.uniform(X_MIN, X_MAX)
                y = random.uniform(Y_MIN, Y_MAX)
                if all(math.hypot(x - px, y - py) > 0.08 for px, py in spawned_positions):
                    spawned_positions.append((x, y))
                    break

            z = CUBE_HALF_HEIGHT
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.w = 1.0

            request = SpawnEntity.Request()
            factory = EntityFactory()
            factory.name = name
            factory.sdf = make_cube_sdf(name, r, g, b)
            factory.pose = pose
            factory.allow_renaming = True
            request.entity_factory = factory

            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result() and future.result().success:
                self.get_logger().info(f'{name} spawned at ({x:.3f}, {y:.3f}, {z})')
            else:
                self.get_logger().error(f'Spawn failed for {name}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectSpawner()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()