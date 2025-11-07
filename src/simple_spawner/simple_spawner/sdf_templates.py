#!/usr/bin/env python3

# Copyright (C) 2025 Clinton Enwerem

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# SDF templates
TEMPLATES = {
    "cube": """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.1 0.1 0.1</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.1 0.1 0.1</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 1 1</ambient> 
          <diffuse>0 0 1 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""",
    "cylinder": """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.1</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.1</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>1 0.647 0 1</ambient> 
          <diffuse>1 0.647 0 1</diffuse> 
          <specular>0.1 0.1 0.1 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""",
    "sphere": """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
      </inertial>
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>0.05</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.05</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0 1 0 1</ambient>
          <diffuse>0 1 0 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
}