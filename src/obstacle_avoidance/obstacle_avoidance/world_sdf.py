"""Generate the Gazebo Harmonic world from ``obstacle_map.OBSTACLES``.

Keeping SDF generation in Python means the walls the planner knows about are
the same walls Gazebo collides with.
"""

from __future__ import annotations

from pathlib import Path

from obstacle_avoidance.obstacle_map import OBSTACLES, BoxObstacle


def _box_model(box: BoxObstacle) -> str:
    r, g, b, a = box.color
    return f"""    <model name="{box.name}">
      <static>true</static>
      <pose>{box.cx} {box.cy} {box.cz} {box.roll} {box.pitch} {box.yaw}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{box.sx} {box.sy} {box.sz}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{box.sx} {box.sy} {box.sz}</size></box>
          </geometry>
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def _vehicle_model() -> str:
    """Simple differential-drive box. Geometry is a half-scale of the
    official gz-sim ``diff_drive`` demo so physics already behave.
    """
    return """    <model name="vehicle">
      <pose>0 0 0.1625 0 0 0</pose>
      <link name="chassis">
        <pose>-0.0757 0 0.0875 0 0 0</pose>
        <inertial>
          <mass>1.14</mass>
          <inertia>
            <ixx>0.016</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.052</iyy>
            <iyz>0</iyz>
            <izz>0.060</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <box><size>1.006 0.50 0.284</size></box>
          </geometry>
          <material>
            <ambient>0.15 0.45 0.95 1</ambient>
            <diffuse>0.15 0.45 0.95 1</diffuse>
            <specular>0.1 0.1 0.3 1</specular>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box><size>1.006 0.50 0.284</size></box>
          </geometry>
        </collision>
      </link>

      <link name="left_wheel">
        <pose>0.277 0.313 -0.0125 -1.5707 0 0</pose>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.009</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.009</iyy>
            <iyz>0</iyz>
            <izz>0.008</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <sphere><radius>0.15</radius></sphere>
          </geometry>
          <material>
            <ambient>0.12 0.12 0.12 1</ambient>
            <diffuse>0.12 0.12 0.12 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <sphere><radius>0.15</radius></sphere>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
                <slip1>0.035</slip1>
                <slip2>0</slip2>
                <fdir1>0 0 1</fdir1>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>

      <link name="right_wheel">
        <pose>0.277 -0.313 -0.0125 -1.5707 0 0</pose>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.009</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.009</iyy>
            <iyz>0</iyz>
            <izz>0.008</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <sphere><radius>0.15</radius></sphere>
          </geometry>
          <material>
            <ambient>0.12 0.12 0.12 1</ambient>
            <diffuse>0.12 0.12 0.12 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <sphere><radius>0.15</radius></sphere>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
                <slip1>0.035</slip1>
                <slip2>0</slip2>
                <fdir1>0 0 1</fdir1>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>

      <link name="caster">
        <pose>-0.479 0 -0.0625 0 0 0</pose>
        <inertial>
          <mass>0.4</mass>
          <inertia>
            <ixx>0.004</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.004</iyy>
            <iyz>0</iyz>
            <izz>0.004</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <sphere><radius>0.10</radius></sphere>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.2 1</ambient>
            <diffuse>0.2 0.2 0.2 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <sphere><radius>0.10</radius></sphere>
          </geometry>
        </collision>
      </link>

      <joint name="left_wheel_joint" type="revolute">
        <parent>chassis</parent>
        <child>left_wheel</child>
        <axis>
          <xyz>0 0 1</xyz>
          <limit>
            <lower>-1.79769e+308</lower>
            <upper>1.79769e+308</upper>
          </limit>
        </axis>
      </joint>
      <joint name="right_wheel_joint" type="revolute">
        <parent>chassis</parent>
        <child>right_wheel</child>
        <axis>
          <xyz>0 0 1</xyz>
          <limit>
            <lower>-1.79769e+308</lower>
            <upper>1.79769e+308</upper>
          </limit>
        </axis>
      </joint>
      <joint name="caster_wheel" type="ball">
        <parent>chassis</parent>
        <child>caster</child>
      </joint>

      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.626</wheel_separation>
        <wheel_radius>0.15</wheel_radius>
        <topic>/cmd_vel</topic>
        <odom_topic>/odom</odom_topic>
        <tf_topic>/tf</tf_topic>
        <frame_id>odom</frame_id>
        <child_frame_id>base_link</child_frame_id>
        <odom_publish_frequency>50</odom_publish_frequency>
        <max_linear_acceleration>2</max_linear_acceleration>
        <min_linear_acceleration>-2</min_linear_acceleration>
        <max_angular_acceleration>3</max_angular_acceleration>
        <min_angular_acceleration>-3</min_angular_acceleration>
        <max_linear_velocity>0.8</max_linear_velocity>
        <min_linear_velocity>-0.8</min_linear_velocity>
        <max_angular_velocity>1.5</max_angular_velocity>
        <min_angular_velocity>-1.5</min_angular_velocity>
      </plugin>
    </model>
"""


def generate_world_sdf() -> str:
    obstacles_xml = "\n".join(_box_model(box) for box in OBSTACLES)
    return f"""<?xml version="1.0" ?>
<!-- Generated by obstacle_avoidance.world_sdf — do not hand-edit obstacle poses.
     Change obstacle_map.OBSTACLES and regenerate. -->
<sdf version="1.8">
  <world name="obstacle_course">
    <physics name="1ms" type="ignored">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <ambient>0.4 0.4 0.45 1</ambient>
      <background>0.65 0.72 0.80 1</background>
      <shadows>true</shadows>
    </scene>

    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.6 0.6 0.6</ambient_light>
        <background_color>0.65 0.72 0.80</background_color>
        <camera_pose>-14 -14 16 0 0.55 0.78</camera_pose>
      </plugin>
      <plugin filename="EntityContextMenuPlugin" name="Entity context menu">
        <gz-gui>
          <property key="state" type="string">floating</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="InteractiveViewControl" name="Interactive view control">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="WorldControl" name="World control">
        <gz-gui>
          <title>World control</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">72</property>
          <property type="double" key="width">121</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="left" target="left"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <play_pause>true</play_pause>
        <step>true</step>
        <start_paused>false</start_paused>
        <use_event>true</use_event>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <gz-gui>
          <title>World stats</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">110</property>
          <property type="double" key="width">290</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <sim_time>true</sim_time>
        <real_time>true</real_time>
        <real_time_factor>true</real_time_factor>
        <iterations>true</iterations>
      </plugin>
      <plugin filename="EntityTree" name="Entity tree">
        <gz-gui>
          <property type="string" key="state">docked_collapsed</property>
        </gz-gui>
      </plugin>
    </gui>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>40 40</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>40 40</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.62 0.70 0.55 1</ambient>
            <diffuse>0.62 0.70 0.55 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

{obstacles_xml}
{_vehicle_model()}
  </world>
</sdf>
"""


def world_path() -> Path:
    return Path(__file__).resolve().parents[1] / "worlds" / "obstacle_course.sdf"


def write_world(path: Path | None = None) -> Path:
    dest = path or world_path()
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(generate_world_sdf())
    return dest
