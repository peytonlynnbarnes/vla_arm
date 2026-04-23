"""Emit SDF snippets for a ball model. Used by the spike script to respawn balls cleanly between trials."""

BALL_SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{inertia}</ixx>
          <iyy>{inertia}</iyy>
          <izz>{inertia}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <sphere><radius>{radius}</radius></sphere>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <sphere><radius>{radius}</radius></sphere>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def ball_sdf(name: str, xyz, rgb, radius=0.03, mass=0.05):
    # Sphere inertia: (2/5) m r^2
    inertia = (2.0 / 5.0) * mass * radius * radius
    r, g, b = rgb
    x, y, z = xyz
    return BALL_SDF_TEMPLATE.format(
        name=name, x=x, y=y, z=z, radius=radius,
        mass=mass, inertia=f'{inertia:.8f}',
        r=r, g=g, b=b,
    )
