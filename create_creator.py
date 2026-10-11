# make_crater_world.py
import math, random

random.seed(42)

N = 200  # grid points per side
SIZE = 100.0  # world size in meters
SP = SIZE / (N - 1)
NUM_CRATERS = 12
# hills: (cx, cy, radius, height)
hills = [
    (
        random.uniform(10, SIZE - 10),
        random.uniform(10, SIZE - 10),
        random.uniform(8, 15),
        random.uniform(2, 5),
    )
    for _ in range(6)
]

# sharp pits: (cx, cy, radius, depth)
pits = [
    (
        random.uniform(10, SIZE - 10),
        random.uniform(10, SIZE - 10),
        random.uniform(1.5, 2.5),
        random.uniform(2, 3.5),
    )
    for _ in range(10)
]

# cliff mesas: (cx, cy, angle, half_width, half_length, drop)
cliffs = [
    (
        random.uniform(20, SIZE - 20),
        random.uniform(20, SIZE - 20),
        random.uniform(0, math.pi),
        random.uniform(5, 10),
        random.uniform(8, 15),
        random.uniform(2, 4),
    )
    for _ in range(3)
]
# (cx, cy, radius, depth, rim_height)
craters = []
for _ in range(NUM_CRATERS):
    R = random.uniform(3, 12)
    craters.append(
        (
            random.uniform(10, SIZE - 10),
            random.uniform(10, SIZE - 10),
            R,
            R * random.uniform(0.15, 0.25),  # depth ~ 15-25% of radius
            R * random.uniform(0.03, 0.06),  # rim height
        )
    )


def height(x, y):
    h = 0.0
    for cx, cy, R, d, rim in craters:
        r = math.hypot(x - cx, y - cy)
        if r < R:
            h -= d * (1 - (r / R) ** 2)  # parabolic bowl
        h += rim * math.exp(-(((r - R) / (0.25 * R)) ** 2))  # raised rim
    for cx, cy, R, H in hills:
        h += H * math.exp(-((x - cx) ** 2 + (y - cy) ** 2) / (R * R))
    for cx, cy, R, D in pits:
        r = math.hypot(x - cx, y - cy)
        h -= D * math.exp(-((r / R) ** 4))  # power 4 gives steep walls
    for cx, cy, a, W, L, drop in cliffs:
        dx, dy = x - cx, y - cy
        across = dx * math.cos(a) + dy * math.sin(a)
        along = -dx * math.sin(a) + dy * math.cos(a)
        h += (
            drop
            * 0.5
            * (1 + math.tanh((W - abs(across)) / 0.4))
            * 0.5
            * (1 + math.tanh((L - abs(along)) / 0.4))
        )
    # small roughness
    h += 0.02 * math.sin(x * 1.3) * math.cos(y * 1.7)
    return h


heights = [height(i * SP, j * SP) for j in range(N) for i in range(N)]

with open("crater_world.wbt", "w") as f:
    f.write(f"""#VRML_SIM R2025a utf8

WorldInfo {{
  basicTimeStep 16
}}
Viewpoint {{
  orientation -0.3 0.3 0.9 1.6
  position -20 -20 30
}}
TexturedBackground {{
}}
TexturedBackgroundLight {{
}}
Solid {{
  translation {-SIZE/2} {-SIZE/2} 0
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.55 0.53 0.5
        roughness 1
        metalness 0
      }}
      geometry DEF CRATERS ElevationGrid {{
        xDimension {N}
        xSpacing {SP}
        yDimension {N}
        ySpacing {SP}
        height [
          {" ".join(f"{v:.4f}" for v in heights)}
        ]
      }}
    }}
  ]
  boundingObject USE CRATERS
}}
Pioneer3at {{
  translation 0 0 1
  controller "rover_sensors"
  extensionSlot [
    Camera {{
      translation 0.25 0 0.3
      name "camera"
      width 640
      height 480
      fieldOfView 1.2
    }}
    SpotLight {{
      location 0.28 0 0.32
      direction 1 0 -0.15
      intensity 8
      beamWidth 0.8
      cutOffAngle 1.0
      radius 40
    }}
    Lidar {{
      translation 0 0 0.45
      name "lidar"
      horizontalResolution 256
      fieldOfView 3.14159
      numberOfLayers 16
      verticalFieldOfView 0.5
      minRange 0.3
      maxRange 30
    }}
    InertialUnit {{
      name "imu"
    }}
    Accelerometer {{
      name "accel"
    }}
    Gyro {{
      name "gyro"
    }}
  ]
}}
""")
print("Wrote crater_world.wbt")
