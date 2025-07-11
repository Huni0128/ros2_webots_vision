import math

# 그리퍼을 Open/Closed 로 구분할 임계값 (m 단위)
GRIPPER_OPEN_THRESHOLD = 0.015

# Panda PROTO에서 정의된 관절 한계값 (rad 단위)
_RAD_LIMITS = {
    'panda_joint1': (-2.9671,  2.9671),
    'panda_joint2': (-1.8326,  1.8326),
    'panda_joint3': (-2.9671,  2.9671),
    'panda_joint4': (-3.1416, -0.4000),
    'panda_joint5': (-2.9671,  2.9671),
    'panda_joint6': (-0.0873,  3.8223),
    'panda_joint7': (-2.9671,  2.9671),
}

# 위 한계값을 도(°) 단위로 변환
JOINT_LIMITS_DEG = {
    name: (math.degrees(lo), math.degrees(hi))
    for name, (lo, hi) in _RAD_LIMITS.items()
}
