#!/usr/bin/env bash
# Prueba directa: avanza ~0.1 m/s durante 3 s y frena. Sin teleop.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/real_env.sh"
python3 - <<'PY'
import time, rclpy
from geometry_msgs.msg import Twist
rclpy.init()
n = rclpy.create_node('tnr_pubtest')
p = n.create_publisher(Twist, '/tb4_2/cmd_vel', 10)
t0 = time.time()
while time.time() - t0 < 1.0:   # dar tiempo al discovery
    rclpy.spin_once(n, timeout_sec=0.1)
print('Subscribers en /tb4_2/cmd_vel:', p.get_subscription_count())
m = Twist(); m.linear.x = 0.1
print('Publicando linear.x = 0.1 durante 3 s...')
t0 = time.time()
while time.time() - t0 < 3.0:
    p.publish(m); time.sleep(0.1)
p.publish(Twist()); time.sleep(0.2)
print('Fin: velocidad 0 enviada')
n.destroy_node(); rclpy.shutdown()
PY
