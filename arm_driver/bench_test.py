import time
from canbus_stepper import CANBus

bus = CANBus("/dev/ttyACM0")
n = bus.node(1)
n.enable(); n.set_current(60)

print("--- idle watch (enabled, no command) ---")
for i in range(15):
    print(f"t={i*0.2:.1f}s angle={n.get_angle():8.2f}  vel={n.get_velocity():8.2f}")
    time.sleep(0.2)

start = n.get_angle()
print(f"--- commanding +90 deg (small move), start={start:.2f} ---")
n.set_position(start + 90)
for i in range(25):
    print(f"t={i*0.2:.1f}s angle={n.get_angle():8.2f}  vel={n.get_velocity():8.2f}")
    time.sleep(0.2)
bus.close()
