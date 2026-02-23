#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, termios, tty, json, time, os

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.pub_topside = self.create_publisher(String, '/comm/topside_tx', 10)
        self.sub_rx = self.create_subscription(String, '/comm/topside_rx', self.rx_callback, 10)
        
        self.settings = termios.tcgetattr(sys.stdin)
        self.latest_telemetry = {}
        self.last_dashboard_update = 0
        self.last_packet_time = 0
        
        self.get_logger().info("Teleop Node Started. Waiting for Link...")

    def rx_callback(self, msg):
        try:
            data = json.loads(msg.data)
            # Update time on ANY valid JSON as requested to fix DISCONNECTED flicker
            self.last_packet_time = time.time()
            if isinstance(data, dict):
                 self.latest_telemetry = data
                 # DEBUG STATE
                 # s = data.get("state", "None")
                 # print(f"DEBUG: RX STATE: {s}") 
        except Exception as e:
            pass # Ignore bad JSON

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        key = sys.stdin.read(1) if rlist else ''
        if key == '\x1b': key += sys.stdin.read(2)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def print_dashboard(self):
        if time.time() - self.last_dashboard_update < 0.2: return
        self.last_dashboard_update = time.time()
        
        # Clear Screen
        print("\033[H\033[J", end="")
        
        # Visual Header
        print("\033[1;36m" + "="*60)
        print("          DOOM AUV - MISSION CONTROL INTERFACE")
        print("="*60 + "\033[0m")
        
        # Status Line
        col = self.latest_telemetry.get("targets_collected", 0)
        remaining = 5 - col
        state = self.latest_telemetry.get("state", "STANDBY")
        
        waiting_conf = "\033[1;32mNO \033[0m"
        if state == "WAIT_CONFIRM":
             waiting_conf = "\033[1;31mYES\033[0m"

        if state == "SURFACING":
             print(f"\n \033[1;31;5m⚠ ASCENSO DE EMERGENCIA ⚠\033[0m")
             state = "\033[1;31mEMERGENCY\033[0m"

        print(f" \033[1;35mREMAINING TARGETS:\033[0m {remaining}  |  \033[1;33mWAITING CONFIRMATION:\033[0m {waiting_conf}")
        
        # ALIGNMENT ERROR DISPLAY
        if state == "ALIGN" or state == "WAIT_CONFIRM":
             align_err = self.latest_telemetry.get("align_error", 0.0)
             color = "\033[1;32m" if abs(align_err) < 10.0 else "\033[1;31m"
             print(f" \033[1;36mALIGNMENT ERROR:\033[0m {color}{align_err:.1f}°\033[0m")
             
        print("-" * 60)
        
        # Requested Controls Visualization
        print("\033[1;37m")
        print(" ┌────────────────────── controls ──────────────────────┐")
        print(" │ [W/S] Move Fwd/Back  | [A/D] Turn Left/Right (0.25) │")
        print(" │ [R/F] Depth Up/Down  | [SPACE] EMERGENCY SURFACE    │")
        print(" ├──────────────────────────────────────────────────────┤")
        print(" │ [M]   START AUTO     | [N]     STOP (MANUAL)        │")
        print(" │ [Y]   Confirm Tgt    | [X]     Reject Tgt           │")
        print(" ├──────────────────────────────────────────────────────┤")
        print(" │ [U]   Grip Toggle    | [I/K/J/L] Arm Move           │")
        print(" │ [O/P] Rotate Grip                                   │")
        print(" └──────────────────────────────────────────────────────┘")
        print("\033[0m")
        print(" (Press CTRL+C to Quit)")

    def run(self):
        lin_x, ang_z, lin_z = 0.0, 0.0, 0.0
        last_key_time = 0.0
        
        try:
            while True:
                rclpy.spin_once(self, timeout_sec=0) # Process ROS callbacks (CRITICAL FIX)
                self.print_dashboard() # Restored per user request
                key = self.get_key()
                now = time.time()
                
                if key != '': last_key_time = now
                if now - last_key_time > 0.2:
                    lin_x, ang_z, lin_z = 0.0, 0.0, 0.0
                
                # CONTROLS - FIXED MAPPING & GAIN
                if key == 'w': lin_x = 0.5
                elif key == 's': lin_x = -0.5
                elif key == 'a': ang_z = 0.25   # LEFT (Positive)
                elif key == 'd': ang_z = -0.25  # RIGHT (Negative)
                elif key == 'r': lin_z = 0.5
                elif key == 'f': lin_z = -0.5
                
                # Commands
                elif key == ' ': self.send_cmd({"type": "arm", "cmd": "surface"})
                elif key == 'm': self.send_cmd({"type": "mission", "action": "start"})
                elif key == 'n': self.send_cmd({"type": "mission", "action": "stop"})
                elif key == 'y': self.send_cmd({"type": "mission", "action": "confirm"})
                elif key == 'x': self.send_cmd({"type": "mission", "action": "reject"})
                
                # Arm
                elif key == 'u': self.send_cmd({"type": "arm", "cmd": "grip"})
                elif key == 'i': self.send_cmd({"type": "arm", "cmd": "up"})
                elif key == 'k': self.send_cmd({"type": "arm", "cmd": "down"})
                elif key == 'j': self.send_cmd({"type": "arm", "cmd": "left"})
                elif key == 'l': self.send_cmd({"type": "arm", "cmd": "right"})
                elif key == 'o': self.send_cmd({"type": "arm", "cmd": "rot_left"})
                elif key == 'p': self.send_cmd({"type": "arm", "cmd": "rot_right"})

                if key == '\x03': break
                
                # PACKETIZE MANUAL CONTROL FOR ACOUSTIC CHANNEL
                manual_cmd = {
                    "type": "manual_control",
                    "linear": float(lin_x),
                    "angular": float(ang_z),
                    "vertical": float(lin_z)
                }
                self.send_cmd(manual_cmd)
                
                # Direct publishing DISABLED for acoustic realism
                # t = Twist()
                # t.linear.x, t.angular.z, t.linear.z = float(lin_x), float(ang_z), float(lin_z)
                # self.pub_manual_vel.publish(t)
                
        except Exception as e:
            print(e)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def send_cmd(self, cmd):
        msg = String()
        msg.data = json.dumps(cmd)
        self.pub_topside.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    TeleopNode().run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
