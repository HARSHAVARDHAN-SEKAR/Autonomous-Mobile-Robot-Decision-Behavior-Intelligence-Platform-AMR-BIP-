#!/usr/bin/env python3
"""Read-only ROS 2 terminal overlay; do not control motion."""
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Monitor(Node):
    def __init__(self):
        super().__init__('bip_bt_status_monitor')
        self.create_subscription(String, '/executive/status', self.on_status, 10)
        self.last_signature=None
        self.last_print=0.0
        print('AMR-BIP | BT state monitor | waiting for /executive/status',flush=True)

    def on_status(self,msg):
        try: d=json.loads(msg.data)
        except Exception: return
        sig=(d.get('state'),d.get('mission'),d.get('step'),d.get('charging'),tuple(d.get('completed',[])),tuple(x.get('name') for x in d.get('failed',[])))
        now=time.monotonic()
        if sig!=self.last_signature or now-self.last_print>=3:
            print(f"STATE {str(d.get('state')):19} | battery {float(d.get('battery',0)):5.1f}% | charge={str(d.get('charging')):5} | mission {str(d.get('mission')):19} step {d.get('step')} | done={len(d.get('completed',[]))} failed={len(d.get('failed',[]))}",flush=True)
            if d.get('state')=='DONE':
                print('FINAL:',msg.data,flush=True)
            self.last_print=now;self.last_signature=sig

def main():
    rclpy.init();node=Monitor()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
