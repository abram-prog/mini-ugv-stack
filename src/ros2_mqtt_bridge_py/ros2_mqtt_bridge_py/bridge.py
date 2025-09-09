# ros2_mqtt_bridge_py/bridge.py
import os
import json
import threading
from importlib import resources

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu

# MQTT
import paho.mqtt.client as mqtt

# REST + статическая панель
from fastapi import FastAPI
from fastapi.responses import HTMLResponse
from starlette.staticfiles import StaticFiles
import uvicorn
import logging

# ---------- конфиг ----------
MQTT_HOST = os.getenv('MQTT_HOST', '127.0.0.1')
MQTT_PORT = int(os.getenv('MQTT_PORT', '1883'))

TOPIC_HEALTH = 'robot/system/health'
TOPIC_IMU    = 'robot/telemetry/imu'

# храним последние значения для REST/UI
LATEST = {
    'health': None,
    'imu': None,
}

# ---------- REST / UI ----------
app = FastAPI(title="ROS2 MQTT Bridge")


# Безопасное монтирование статики из установленного пакета.
# Если каталога нет — просто пропускаем, чтобы сервис не падал.
try:
    static_dir = resources.files("ros2_mqtt_bridge_py") / "static"
    # Открой http://127.0.0.1:8000/  → index.html из static/
    app.mount("/ui", StaticFiles(directory=STATIC_DIR, html=True), name="ui")
except Exception:
    logging.warning("Static UI not found; skipping mount")

@app.get('/health')
def rest_health():
    return {'status': LATEST['health'] or 'unknown'}

@app.get('/telemetry/imu/last')
def rest_last_imu():
    return LATEST['imu'] or {}

@app.get("/ui", response_class=HTMLResponse)
def panel():
    return """
<!doctype html><meta charset="utf-8">
<title>Mini UGV Panel</title>
<h2>Health: <span id="health">...</span></h2>
<pre id="imu">{}</pre>
<script>
async function tick(){
  try{
    const h = await fetch('/health'); 
    document.getElementById('health').textContent = (await h.json()).status;
    const i = await fetch('/telemetry/imu/last'); 
    document.getElementById('imu').textContent = JSON.stringify(await i.json(), null, 2);
  }catch(e){ console.log(e); }
}
setInterval(tick, 1000); tick();
</script>
"""

def run_rest_server():
    # Uvicorn REST сервер: 0.0.0.0:8000
    uvicorn.run(app, host='0.0.0.0', port=8000, log_level='info')


# ---------- ROS2 ↔ MQTT ----------
class BridgeNode(Node):
    def __init__(self):
        super().__init__('ros2_mqtt_bridge')

        # MQTT клиент
        self.mqtt = mqtt.Client()
        self.mqtt.on_connect = self._on_mqtt_connect
        self.mqtt.connect_async(MQTT_HOST, MQTT_PORT, 60)
        self.mqtt.loop_start()

        # Подписки ROS2
        self.sub_health = self.create_subscription(
            String, '/system/health', self.on_health, 10
        )
        self.sub_imu = self.create_subscription(
            Imu, '/sensors/imu', self.on_imu, 10
        )
        self.get_logger().info('Bridge up: ROS2 → MQTT + REST')

    # MQTT callbacks
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('MQTT connected')
        else:
            self.get_logger().warn(f'MQTT connect rc={rc}')

    # ROS2 handlers
    def on_health(self, msg: String):
        LATEST['health'] = msg.data
        self._mqtt_pub(TOPIC_HEALTH, {'status': msg.data})

    def on_imu(self, msg: Imu):
        data = {
            'ts': self.get_clock().now().nanoseconds,
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z,
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z,
            },
            'frame_id': msg.header.frame_id,
        }
        LATEST['imu'] = data
        self._mqtt_pub(TOPIC_IMU, data)

    # publish helper
    def _mqtt_pub(self, topic: str, obj):
        try:
            self.mqtt.publish(topic, json.dumps(obj), qos=0, retain=False)
        except Exception as e:
            self.get_logger().error(f'MQTT publish error on {topic}: {e}')


def main():
    # REST сервер в отдельном потоке
    rest_thread = threading.Thread(target=run_rest_server, daemon=True)
    rest_thread.start()

    # ROS2 нода
    rclpy.init()
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
