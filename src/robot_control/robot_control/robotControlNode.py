import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry # Added for live tracking
from fastapi import FastAPI, HTTPException, Depends, Header
from fastapi.staticfiles import StaticFiles
from geometry_msgs.msg import PoseWithCovarianceStamped
from pydantic import BaseModel
import uvicorn
import threading
import os
from fastapi.middleware.cors import CORSMiddleware

# --- ROS 2 NODE LOGIC ---
class RobotControlNode(Node):
    def __init__(self):
        super().__init__('web_teleop_gateway')
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Subscriber for live tracking (Odometry)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_pos = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.get_logger().info("ROS 2 Gateway Node Active with Tracking.")

    def odom_callback(self, msg):
        self.current_pos["x"] = round(msg.pose.pose.position.x, 2)
        self.current_pos["y"] = round(msg.pose.pose.position.y, 2)

    def send_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def send_initial_pose(self, x, y):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.w = 1.0 # Facing forward
        self.init_pose_pub.publish(msg)

    def send_goal(self, x, y):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)


# --- FASTAPI SETUP ---
app = FastAPI()
VALID_AUTH_KEY = "mybotkeyisnothing"

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize ROS 2
rclpy.init()
ros_node = RobotControlNode()
threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True).start()

class MoveRequest(BaseModel):
    linear: float
    angular: float

class NavRequest(BaseModel):
    x: float
    y: float

async def verify_key(x_auth_key: str = Header(None)):
    if x_auth_key != VALID_AUTH_KEY:
        raise HTTPException(status_code=403, detail="Unauthorized")
    return x_auth_key

@app.get("/robot/status")
async def get_status(key: str = Depends(verify_key)):
    return {
        "status": "online",
        "position": ros_node.current_pos # Returns X and Y to the dashboard
    }

@app.post("/robot/move")
async def move_robot(req: MoveRequest, key: str = Depends(verify_key)):
    ros_node.send_velocity(req.linear, req.angular)
    return {"status": "ok"}

@app.post("/robot/navigate")
async def navigate_to(req: NavRequest, key: str = Depends(verify_key)):
    ros_node.send_goal(req.x, req.y)
    return {"status": "goal_sent"}
# Add the FastAPI Endpoint
@app.post("/robot/init_pose")
async def init_pose(req: NavRequest, key: str = Depends(verify_key)):
    ros_node.send_initial_pose(req.x, req.y)
    return {"status": "initial_pose_sent"}
frontend_path = os.path.expanduser("~/web_robot_ws/src/web")
app.mount("/", StaticFiles(directory=frontend_path, html=True), name="frontend")

def main(args=None):
    try:
        uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            ros_node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()