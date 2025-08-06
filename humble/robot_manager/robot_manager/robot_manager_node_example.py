import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
import json

# 관리자 인터페이스
from manager_interfaces.action import ExecuteTask

# 실무자 인터페이스
from robot_interfaces.action import MovePoint, GoHome, Dock, Patrol
from robot_interfaces.srv import StopMotion, EmergencyStop
# from mapping_interfaces.action import StartMapping # 맵핑 기능 추가 시

class RobotManagerNode(Node):
    def __init__(self):
        super().__init__('robot_manager_node')
        self.get_logger().info("Robot Manager is starting...")
        
        # 로봇의 현재 상태 관리
        self.robot_state = 'IDLE'

        # 1. 최상위 명령을 받는 Action 서버 생성
        self._action_server = ActionServer(
            self, ExecuteTask, 'execute_task', self.execute_task_callback)

        # 2. 모든 실무자 노드와 통신할 Action/Service 클라이언트 생성
        self.move_point_client = ActionClient(self, MovePoint, 'move_point')
        self.go_home_client = ActionClient(self, GoHome, 'go_home')
        self.patrol_client = ActionClient(self, Patrol, 'patrol')
        self.stop_motion_client = self.create_client(StopMotion, 'stop_motion')
        # ... 다른 모든 클라이언트들 ...

    def execute_task_callback(self, goal_handle):
        task_name = goal_handle.request.task_name
        self.get_logger().info(f"Received high-level task: '{task_name}'")

        # 작업 중일 경우 새 명령 거부 (또는 현재 작업 취소 로직 추가)
        if self.robot_state != 'IDLE':
            goal_handle.abort()
            return ExecuteTask.Result(success=False, message="Robot is busy.")
        
        # 명령에 따라 적절한 실무자에게 작업 위임
        if task_name == "go_home":
            self.robot_state = 'HOMING'
            # go_home 액션 클라이언트를 호출하는 로직
            # ...
        
        elif task_name == "patrol":
            self.robot_state = 'PATROLLING'
            # patrol 액션 클라이언트를 호출하는 로직
            # ...

        elif task_name == "move_point":
            self.robot_state = 'MOVING'
            try:
                params = json.loads(goal_handle.request.task_params)
                # move_point 액션 클라이언트에 params를 사용해 goal을 보내는 로직
                # ...
            except json.JSONDecodeError:
                goal_handle.abort()
                return ExecuteTask.Result(success=False, message="Invalid JSON params.")

        elif task_name == "stop":
            # stop 서비스 클라이언트를 호출하는 로직
            # ...
            self.robot_state = 'IDLE'
        
        # ... 다른 모든 task에 대한 처리 ...
        
        else:
            goal_handle.abort()
            return ExecuteTask.Result(success=False, message=f"Unknown task: {task_name}")

        # 작업이 끝나면 상태를 다시 IDLE로 변경하는 로직 필요
        # (Action의 done_callback 활용)
        
        goal_handle.succeed()
        result = ExecuteTask.Result()
        result.success = True
        return result
# (main 함수는 일반적인 노드와 동일)