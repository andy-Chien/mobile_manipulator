import rclpy
import quaternion as qtn
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future
from nav2_msgs.action import FollowPath, ComputePathToPose
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from mm_msgs.srv import PathToTraj, TrajMerge

USING_MM_TRAJECTORY_CONTROLLER = True

class PathAdapter(Node):

    def __init__(self, name):
        super().__init__(name)
        self._traj_compute_client = ActionClient(
            self, MoveGroup, '/move_action'
        )
        self._path_compute_client = ActionClient(
            self, ComputePathToPose, '/compute_path_to_pose'
        )
        self._execute_path_client = ActionClient(
            self, FollowPath, 'mobile_path_controller/follow_path'
        )
        self._execute_traj_client = ActionClient(
            self, FollowJointTrajectory, 'mm_trajectory_controller/follow_joint_trajectory'
        )
        self.path_to_traj_client_ = self.create_client(
            PathToTraj, 'path_to_trajectory'
        )
        self.merge_traj_client_ = self.create_client(
            TrajMerge, 'trajectory_merging'
        )
        self.point_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.clicked_point_callback,
            1)
        self.plan_req_subscription = self.create_subscription(
            MotionPlanRequest,
            '/motion_plan_request',
            self.plan_req_callback,
            1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ignore_selves_request = False
        
        self.point_subscription  # prevent unused variable warning
        self.plan_req_subscription # prevent unused variable warning
        self._get_result_future: Future
        self._planned_traj = None
        self.timer = self.create_timer(10, self.timer_callback) # to reset _planned_traj
        self.get_logger().info('Hi, "{}" node is on :)'.format(name))

    def send_traj_compute_goal(self, plan_req):
        goal_msg = MoveGroup.Goal()
        goal_msg.request = plan_req
        goal_msg.planning_options.plan_only = True
        if not self._traj_compute_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('MoveGroup action server not responding')
            return
        self.ignore_selves_request = True
        self._send_goal_future = self._traj_compute_client.send_goal_async(
            goal_msg, feedback_callback=self.traj_compute_feedback_cb)
        self._send_goal_future.add_done_callback(self.traj_compute_goal_res_cb)

    def send_path_compute_goal(self, goal_pose):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal_pose
        goal_msg.use_start = False
        goal_msg.planner_id = "GridBased"
        if not self._path_compute_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('ComputePathToPose action server not responding')
            return
        self._send_goal_future = self._path_compute_client.send_goal_async(
            goal_msg, feedback_callback=self.path_compute_feedback_cb)
        self._send_goal_future.add_done_callback(self.path_compute_goal_res_cb)

    def send_path_execute_goal(self, goal_path):
        goal_msg = FollowPath.Goal()
        goal_msg.path = goal_path
        goal_msg.controller_id="FollowPath"
        goal_msg.goal_checker_id="general_goal_checker"
        if not self._execute_path_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('FollowPath action server not responding')
            return
        self._send_goal_future = self._execute_path_client.send_goal_async(
            goal_msg, feedback_callback=self.path_execute_feedback_cb)
        self._send_goal_future.add_done_callback(self.path_execute_goal_res_cb)

    def send_traj_execute_goal(self, goal_traj):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = goal_traj
        goal_msg.trajectory.header.stamp.sec = 0
        goal_msg.trajectory.header.stamp.nanosec = 0
        if not self._execute_traj_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('FollowJointTrajectory action server not responding')
            return
        self._send_traj_goal_future = self._execute_traj_client.send_goal_async(
            goal_msg, feedback_callback=self.traj_execute_feedback_cb)
        self._send_traj_goal_future.add_done_callback(self.traj_execute_goal_res_cb)

    def send_path_traj_req(self, goal_path):
        req = PathToTraj.Request()
        req.path = goal_path
        if not self.path_to_traj_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('PathToTraj server not responding')
            return
        self._send_req_future = self.path_to_traj_client_.call_async(req)
        self._send_req_future.add_done_callback(self.path_traj_res_cb)

    def send_traj_merge_req(self, trajectories):
        req = TrajMerge.Request()
        req.trajectories = trajectories
        if not self.merge_traj_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('TrajMerge server not responding')
            return
        self._send_req_future = self.merge_traj_client_.call_async(req)
        self._send_req_future.add_done_callback(self.traj_merge_res_cb)

    def traj_compute_goal_res_cb(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('MoveGroup goal rejected :(')
            self.ignore_selves_request = False
            return

        self.get_logger().info('MoveGroup goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.traj_compute_result_cb)

    def path_compute_goal_res_cb(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('ComputePathToPose goal rejected :(')
            return

        self.get_logger().info('ComputePathToPose goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.path_compute_result_cb)

    def path_execute_goal_res_cb(self, future: Future):
        goal_handle: ClientGoalHandle
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('FollowPath goal rejected :(')
            return

        self.get_logger().info('FollowPath goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.path_execute_result_cb)

    def traj_execute_goal_res_cb(self, future: Future):
        goal_handle: ClientGoalHandle
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Trajectory goal rejected :(')
            return

        self.get_logger().info('Trajectory goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.traj_execute_result_cb)

    def path_traj_res_cb(self, future: Future):
        if self._planned_traj is None:
            self.send_traj_execute_goal(future.result().trajectory)
        else:
            self.send_traj_merge_req([future.result().trajectory, self._planned_traj])
        self._planned_traj = None

    def traj_merge_res_cb(self, future: Future):
        if future.result().success:
            self.send_traj_execute_goal(future.result().trajectory)

    def traj_compute_result_cb(self, future: Future):
        result: MoveGroup.Result
        result = future.result().result
        if result.error_code._val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('MoveIt planning successed: {}'.format(result.planning_time))
        else:
            self.get_logger().warn('MoveIt plan failed, {}'.format(result.error_code))
        self._planned_traj = result.planned_trajectory.joint_trajectory
        self.timer.reset()
        self.ignore_selves_request = False

    def path_compute_result_cb(self, future: Future):
        result: ComputePathToPose.Result
        result = future.result().result
        # TODO(andy): the future update will has error code can check result
        self.get_logger().info('Result: {0}'.format(result.planning_time))
        start_pose = PoseStamped()
        try:
            t = self.tf_buffer.lookup_transform(
                "map",
                "mobile_base_footprint",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform map to mobile_base_footprint: {ex}')
            return
        start_pose.header = result.path.poses[0].header
        start_pose.pose.position.x = t.transform.translation.x
        start_pose.pose.position.y = t.transform.translation.y
        start_pose.pose.position.z = t.transform.translation.z
        start_pose.pose.orientation = t.transform.rotation
        result.path.poses.insert(0, start_pose)

        p_0 = result.path.poses[0].pose
        p_end = result.path.poses[-1].pose
        q_0 = qtn.quaternion(
            p_0.orientation.w, p_0.orientation.x, p_0.orientation.y, p_0.orientation.z)
        q_end = qtn.quaternion(
            p_end.orientation.w, p_end.orientation.x, p_end.orientation.y, p_end.orientation.z)
        pose_size = len(result.path.poses)
        for i, pose in enumerate(result.path.poses):
            q = qtn.slerp(q_0, q_end, 0.0, 1.0, float(i) / float(pose_size - 1))
            pose.pose.orientation.w = q.w
            pose.pose.orientation.x = q.x
            pose.pose.orientation.y = q.y
            pose.pose.orientation.z = q.z
        if USING_MM_TRAJECTORY_CONTROLLER:
            self.send_path_traj_req(result.path)
        else:
            self.send_path_execute_goal(result.path)

    def path_execute_result_cb(self, future: Future):
        result: FollowPath.Result
        result = future.result().result
        # TODO(andy): the future update will has error code can check result
        self.get_logger().info('Result: Nice bro, path execution successed')
    
    def traj_execute_result_cb(self, future: Future):
        result: FollowJointTrajectory.Result
        result = future.result().result
        if result.error_code is FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Result: Nice bro, traj execution successed')
        else:
            self.get_logger().warn('Result: So sad bro, {}'.format(result.error_string))

    def traj_compute_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            'Received feedback: {0}'.format(feedback.state))

    def path_compute_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            'Received feedback: {0}'.format(feedback.partial_sequence))

    def path_execute_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            'Received feedback: distance = {0}, speed = {1}'.format(
                feedback.distance_to_goal, feedback.speed))

    def traj_execute_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            'Received feedback: actual = {0}, error = {1}'.format(
                feedback.actual, feedback.error))

    def clicked_point_callback(self, msg: PoseStamped):
        self.send_path_compute_goal(msg)

    def plan_req_callback(self, msg: MotionPlanRequest):
        if self.ignore_selves_request is not True:
            self.send_traj_compute_goal(msg)

    def timer_callback(self):
        self._planned_traj = None
        self.timer.cancel()