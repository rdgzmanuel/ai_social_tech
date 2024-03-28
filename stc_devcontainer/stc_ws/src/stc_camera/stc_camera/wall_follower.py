from enum import Enum
from rclpy.node import Node
import numpy as np
from collections import Counter
 
class WallFollower(Node):
    """Class to safely explore an environment (without crashing) when the pose is unknown."""
 
    def __init__(self, dt: float) -> None:
        """Wall following class initializer.
 
        Args:
            dt: Sampling period [s].
 
        """
        super().__init__("wall_follower")
        self._dt: float = dt
        self.state = Estates.CONTINUE
        self.explored = 0
        self.previous_states: list = [Estates.CONTINUE]
 
    def compute_commands(self, z_scan: list[float], z_v: float, z_w: float) -> tuple[float, float]:
        """Wall following exploration algorithm.
 
        Args:
            z_scan: Distance from every LiDAR ray to the closest obstacle [m].
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].
 
        Returns:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].
 
        """
        # TODO: 1.14. Complete the function body with your code (i.e., compute v and w).
        n_lasers: int = len(z_scan)
       
        left_side = z_scan[(n_lasers // 8) - 2: (3 * n_lasers // 8) + 2]
        bottom_side = z_scan[(3 * n_lasers // 8) - 2: (5 * n_lasers // 8) + 2]
        right_side = z_scan[(5 * n_lasers // 8) - 2: (7 * n_lasers // 8) + 2]
        top_side = z_scan[:n_lasers // 8] + z_scan[7 * n_lasers // 8:]
 
        distance_top = np.nanmean(top_side)
        distance_bottom = np.nanmean(bottom_side)
        distance_left = np.nanmean(left_side)
        distance_right = np.nanmean(right_side)
 
        margin = 0.05
        v = 0.18
        w = 0.0
        close_distance = 0.3
        far_distance = 0.6
        side_close_distance = 0.2

        _counter = Counter(self.previous_states)
        previous_state = _counter.most_common(1)[0][0]

        self.get_logger().warn(f"_________STATE: {self.state}")
        self.get_logger().warn(f"PREVIOUS_STATE: {self.state}")
 
        # Avoid top colision
        if distance_top < close_distance:
            self.state = Estates.STOP
 
        # Control change states
        if self.state == Estates.CONTINUE:
            distance_top_min = min(top_side)
            # self.get_logger().warn(f"TOP {distance_top}")
 
            # Avoid side collision
            if (distance_right < side_close_distance + margin or
                distance_left < side_close_distance + margin or
                distance_top_min < close_distance):
                self.state = Estates.STOP
           
            elif distance_left > 2 * far_distance and previous_state == Estates.CONTINUE:
                self.state = Estates.TURN_LEFT
   
 
        elif self.state == Estates.ROTATE_RIGHT:
            if distance_right < side_close_distance + margin or np.nanmean(z_scan[205:215]) < side_close_distance or np.nanmean(z_scan[195:205]) < side_close_distance:
                self.state = Estates.ROTATE_LEFT
            elif distance_left > 0.3 - margin and distance_left < 0.3 + margin:
                self.state = Estates.CONTINUE
 
 
        elif self.state == Estates.ROTATE_LEFT:
            if distance_left < side_close_distance + margin or np.nanmean(z_scan[25:35]) < side_close_distance or np.nanmean(z_scan[15:25]) < side_close_distance:
                self.state = Estates.ROTATE_RIGHT
            elif distance_right > 0.3 - margin and distance_right < 0.3 + margin:
                self.state = Estates.CONTINUE
 
 
        elif self.state == Estates.STOP:
            top_side = z_scan[:n_lasers // 32] + z_scan[31 * n_lasers // 32:]
            
            if distance_left > far_distance:
                self.state = Estates.TURN_LEFT
            elif distance_right > far_distance:
                self.state = Estates.TURN_RIGHT
            elif distance_right < close_distance + margin and distance_left < close_distance + margin and distance_top < close_distance + margin:
                self.state = Estates.ROTATE_180
            elif distance_right > distance_left:
                self.state = Estates.ROTATE_RIGHT
            elif distance_right < distance_left:
                self.state = Estates.ROTATE_LEFT
            
            elif min(top_side) > 0.3 + margin:
                self.state = Estates.CONTINUE
 

        elif self.state == Estates.TURN_LEFT:
            top_side = z_scan[:n_lasers // 32] + z_scan[31 * n_lasers // 32:]
            distance_top_max = max(top_side)
            distance_top_min = min(top_side)
            if (distance_top_min > 0.3
                and (min(right_side) in z_scan[178: 182] or distance_top_max in z_scan[0:2] + z_scan[-2:])
                and previous_state == Estates.TURN_LEFT
            ):
                self.state = Estates.CONTINUE
 
        elif self.state == Estates.TURN_RIGHT:
            top_side = z_scan[:n_lasers // 32] + z_scan[31 * n_lasers // 32:]
            distance_top_max = max(top_side)
            distance_top_min = min(top_side)
           
            if (distance_top_min > 0.3
                and (min(left_side) in z_scan[58:62] or distance_top_max in z_scan[0:2] + z_scan[-2:])
            ):
                self.state = Estates.CONTINUE
 
        # elif self.state == Estates.TURN_LEFT_EXPLORE or self.state == Estates.TURN_RIGHT_EXPLORE:
        #     top_side = z_scan[:n_lasers // 32] + z_scan[31 * n_lasers // 32:]
        #     bottom_side = z_scan[110: 130]
        #     distance_bottom = np.nanmean(bottom_side)
        #     distance_top_max = max(top_side)
        #     distance_top_min = min(top_side)
        #     ## Second condition to avoid exploring in diagonal
        #     if (distance_top_min > 0.3
        #     and distance_bottom < far_distance):
        #         self.state = Estates.CONTINUE
 
       
        elif self.state == Estates.ROTATE_180:
            if (
                distance_top > far_distance
                and max(top_side) == z_scan[0]
            ):
                self.state = Estates.STOP
 
        # elif self.state == Estates.DO_NOT_EXPLORE:
        #     distance_top_min = min(top_side)
       
        #     self.get_logger().warn(f"TOP NOT EXPLORING {distance_top}")
        #     # Avoid top colision
        #     if distance_top < close_distance:
        #         self.state = Estates.STOP
        #         self.explored = 1
        #     # Avoid side collision
        #     elif np.nanmean(z_scan[25:35]) < side_close_distance + margin:
        #         self.state = Estates.STOP
        #         self.get_logger().warn(f"WHEEL 1 {distance_left}")
        #         self.explored = 1
        #     elif np.nanmean(z_scan[205:215]) < side_close_distance + margin:
        #         self.state = Estates.STOP
        #         self.get_logger().warn(f"WHEEL 2 {distance_right}")
        #         self.explored = 1
        #     elif distance_right < side_close_distance + margin:
        #         self.state = Estates.ROTATE_LEFT
        #         self.get_logger().warn(f"LEFT {distance_right}")
        #         self.explored = 1
        #     elif distance_left < side_close_distance + margin:
        #         self.state = Estates.ROTATE_RIGHT
        #         self.get_logger().warn(f"RIGHT {distance_left}")
        #         self.explored = 1
 
        # Modify speeds depending on the state
        if self.state == Estates.STOP:
            v = 0.0
            w = 0.0
       
        # CONTINUE HAS DEFAULT SPEEDS
 
        elif self.state == Estates.ROTATE_LEFT:
            w += 0.2
            v = 0.05
 
        elif self.state == Estates.ROTATE_RIGHT:
            w -= 0.2
            v = 0.05
 
        elif self.state == Estates.TURN_LEFT or self.state == Estates.TURN_LEFT_EXPLORE:
            w = 0.35
            v = 0.0
 
        elif self.state == Estates.TURN_RIGHT or self.state == Estates.TURN_RIGHT_EXPLORE:
            w = -0.35
            v = 0.0
       
        elif self.state == Estates.ROTATE_180:
            v = 0.0
            w = 0.4
        
        # Add current state to previous states
        if len(self.previous_states) > 15:
            self.previous_states.pop(0)
        self.previous_states.append(self.state)


 
        return v, w
 
 
class Estates(Enum):
    TURN_RIGHT = 1
    TURN_LEFT = 2
    CONTINUE = 3
    ROTATE_RIGHT = 4
    ROTATE_LEFT = 5
    STOP = 6
    ROTATE_180 = 7
    TURN_LEFT_EXPLORE = 8
    TURN_RIGHT_EXPLORE = 9
    DO_NOT_EXPLORE = 10