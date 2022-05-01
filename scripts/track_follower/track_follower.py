#!/usr/bin/env python

import rospy
import numpy as np

from final_challenge.msg import LaneLocation, LaneError
from ackermann_msgs.msg import AckermannDriveStamped

class TrackFollower():
    """
    A controller for following the indoor Johnson track.
    Listens for lane locations using both left and right
    cameras.
    Not able to used in sim right now, but will add that
    later if needed for debugging.
    """
    def __init__(self):
        rospy.Subscriber("/relative_left_lane", LaneLocation, self.left_lane_callback)
        rospy.Subscriber("/relative_right_lane", LaneLocation, self.right_lane_callback)

        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/lane_error", LaneError, queue_size=10)

        self.relative_x = 0
        self.relative_y = 0
        self.lookahead = 2 #2.0 #2.5
        self.wheelbase_length = 0.325
        self.right_lane = None
        self.left_lane = None
        self.middle_lane = None
        self.speed = 4
        self.angle = 0

    def lane_callback(self, lane_location_msg):
        """
        Takes in a lane location msg and returns the slope and y intecept of lane
        in x and y coordinates
        """
        
        x_values = lane_location_msg.x_pos
        y_values = lane_location_msg.y_pos
        if len(x_values) < 50:
            return None

        p = np.polyfit(x_values, y_values, 1)
        
        if abs(p[0]) > 0.5 or abs(p[1]) > 0.8:
            return None
        start_x = 0; end_x = 6; num_segments = 100

        new_x = np.linspace(start_x, end_x, num_segments+1)
        new_y = p[0]*new_x + p[1]
        return (new_x, new_y, p[0]) # Added slope because it is useful for edge cases

    def right_lane_callback(self, lane_location_msg):
        self.right_lane = self.lane_callback(lane_location_msg)
        self.calculate_middle_lane()

    def left_lane_callback(self, lane_location_msg):
        self.left_lane = self.lane_callback(lane_location_msg)
        self.calculate_middle_lane()
        self.pursuit()
    
    def calculate_middle_lane(self):
    	pts_list = []
        if (self.left_lane is None) and (self.right_lane is not None):
            # Calculate a spot that is 1 meter away that is also 18*0.0254=0.4572 m
            # from the right lane
            new_x = self.right_lane[0]
            new_y = self.right_lane[1]
            slope = self.right_lane[2]
            for i in range(len(new_x)):
                pt = (new_x[i], new_y[i])
                pts_list.append(pt)
            
            perp_dist = 0.4572
            v = np.array([1, slope*1, 0])
            v = v
            w = np.array([0, 0, 1])
            cross_vec = np.cross(w, v)
            unit_perp = cross_vec[:2]
            self.middle_lane = np.array(pts_list) + unit_perp*perp_dist
            

        elif (self.left_lane is not None) and (self.right_lane is None):
            # Calculate a spot that is 1 meter away that is also 18*0.0254=0.4572 m
            # from the left lane
            new_x = self.left_lane[0]
            new_y = self.left_lane[1]
            slope = self.left_lane[2]
            for i in range(len(new_x)):
                pt = (new_x[i], new_y[i])
                pts_list.append(pt)

            perp_dist = 0.4572
            v = np.array([1, slope*1, 0])
            v = v
            w = np.array([0, 0, 1])
            cross_vec = np.cross(v, w)
            unit_perp = cross_vec[:2]
            self.middle_lane = np.array(pts_list) + unit_perp*perp_dist

            
        elif (self.left_lane is not None) and (self.right_lane is not None): # Default case is that both lanes are not none
            new_y_l = self.left_lane[1]
            new_y_r = self.right_lane[1]
            new_y = (new_y_l + new_y_r)/2

            new_x = self.left_lane[0]
            for i in range(len(new_y)):
                pt = (new_x[i], new_y[i])
                pts_list.append(pt)

            self.middle_lane = np.array(pts_list)
        #rospy.loginfo("Calculating middle_lane")
        #rospy.loginfo(self.middle_lane) 

    def find_closest_point(self):
        """ Finds the closest point on the followed trajectory based on localization information.
            Inputs
            --------
            odom: Odometry msg, describes vehicle's position in the world

            Outputs
            --------
            closest: Tuple of the closest point, closest distance, and the index value of the trajectory
                    segment (starting from 0)
        """
        if self.middle_lane is None:
            return None
        # Current position based on odometry
        cur_pos = np.array([0, 0])

        # Trajectory points
        traj_pts = self.middle_lane

        # Relative Trajectory Segment Vectors
        traj_deltas = np.diff(traj_pts, axis=0)
        traj_norm_squared = np.sum(np.multiply(traj_deltas, traj_deltas), axis=1)

        # Relative Vectors from Current Pose to Trajectory Points
        traj2pos_deltas = np.subtract(cur_pos, traj_pts[:-1])

        # Finding closest point for each segment
        #rospy.loginfo("Traj 2 Pos: [%.2f, %.2f]", traj2pos_deltas[0][0], traj2pos_deltas[0][1])
        #rospy.loginfo("Traj Delta: [%.2f, %.2f]", traj_deltas[10][0], traj_deltas[10][1])

        dot_product = np.sum(np.multiply(traj2pos_deltas, traj_deltas), axis=1)
        #rospy.loginfo("Dot Product: %.2f", dot_product[0])

            # Scale it to the norms of the trajectory segments
        scaled_dot_product = np.divide(dot_product, traj_norm_squared)
        #rospy.loginfo("Traj Norm: %.2f", traj_norms[10])
            # Cap the result to either a 0 or a 1, to stay within segment boundaries
        capped = np.maximum(np.minimum(scaled_dot_product, 1), 0)
            # Multiply this value to the segment deltas and add it to the traj_pts to get closest points
            # on each segment
        closest_pts_list = np.add(traj_pts[:-1], np.multiply(traj_deltas, capped.reshape(len(capped), 1)))
        difference = np.subtract(cur_pos, closest_pts_list)
        closest_dist_list = np.sqrt(np.sum(np.multiply(difference,difference), axis=1))
        # index of the actual closesst point
        if len(closest_dist_list) > 0:
            c_ind = closest_dist_list.argmin()
        else:
            #rospy.loginfo("Something went wrong again, the list is empty")
            #rospy.loginfo(len(closest_pts_list))
            #rospy.loginfo(len(traj_deltas))
            return None

        #rospy.loginfo("Closest Point: (%f.2, %f.2)", closest_pts_list[c_ind][0], closest_pts_list[c_ind][1])
        #rospy.loginfo("Closest Distance: %f.2", closest_dist_list[c_ind])
        #rospy.loginfo("Current Position: (%.2f, %.2f)", cur_pos[0], cur_pos[1])

        # Return a tuple of the closest point, and the index value
        closest = (closest_pts_list[c_ind], c_ind, closest_dist_list[c_ind])
        return closest
        
    def find_goal_point(self):
        """ Find the goal point on the current trajectory
        """
        cur_pos = np.array([0, 0])
        closest = self.find_closest_point()
        if closest is None:
            return None

        traj_pts = self.middle_lane

        closest_pt = closest[0]
        closest_ind = closest[1]
        closest_dist = closest[2]

        if closest_dist > self.lookahead:
            return (closest_pt, False)
        else:
            # Find the first point that is farther than the lookahead distance
            first_out = closest_ind
            for i in range(closest_ind+1, len(traj_pts)):
                pt = traj_pts[i]
                squared_dist = np.dot(cur_pos-pt, cur_pos-pt)
                #rospy.loginfo("Point: [%.2f, %.2f]", pt[0], pt[1])
                #rospy.loginfo("Squared Dist: %.2f", squared_dist)
                if (squared_dist > self.lookahead**2):
                    first_out = i
                    break
            out_pt = traj_pts[first_out][:]
            in_pt = traj_pts[first_out-1][:]
            
            vec_v = out_pt - in_pt
            vec_w = in_pt - cur_pos
            
            c = np.dot(vec_w, vec_w) - self.lookahead**2
            b = 2*np.dot(vec_v, vec_w)
            a = np.dot(vec_v, vec_v)
            
            t = 0
            if (b**2 - 4*a*c) > 0:
                t1 = (-b + np.sqrt(b**2 - 4*a*c))/(2*a)
                t2 = (-b - np.sqrt(b**2 - 4*a*c))/(2*a)
                
                t = max(t1, t2)
                    
                #rospy.loginfo("This is the expected case")
                #rospy.loginfo("t1 = %.2f, t2 = %.2f", t1, t2)

                return (in_pt + t*vec_v, True)

            elif abs(b**2 - 4*a*c) < 1e-6 and b < 0:
                t = -b/(2*a)
                #rospy.loginfo("This is not expected, but should still work")
                #rospy.loginfo("t = %.2f", t)
                return (in_pt + t*vec_v, True)

            else:
                #rospy.loginfo("wtf, negative b^2-4*a*c")
                return None

    def pursuit(self):
        """ Publishes drive instructions based on current PF pose information
        """

        # Find the goal point
        goal = self.find_goal_point()
        #rospy.loginfo("Running Pure Pursuit")
        #rospy.loginfo(goal)
        """
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.header.frame_id = msg.header.frame_id
        ack_msg.drive.steering_angle = 0
        ack_msg.drive.speed = 0
        self.drive_pub.publish(ack_msg)
        """
        if goal is not None:
            goal_point = goal[0]
            goal_point = np.append(goal_point, [0])
            
            # Find current position and orientation
            cur_pos = np.array([0, 0, 0])
            
            orientation_vec = np.array([1, 0, 0])
            diff_vec = np.subtract(goal_point, cur_pos)
            cross = np.cross(orientation_vec, diff_vec)
            
            #rospy.loginfo("Orientation: %.2f, %.2f", orientation_vec[0], orientation_vec[1])
            #rospy.loginfo("angle: %.2f", cur_theta)
            #rospy.loginfo("Goal Point: %.2f, %.2f", goal[0][0], goal[0][1])
            #rospy.loginfo("Current Position: %.2f, %.2f", cur_pos[0], cur_pos[1])
            #rospy.loginfo("Diff_vec: [%.2f, %.2f]", diff_vec[0], diff_vec[1])
            perp_dist = cross[2] # Allowed to be negative to distinguish between turing left or right
            #rospy.loginfo("Cross: [%.2f, %.2f, %.2f]", cross[0], cross[1], cross[2])
            #rospy.loginfo("Perpendicular Distance: %.2f", perp_dist)
       
            
            if goal[1]:
                arc_curvature = (2*perp_dist)/(self.lookahead**2)
                #rospy.loginfo("Curvature: %.2f", arc_curvature)
                steering_angle = np.arctan(self.wheelbase_length*arc_curvature)-0.02 # Getting steering angle from curvature
            else:
                if abs(perp_dist) > 1e-3:
                    #rospy.loginfo("Edge Case")
                    steering_angle = np.sign(perp_dist)*1
                else:
                    steering_angle = 0
            #rospy.loginfo("Steering: %.2f", steering_angle)
            #rospy.loginfo("----------------------------")
            
            ack_msg = AckermannDriveStamped()
            ack_msg.header.stamp = rospy.Time.now()
            ack_msg.header.frame_id = "/map"
            ack_msg.drive.steering_angle = steering_angle
            ack_msg.drive.speed = self.speed
            err = LaneError()
            err.y_error = -self.middle_lane[0][0]
            err.theta_error = -np.arctan((self.middle_lane[-1][1]-self.middle_lane[0][1])/(self.middle_lane[-1][0] - self.middle_lane[0][0]))
            self.drive_pub.publish(ack_msg)
            self.error_pub.publish(err)
            #rospy.loginfo("Is this working")
        else:
            # rospy.loginfo("Not working, empty trajectory")
            pass

if __name__=="__main__":
    rospy.init_node("track_follower")
    track_follower = TrackFollower()
    rospy.spin()
