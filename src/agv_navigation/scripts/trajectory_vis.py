import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class TrajectoryDisplay:
    def __init__(self):
        rospy.init_node('trajectory')
        self.frame_id_ = 'map'
        self.odom_topic = '/amcl_pose'
        self.traj_topic = '~trajectory'
        
        self.odom_sub = rospy.Subscriber(self.odom_topic, PoseWithCovarianceStamped, self.callback)
        self.traj_pub = rospy.Publisher(self.traj_topic, MarkerArray, queue_size=10)
        
        self.last_timestamp = None
        
        self.points = None
        self.linestrip = None
        self.trajectory = None
        
        self.initialized = False
        
        self.skip_cnt = 0
        self.linestrip_disp_cnt = 2
        self.points_disp_cnt =  self.linestrip_disp_cnt * 100
        
        self.rate = rospy.Rate(20)
    
    def create_markers(self):
        self.points = Marker()
        self.linestrip = Marker()
        
        self.points.header.frame_id = self.frame_id_
        self.linestrip.header.frame_id = self.frame_id_
        
        self.points.ns = 'waypoints'
        self.linestrip.ns = 'partial_trajectory'
        
        self.points.id = 1
        self.points.id = 2
        
        self.points.type =  Marker.POINTS
        self.linestrip.type = Marker.LINE_STRIP
        
        self.points.action = Marker.ADD
        self.linestrip.action = Marker.ADD
        
        self.points.color.r = 0.0
        self.points.color.g = 1.0
        self.points.color.b = 0.0
        self.points.color.a = 0.8
        
        self.linestrip.color.r = 0.0
        self.linestrip.color.g = 0.0
        self.linestrip.color.b = 0.8
        self.linestrip.color.a = 0.6
        
        self.points.scale.x = 0.15
        self.points.scale.y = 0.15
        self.points.scale.z = 0.0
        
        self.points.pose.orientation.w = 1.0
        self.linestrip.pose.orientation.w = 1.0
        
        self.linestrip.scale.x = 0.1
        
    def callback(self, msg):
        if not self.initialized:
            self.initialized = True
            
        if self.initialized:
            # print('chk')
            if self.skip_cnt % self.linestrip_disp_cnt == 0:
                self.last_timestamp = msg.header.stamp
                tmp_x = msg.pose.pose.position.x
                tmp_y = msg.pose.pose.position.y
                tmp_z = 0.0
                # print('{}, {}, {}'.format(tmp_x, tmp_y, tmp_z))
                
                self.linestrip.header.stamp = self.last_timestamp
                self.linestrip.points.append(Point(tmp_x, tmp_y, tmp_z))
                
                if self.skip_cnt % self.points_disp_cnt == 0:
                    self.points.header.stamp = self.last_timestamp
                    self.points.points.append(Point(tmp_x, tmp_y, tmp_z))
                    
                self.trajectory = MarkerArray()
                self.trajectory.markers.append(self.points)
                self.trajectory.markers.append(self.linestrip)
                
                self.traj_pub.publish(self.trajectory)
            if(self.skip_cnt == self.points_disp_cnt):
                self.skip_cnt = 1
            else:
                self.skip_cnt += 1
        
    def run(self):
        try:
            while not rospy.is_shutdown():
                self.rate.sleep()
            
        except Exception as e:
            print(e)
            exit()
            
            
if __name__ == '__main__':
    node_ = TrajectoryDisplay()
    node_.create_markers()
    node_.run()