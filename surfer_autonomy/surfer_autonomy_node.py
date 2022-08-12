import math
from functools import partial
import pluginlib
import surfer_autonomy.autonomy
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseArray,PoseWithCovarianceStamped
from surfer_msgs.msg import Status
from surfer_msgs.srv import SetBehavior, GetBehaviors
#import tf2_py
## TODO: get behaviors service to populate website with options
valid_behaviors = ['CIRCLE','FOLLOW','WAYPOINT']

##TODO: Manager should own the behavior service and get the behaviors from a parameter file common to the autonomy node.
# this should allow the optional parameters to be loaded from the parameterr
# Down the line the parameters files can be configured from web interface

class SurferAutonomy(Node):

    def __init__(self):
        super().__init__('autonomy_node')
        self.status = Status()
        self.prev_status = Status()
        # Declare Parameters for Autonomy Make sure Parameters for your plugins are properly defined
        # Todo provide away for the user to set the parameters from interface

        self.declare_parameter('behaviors',[''])
        self.declare_parameter('loop_period',0.5)
        self.declare_parameter('radius',1.0)
        self.declare_parameter('speed',1.0)
        self.declare_parameter('loop',False)
        self.declare_parameter('prefix','/consensus')
        self.behaviors = self.get_parameter('behaviors').get_parameter_value().string_array_value


        self.pose_prefix = self.get_parameter('prefix').get_parameter_value().string_value
        print(self.pose_prefix)
        #print(self.get_parameter('waypoint/radius').get_parameter_value().double_value())
        self.plugin_list = self.load_plugins(self.behaviors)


        self.group_names = []
        #self.plugin = self.plugin_list.autonomy_plugin.waypoint()

        self.timer_period = self.get_parameter('loop_period').get_parameter_value().double_value  # seconds
        self.i = 0

        self.status_sub = self.create_subscription(Status,'status',self.status_cb,10)
        self.globe_status_sub = self.create_subscription(Status,'/global_status',self.global_status_cb,10)


        self.des_pose = Pose()
        self.params = 1
        #self.plugin.init(params)
        self.timer = self.create_timer(self.timer_period, self.timer_cb)


    def status_cb(self,msg):
        self.status = msg


    def global_status_cb(self,msg):
        if(msg.group == self.status.group):
            if(msg.name not in self.group_names):
                self.group_names.append(msg.name)


    def load_plugins(self,behaviors):
        #loader = pluginlib.PluginLoader(paths=['.'])
        plugins =[]
        module_list = []
        for x in behaviors:
            module_string = 'surfer_autonomy.'+x+'_plugin'
            module_list.append(module_string)

        loader = pluginlib.PluginLoader(modules=module_list)
        plugins = loader.plugins
        #print(loader)
        print(plugins)
        #module = plugins.autonomy_plugin.waypoint()
        return plugins



    def change_plugin(self,plugin_name):

        # Check if plugin object exists if so call stop method to cleanly finish
        # current behavior plugin
        try:
            self.plugin.stop()
          #  self.timer.destroy()
        except:
            self.get_logger().warning("No plugin is currently loaded none to stop")
        # Load new plugin object based on name from behavior status change
        print(plugin_name)

        self.plugin = self.plugin_list.autonomy_plugin[plugin_name]()
        #self.get_logger().info("loading plugin %s " % plugin_name)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry,'odom',self.plugin.odom_cb,1)
        self.wp_sub = self.create_subscription(Pose,'waypoint',self.plugin.wp_cb,1)
        self.path_sub = self.create_subscription(PoseArray,'path',self.plugin.path_cb,1)
        k = 0
        self.subs = []
        for name in self.group_names:
            sub = self.create_subscription(PoseWithCovarianceStamped,self.pose_prefix+'/'+name+'/pose',partial(self.plugin.poses_cb,name),1)
            self.subs.append(sub)
            k+=1

        self.plugin.set_publisher(self.cmd_vel_pub)
        self.plugin.set_status(self.status)
        self.plugin.init(self.params)


    def timer_cb(self):

        if(self.status.behavior != self.prev_status.behavior):
            self.change_plugin(self.status.behavior)
           # self.timer = self.create_timer(self.timer_period, self.timer_cb)


        if(self.status.mode == 'AUTO'):
            #try:
                self.plugin.run()
            #except:
            #    self.get_logger().error("Plugin Not Yet Selected")

        self.prev_status = self.status

def main(args=None):
    rclpy.init(args=args)
    sa = SurferAutonomy()
    rclpy.spin(sa)
    sa.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
