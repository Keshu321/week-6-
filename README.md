# 1. Managing Dependencies with rosdep

How does the rosdep tool work? We can utilize the software now that we understand the basics of rosdep, package.xml, and rosdistro. Before using rosdep for the first time, it must first be initialized using:

```
sudo rosdep init
rosdep update
```
![image](https://user-images.githubusercontent.com/92859942/196242453-4be81214-cb9e-487e-b88f-176c703afb11.png)


Finally, we could use rosdep install to install dependencies. In a workspace with multiple packages, this is frequently done just once to install all dependencies. If the directory src holding the source code was present in the workspace's root, a call for it may appear as follows.

```
rosdep install --from-paths src -y --ignore-src
```
![image](https://user-images.githubusercontent.com/92859942/196242601-8a0e88f7-56c3-4273-b687-2045933547c1.png)

# creating an action 

## requirenment 

we should have ROS 2 and colcon installed.

Set up a workspace and create a package named action_tutorials_interfaces:

(Remember to source your ROS 2 installation first.)

```
mkdir -p ros2_ws/src #you can reuse existing workspace with this naming convention
cd ros2_ws/src
ros2 pkg create action_tutorials_interfaces
```
![image](https://user-images.githubusercontent.com/92859942/196243171-fb28e9f2-120a-4b66-ace2-d75173749068.png)


# task 

# 1. defining an action 

Actions are described in .action files with the following format:
```
# Request
---
# Result
---
# Feedback
```

n instance of an action is typically referred to as a goal.

Say we want to define a new action “Fibonacci” for computing the Fibonacci sequence.

Create an action directory in our ROS 2 package action_tutorials_interfaces:

```
cd action_tutorials_interfaces
mkdir action
```
![image](https://user-images.githubusercontent.com/92859942/196244819-c8de7edc-47a9-41f1-bb43-d3192b8c9e1a.png)


# 2. Building an Action 
To use the new Fibonacci action in our code, we must send the definition to the Rosidl code. To achieve this, we must insert the lines below before the ament package() line in the action tutorials interfaces of our CMakeLists.txt file.

```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```
We must also include the necessary dependencies in our package.xml file:

```
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

After all these, we will be able to build the package containing the Fibonacci action definition as follows:

```
# Change to the root of the workspace
cd ~/ros2_ws
# Build
colcon build
```

Using the command line tool, we can verify that our action was built successfully:

```
# Source our workspace
# On Windows: call install/setup.bat
. install/setup.bash
# Check that our action definition exists
ros2 interface show action_tutorials_interfaces/action/Fibonacci
```

# Writing an action server and client

Here, were create a new file named: fibonacci_action_server.py in the home directory and added the following code:

## A. Writing an Action Server

Here, were create a new file named: fibonacci_action_server.py in the home directory and added the following code:

```
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Fibonacci.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
```

Now lets run our server:

```
python3 fibonacci_action_server.py
```
We can communicate a goal via the command line interface to another terminal:

```
ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```






