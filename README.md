CONTENT :

 MANAGING DEPENDENCIES WITH ROSDEP
 
 CREATING AN ACTION
 
 WRITING AN ACTION SERVER AND CLIENT 
 
 COMPOSING MULTIPLE NODES IN A SINGLE PROCESS
 
 LAUNCH :
 
     - Creating a launch file.
     -Launching and monitoring multiple nodes
     - Using substitutions
     - Using event handlers


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

![image](https://user-images.githubusercontent.com/92859942/196399859-1850b216-e41d-4f50-b839-eae66b37e20c.png)

We must also include the necessary dependencies in our package.xml file:

```
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```
![image](https://user-images.githubusercontent.com/92859942/196400072-98a5e077-6d92-4968-b69a-72f93def9c51.png)

After all these, we will be able to build the package containing the Fibonacci action definition as follows:

```
# Change to the root of the workspace
cd ~/ros2_ws
# Build
colcon build
```
![image](https://user-images.githubusercontent.com/92859942/196400205-679c41c4-2cfe-4043-a82e-6cb7a2582beb.png)

Using the command line tool, we can verify that our action was built successfully:

```
# Source our workspace
# On Windows: call install/setup.bat
. install/setup.bash
# Check that our action definition exists
ros2 interface show action_tutorials_interfaces/action/Fibonacci
```
![image](https://user-images.githubusercontent.com/92859942/196400347-d41749e6-8623-40e8-ae21-7ca92a9eb7d4.png)

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
![image](https://user-images.githubusercontent.com/92859942/196408665-05016d5a-1579-48c0-89a8-82281b4c7b84.png)

Now lets run our server:

```
python3 fibonacci_action_server.py
```
![image](https://user-images.githubusercontent.com/92859942/196408828-3dca3ca2-da5a-4213-a7bd-1088c66a1c65.png)


We can communicate a goal via the command line interface to another terminal:

```
ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```
![image](https://user-images.githubusercontent.com/92859942/196408915-ec7e1ec2-b5f9-4de1-850c-5c0759ad2a0a.png)

In the terminal that is running the action server, you should see the logged message "Executing objective..." followed by a warning that the goal state was not established. If the goal handle state is not set in the execute callback, the aborted state is assumed by default.

To demonstrate that the objective was accomplished, use the successfully() function on the goal handle:

```
def execute_callback(self, goal_handle):
    self.get_logger().info('Executing goal...')

    goal_handle.succeed()

    result = Fibonacci.Result()
    return result
```

![image](https://user-images.githubusercontent.com/92859942/196409881-5735d95b-e904-48c3-a96d-60b1fa6fa9af.png)


You should see the goal completed with the status SUCCEED if you restart the action server and send another goal at this point.

![image](https://user-images.githubusercontent.com/92859942/196410744-0ce7dc67-5180-40c7-bd02-c676b583876c.png)

![image](https://user-images.githubusercontent.com/92859942/196411241-935ba425-5d25-4269-a02f-955dfa40e930.png)

Let's now make sure that our target execution computes and returns the specified Fibonacci sequence:
```
def execute_callback(self, goal_handle):
    self.get_logger().info('Executing goal...')


    sequence = [0, 1]



    for i in range(1, goal_handle.request.order):

        sequence.append(sequence[i] + sequence[i-1])


    goal_handle.succeed()

    result = Fibonacci.Result()

    result.sequence = sequence

    return result
```
![image](https://user-images.githubusercontent.com/92859942/196412099-3ec2fe82-38f2-474d-aea8-ec6b3cc2d3c9.png)


The sequence is calculated, put in the result message field, and we move on to the return.

Restart the action server and send a new goal. The objective must be fulfilled with the anticipated outcomes appearing in the appropriate order. The successfully() function on the goal handle can be used to demonstrate the goal's accomplishment:

![image](https://user-images.githubusercontent.com/92859942/196419464-3c0803f0-cecc-40be-8b14-bbacd0626d8a.png)

![image](https://user-images.githubusercontent.com/92859942/196419527-44e35601-b4a5-455c-900c-454fed01979c.png)


## A.1 publishing feedback: 

By invoking the publsih feedback() function on the goal handle, we can force our action server to publish feedback for action clients.

after using a feedback message to save the sequence in place of the sequence variable. We broadcast the feedback message with each update to the for-loop feedback message:

```
import time


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


        feedback_msg = Fibonacci.Feedback()

        feedback_msg.partial_sequence = [0, 1]


        for i in range(1, goal_handle.request.order):

            feedback_msg.partial_sequence.append(

                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])

            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))

            goal_handle.publish_feedback(feedback_msg)

            time.sleep(1)


        goal_handle.succeed()

        result = Fibonacci.Result()

        result.sequence = feedback_msg.partial_sequence

        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
```
After restarting the action server, it is confirmed that feedback is now published by using the command line tool with the --feedback option:

```
ros2 action send_goal --feedback fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```
![image](https://user-images.githubusercontent.com/92859942/196525373-16375850-54d0-4876-bd85-3c1dde1780de.png)

![image](https://user-images.githubusercontent.com/92859942/196423218-b616c53d-b6d8-41f0-9ba0-ac8505eedad5.png)


## B. Writing an action client

The action client will also be limited to a single file. Next, create a new file with the name fibonacci action client.py. To the new file, add the following boilerplate code:

![image](https://user-images.githubusercontent.com/92859942/196526351-b9ed22d2-30c1-4e63-914e-1804b06a44a3.png)


```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    future = action_client.send_goal(10)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
```

![image](https://user-images.githubusercontent.com/92859942/196526226-ecab7673-e459-4bc6-87d5-93fa4b44172f.png)


Let's test our action client by first launching the earlier-built action server:

```
python3 fibonacci_action_server.py
```

Run the action client in an other terminal.

```
python3 fibonacci_action_client.py
```

he action server executes the goal and we are able to see the messages successfully.

The action client start up and quickly finish but we don't get any feedback.

```
[INFO] [fibonacci_action_server]: Executing goal...
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5])
# etc.
```

![image](https://user-images.githubusercontent.com/92859942/196529165-c1e40929-d9aa-4d1e-b268-6d1c88cf6b83.png)

## B.1 getting the result 

We need to set a goal handle for the goal we sent and hence here is the complete code for this:

```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

Try running our Fibonacci action client while an action server is active on a different terminal.

```
python3 fibonacci_action_client.py
```

![image](https://user-images.githubusercontent.com/92859942/196529909-83fa97db-60aa-4829-a23f-3aa5cdb0d2c3.png)


## B.2 getting feedback 

Here is the whole code for collecting some feedback regarding the goals we provide from the action server once the action client is allowed to communicate the objectives:

```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

Everything is ready for us. Your screen should display feedback if we run our action client.

```
python3 fibonacci_action_client.py
```

![image](https://user-images.githubusercontent.com/92859942/196530529-f7db7fc6-4050-4ab1-909a-a81f8d353c00.png)

we created an action. 

# 3. Composing multiple nodes in a single process

## To discover avilable components

In order to check the available components in the workspace, we run the following commands.

```
ros2 component types
```

![image](https://user-images.githubusercontent.com/92859942/196531590-83d5f202-93df-427a-8510-f329d24088c9.png)

## Run-time composition using ROS services with a publisher and subscriber

Run-time composition using ROS services with a publisher and subscriber

```
ros2 run rclcpp_components component_container
```
Using the ros2 command-line tools, we run the following command in the second terminal to show the name of the component as an output and confirm that the container is operating.

```
ros2 component list
```
![image](https://user-images.githubusercontent.com/92859942/196532868-bef5e720-419d-412b-96e7-5a385d98277c.png)

After this step, in the second terminal, we load the talker component:

```
ros2 component load /ComponentManager composition composition::Talker
```

After this, we run following code in the second terminal in order to load the listener component:

```
ros2 component load /ComponentManager composition composition::Listener
```
![image](https://user-images.githubusercontent.com/92859942/196533209-376650cb-b9cc-40b3-afeb-a41394c1b62f.png)


![image](https://user-images.githubusercontent.com/92859942/196533016-73b99ccb-e614-4452-923e-2bcdc1f0b252.png)

This command will return the node name and the distinctive ID of the loaded component:

Finally we can run the ros2 command line utility to inspect the state of the container:

```
ros2 component list
```
We can see the result as follows:

```
/ComponentManager
   1  /talker
   2  /listener
```
## Run-time composition using ROS services with a server and client
The steps are pretty similar to what we performed with the talker and listener.

Our first terminal is where we execute:

```
ros2 run rclcpp_components component_container
```
and after that, in the second terminal, we run following commands to see server and client source code:


```
ros2 component load /ComponentManager composition composition::Server
ros2 component load /ComponentManager composition composition::Client
```

![image](https://user-images.githubusercontent.com/92859942/196535552-bf513e3b-85ae-4f5d-b0dd-806cdade9c13.png)


![image](https://user-images.githubusercontent.com/92859942/196535376-146ee3c7-31fb-4220-8e91-8fa1448f901f.png)

## Compile-time composition using ROS services

This example demonstrates how the same shared libraries may be used to create a single executable that runs a number of different components.

All four of the aforementioned parts—talker, listener, server, and client—are present in the executable.

in one terminal 

```
ros2 run composition manual_composition
```
![image](https://user-images.githubusercontent.com/92859942/196536062-320156c7-c06f-4b5e-9d73-6a22e88c7dda.png)

## Run-time composition using dlopen

By constructing a generic container process and explicitly passing the libraries to load without using ROS interfaces, this demonstration provides an alternative to run-time composition. Each library will be opened by the procedure, and one instance of each "rclcpp::Node" class will be created in the library's source code.


```
ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so
```

![image](https://user-images.githubusercontent.com/92859942/196536362-d01b5815-4470-4c80-8315-1a8a79b8a98f.png)

## Composition using launch actions

While the command line tools are helpful for troubleshooting and diagnosing component setups, starting a group of components at once is frequently more practical. We can make use of ros2 launch's functionality to automate this process.


```
ros2 launch composition composition_demo.launch.py
```

![image](https://user-images.githubusercontent.com/92859942/196536675-9ed0ad18-9eae-44ed-afd6-8390c876428e.png)


# 4. Creating a launch file

We use the rqt graph and turtlesim packages that we previously installed in order to produce a launch file.

## setup and writing the lunch file 

we should make new directory and create new file named turtlesim_mimic_launch.py and use the mention code in the file

```
mkdir launch
```

![image](https://user-images.githubusercontent.com/92859942/196538918-5213acd4-e1a3-42c7-aa12-9138c1a4668a.png)

code:
```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```
![image](https://user-images.githubusercontent.com/92859942/196539540-b2972fdf-9eba-40fb-b565-f38c457bc884.png)

## ros2 launch

In order to run the launch file created, we enter into the earlier created directory and run the following commands:

```
cd launch
ros2 launch turtlesim_mimic_launch.py
```

![image](https://user-images.githubusercontent.com/92859942/196540724-cc3ee018-c4b9-4db1-923a-4cdae1009647.png)



To see the system in action, open a new terminal and run the ros2 topic pub command on the /turtlesim1/turtle1/cmd_vel topic to get the first turtle moving:

```
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```

![image](https://user-images.githubusercontent.com/92859942/196543152-454bb774-8a15-4f21-a07c-772d8c91fc03.png)


## Introspect the system with rqt_graph
Open the new terminal without closing the system and we run rqt_graph.

```
rqt_graph
```
![image](https://user-images.githubusercontent.com/92859942/196543919-a18b22f0-45ac-46ce-8562-56c5bf725dac.png)

# 5. Integrating launch files into ROS2 packages

## Creating a package

Firstly, we create a workspace for the package:

```
mkdir -p launch_ws/src
cd launch_ws/src
```
and create a python package:

```
ros2 pkg create py_launch_example --build-type ament_python
```
![image](https://user-images.githubusercontent.com/92859942/196546845-3949bfdc-369c-4fe0-8592-2e0b0276c8c9.png)

##  Creating the structure to hold launch files


After creating the packages, it should be looking as follows for python package:

![image](https://user-images.githubusercontent.com/92859942/196548000-b2ecafcd-c546-4302-95a7-ba26d89a1e3e.png)


And utilizing the data files setup argument, we must tell Python's setup tools about our launch files in order to colcon to launch files.

We enter these codes in the setup.py file:

```
import os
from glob import glob
from setuptools import setup

package_name = 'py_launch_example'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)
```
## Writing the launch file

Inside your launch directory, create a new launch file called my_script_launch.py. _launch.py is recommended, but not required, as the file suffix for Python launch files.

Your launch file should define the generate_launch_description() function which returns a launch.LaunchDescription() to be used by the ros2 launch verb.

```
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker'),
  ])
```
## Building and running the launch file

We go to top-level of the workspace and build the file using:
```
colcon build
```
After the colcon build has been successful and you’ve sourced the workspace, you should be able to run the launch file as follows:

```
ros2 launch py_launch_example my_script_launch.py
```

# 6. Using Substitutions


We create a new package of build_type ament_python named launch_tutorial :

```
ros2 pkg create launch_tutorial --build-type ament_python
```
and inside of that package, we create a directory called launch.

```
mkdir launch_tutorial/launch
```
The setup.py file is then modified and adjustments are included to ensure a successful installation of the launch file.

```
import os
from glob import glob
from setuptools import setup

package_name = 'launch_tutorial'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)
```
## Parent Launch File

After the above steps, we created a launch file named : example_main.launch.py in the launch folder of the launch_tutorial directory with the following codes in it:

```
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_tutorial'),
                    'example_substitutions.launch.py'
                ])
            ]),
            launch_arguments={
                'turtlesim_ns': 'turtlesim2',
                'use_provided_red': 'True',
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        )
    ])
```

## Substitutions example launch file

The same folder gains a new document called example substitutions.launch.py.

```
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        spawn_turtle,
        change_background_r,
        TimerAction(
            period=2.0,
            actions=[change_background_r_conditioned],
        )
    ])
```
## Building the package

We run the build command in the root of the workspace.

```
colcon build
```
## Launching Example

Now we can use the ros2 launch command to execute the example main.launch.py file.

```
ros2 launch launch_tutorial example_main.launch.py
```

![image](https://user-images.githubusercontent.com/92859942/196817954-cefdc716-4489-4e12-b119-6576ff9e1d2b.png)

A blue backdrop is used while starting a turtlesim node. Following that, a second turtle hatches, and the backgrounds are purple and pink, respectively.

# 7. Using Event Handlers
 
 Event handler example launch file
 
 We created a new file named: example_event_handlers.launch.py in the same directory.. i.e. inside launch folder of launch_tutorial package.

```
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=turtlesim_node,
                on_start=[
                    LogInfo(msg='Turtlesim started, spawning turtle'),
                    spawn_turtle
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessIO(
                target_action=spawn_turtle,
                on_stdout=lambda event: LogInfo(
                    msg='Spawn request says "{}"'.format(
                        event.text.decode().strip())
                )
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=spawn_turtle,
                on_completion=[
                    LogInfo(msg='Spawn finished'),
                    change_background_r,
                    TimerAction(
                        period=2.0,
                        actions=[change_background_r_conditioned],
                    )
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=turtlesim_node,
                on_exit=[
                    LogInfo(msg=(EnvironmentVariable(name='USER'),
                            ' closed the turtlesim window')),
                    EmitEvent(event=Shutdown(
                        reason='Window closed'))
                ]
            )
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason')]
                )]
            )
        ),
    ])
```

![image](https://user-images.githubusercontent.com/92859942/196819121-44a0f48f-af1e-4c63-b295-b701ad84adf5.png)

## Building and Running the Command
 
 After adding the file, we go back to the root of the workspace and run the build command there.

```
colcon build
```
 
 
It is crucial to source the package and execute the following commands for the output after building:

ros2 launch example event handlers.launch.py launch tutorial Use provided red:=True, turtlesim ns:='turtlesim3', and new background r:=200

![image](https://user-images.githubusercontent.com/92859942/196819215-88d3c78d-2b44-4e8c-b7b5-4685f6a5a414.png)


It spawns the second turtle and starts a turtlesim node with a blue backdrop. The background color then changes to pink and then purple. When the turtlesim window closes, the launch file also shuts down.








