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

In the terminal that is running the action server, you should see the logged message "Executing objective..." followed by a warning that the goal state was not established. If the goal handle state is not set in the execute callback, the aborted state is assumed by default.

To demonstrate that the objective was accomplished, use the successfully() function on the goal handle:

```
def execute_callback(self, goal_handle):
    self.get_logger().info('Executing goal...')

    goal_handle.succeed()

    result = Fibonacci.Result()
    return result
```
You should see the goal completed with the status SUCCEED if you restart the action server and send another goal at this point.

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

The sequence is calculated, put in the result message field, and we move on to the return.

Restart the action server and send a new goal. The objective must be fulfilled with the anticipated outcomes appearing in the appropriate order. The successfully() function on the goal handle can be used to demonstrate the goal's accomplishment:


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

## B. Writing an action client

The action client will also be limited to a single file. Next, create a new file with the name fibonacci action client.py. To the new file, add the following boilerplate code:

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

we created an action. 

# 3. Composing multiple nodes in a single process

