# A Simple Publisher and Subscriber

## 1 Create a packege

Navigate into ros2_ws/src and run the package creation command:

```
ros2 pkg create --build-type ament_python py_pubsub
```
A notification from your terminal confirming the creation of your package py pubsub and all of its required files and folders will be shown.

## 2 Write the publisher node

Navigate into ros2_ws/src/py_pubsub/py_pubsub and by executing the following command, the example talker code can be downloaded:

```
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
```
A new file called publisher member function.py will now be present next to __init .py.
Use the text editor of your choice to open the file.

```
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
## 2.1 Add dependencies

Navigate one level back to the ros2_ws/src/py_pubsub directory, where the setup.py, setup.cfg, and package.xml files have been created for you.

Use your text editor to open package.xml, and be sure to complete the description>, maintainer>, and license> tags:

```
<description>Examples of minimal publisher/subscriber using rclpy</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

![image](https://user-images.githubusercontent.com/90166739/193400425-e84fc324-d121-406d-9ad8-0b89ec2588a3.png)

Add the following dependencies following the lines above that match to the import declarations for your node:

```
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

![image](https://user-images.githubusercontent.com/90166739/193400604-116612a9-6e3f-4e3f-9ff9-dd70fc3375e5.png)

This declares that when the package's code is executed, rclpy and std msgs are required.

Ensure that the file is saved.

## 2.2 Add an entry point

Check out the setup.py file. Make sure to match the maintainer, maintainer email, description, and license columns to your package.xml once more:

```
maintainer='YourName',
maintainer_email='you@email.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache License 2.0',
```
Within the console scripts brackets of the entry points field, add the following line:

```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
```
Remember to save.

## 2.3 Check setup.cfg

The setup.cfg file should automatically contain the following information:

```
[develop]
script-dir=$base/lib/py_pubsub
[install]
install-scripts=$base/lib/py_pubsub
```
Simply instruct setuptools to place your executables in the lib directory, where ros2 run will look for them.

If you wanted to see the entire system in action, you could build your package right now, source the local setup files, and launch it. However, let's first create the subscriber node.

## 3 Write the subscriber node

The next node can be created by going back to ros2 ws/src/py pubsub/py pubsub. Fill out your terminal with the following code:

```
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```
Now, the directory must include the following files:

```
__init__.py  publisher_member_function.py  subscriber_member_function.py
```

Now, Open the subscriber_member_function.py with your text editor.

```
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 3.1 Add an entry point

Reopen setup.py and place the subscriber node's entry point beneath the publisher's entry point. Now, the entry points field should be as follows:

```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
```
Once the file has been saved, your pub/sub system should be operational.

## 4 Build and Run

The rclpy and std msgs packages are probably already installed on your ROS 2 system. Before building, it's best practice to run rosdep in the workspace's root directory (ros2 ws) to check for any missing dependencies:

```
rosdep install -i --from-path src --rosdistro foxy -y
```

Still in the root of your workspace, ros2_ws, build your new package:

```
colcon build --packages-select py_pubsub
```

Open a new terminal, navigate to ros2_ws, and source the setup files:

```
. install/setup.bash
```

Now run the talker node:

```
ros2 run py_pubsub talker
```
Starting in 0.5 seconds, the terminal should begin sending out info messages as follows:

```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [minimal_publisher]: Publishing: "Hello World: 3"
[INFO] [minimal_publisher]: Publishing: "Hello World: 4"
```

![image](https://user-images.githubusercontent.com/90166739/193401313-a69a23af-fae1-4474-b080-409fca50a5a6.png)

Launch a new terminal, once more source the setup files from ros2 ws, and then launch the listener node:

```
ros2 run py_pubsub listener
```

Starting at the publisher's current message count, the listener will begin writing messages to the console as follows:

```
[INFO] [minimal_subscriber]: I heard: "Hello World: 10"
[INFO] [minimal_subscriber]: I heard: "Hello World: 11"
[INFO] [minimal_subscriber]: I heard: "Hello World: 12"
[INFO] [minimal_subscriber]: I heard: "Hello World: 13"
[INFO] [minimal_subscriber]: I heard: "Hello World: 14"
```

![image](https://user-images.githubusercontent.com/90166739/193401418-6f192b1e-26b6-4b04-84c3-e0da0d6768dd.png)

Enter Ctrl+C in each terminal to stop the nodes from spinning


