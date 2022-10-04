# A Simple Publisher and Subscriber

## 1 Create a package

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
![image](https://github.com/sanjiblama28/Github/blob/main/sp1.PNG)

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

Add the following dependencies following the lines above that match to the import declarations for your node:

```
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

![image](https://github.com/sanjiblama28/Github/blob/main/sp5.PNG)

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

![image](https://github.com/sanjiblama28/Github/blob/main/sp6.PNG)

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

![image](https://github.com/sanjiblama28/Github/blob/main/sp7.PNG)

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
![image](https://github.com/sanjiblama28/Github/blob/main/sp2%20(2).PNG)

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

![image](https://github.com/sanjiblama28/Github/blob/main/sp3.PNG)

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

![image](https://github.com/sanjiblama28/Github/blob/main/sp4.PNG)

Enter Ctrl+C in each terminal to stop the nodes from spinning

# A Simple Service and Client 

## 1 Create a package

Navigate into ros2_ws/src and run the package creation command:

```
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces
```

A notification from your terminal confirming the creation of your package py_srvcli and all of its required files and folders will be shown.

![image](https://github.com/sanjiblama28/Github/blob/main/ss1.PNG)

## 1.1 Update (package.xml)

You don't need to manually add dependencies to package.xml because you used the --dependencies option when creating the package.

But as always, remember to fill up package.xml with the description, maintainer's name and email, and license details.

```
<description>Python client server tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```
![image](https://github.com/sanjiblama28/Github/blob/main/ssp4.PNG)

## 1.2 Update (setup.py)

The maintainer, maintainer email, description, and license fields should all have the following information added to the setup.py file:

```
maintainer='Your Name',
maintainer_email='you@email.com',
description='Python client server tutorial',
license='Apache License 2.0',
```
![image](https://github.com/sanjiblama28/Github/blob/main/ssp5.PNG)

## 2 Write the service node

Create a new file called service_member_function.py in the ros2 ws/src/py_srvcli/py_srvcli directory, and then paste the following code inside:

```
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 2.1 Add an entry point

The entry point must be added to setup.py (found in the ros2 ws/src/py srvcli directory) in order for the ros2 run command to be able to execute your node.

The following line to be added in between the "console scripts" brackets:

```
'service = py_srvcli.service_member_function:main',
```

![image](https://github.com/sanjiblama28/Github/blob/main/ssp6.PNG)

## 3 Write the client node

Create a new file called client_member_function.py in the ros2 ws/src/py_srvcli/py_srvcli directory, and then paste the following code inside:

```
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
![image](https://github.com/sanjiblama28/Github/blob/main/ssp3.jpg)

## 3.1 Add an entry point

The client node requires an entry point to be added, much like the service node does.

Your setup.py file's entry points column needs to be formatted as follows:

```
entry_points={
    'console_scripts': [
        'service = py_srvcli.service_member_function:main',
        'client = py_srvcli.client_member_function:main',
    ],
},
```

![image](https://github.com/sanjiblama28/Github/blob/main/ssp7.PNG)

## 4 Build and Run

To check for missing dependencies before building, it's a good idea to run rosdep in the workspace's root directory (ros2 ws):

```
rosdep install -i --from-path src --rosdistro foxy -y
```

Navigate back to the root of your workspace, ros2_ws, and build your new package:

```
colcon build --packages-select py_srvcli
```

![image](https://github.com/sanjiblama28/Github/blob/main/ssp2.jpg)

Open a new terminal, navigate to ros2_ws, and source the setup files:

```
. install/setup.bash
```

Now run the service node:

```
ros2 run py_srvcli service
```
The node will await the request from the client.

Open a new terminal and once more source the setup files from ros2_ws. the client node, any two integers, and a space between them.

```
ros2 run py_srvcli client 2 3
```

The client would get a response like this if you selected options 2 and 3 as an example:

```
[INFO] [minimal_client_async]: Result of add_two_ints: for 2 + 3 = 5
```

![image](https://github.com/sanjiblama28/Github/blob/main/ss3.PNG)

The terminal where your service node is executing should be visited again. When it received the request, as you can see, it published the following log messages:

```
[INFO] [minimal_service]: Incoming request
a: 2 b: 3
```

![image](https://github.com/sanjiblama28/Github/blob/main/ssp1.jpg)

Enter Ctrl+C in each terminal to stop the nodes from spinning

# Creating custom msg and srv files

## 1 Create a new package

Navigate into ros2_ws/src and run the package creation command:

```
ros2 pkg create --build-type ament_cmake tutorial_interfaces
```

A notification from your terminal confirming the creation of your package tutorial_interfaces and all of its required files and folders will be shown. It should be noted that it is a CMake package because pure Python packages cannot yet generate.msg or.srv files. A Python node, which will be discussed in the last part, can use a custom interface that you design in a CMake package.

Maintaining.msg and.srv files in separate locations within a package is excellent practice. In ros2 ws/src/tutorial interfaces, create the directories.

```
mkdir msg

mkdir srv
```

![image](https://github.com/sanjiblama28/Github/blob/main/sss1.PNG)

## 2 Create custom definitions

## 2.1 msg definition 

Create a new file called Num.msg in the tutorial interfaces/msg directory that you just made, then add a single line of code stating the data structure in Num.msg:

```
int64 num
```
This custom message transmits the 64-bit integer num, which is one single value.

Create a new file called Sphere.msg in the tutorial interfaces/msg directory that you just established and fill it with the following information:

```
geometry_msgs/Point center
float64 radius
```
This custom message makes use of a message from a different message package (in this case, geometry msgs/Point).

## 2.2 srv definition

Create a new file called AddThreeInts.srv with the following request and response structure back in the tutorial interfaces/srv directory you just made:

```
int64 a
int64 b
int64 c
---
int64 sum
```
This is a custom service that accepts three integers with names a, b, and c and returns an answer with the integer sum.

![image](https://github.com/sanjiblama28/Github/blob/main/sss2.PNG)

## 3 CMakeLists.txt

Add the following lines to CMakeLists.txt to translate the interfaces you defined into language-specific code (such C++ and Python) so they may be utilized in those languages:

```
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
```

## 4 package.xml

These lines should be added to package.xml.

```
<depend>geometry_msgs</depend>

<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

## 5 Build the tutorial_interfaces package

You may construct your custom interfaces package now that all of its components are in place. Run the command below in the workspace's root (~/ros2_ws):

```
colcon build --packages-select tutorial_interfaces
```
![image](https://github.com/sanjiblama28/Github/blob/main/sss3.PNG)

Other ROS 2 programs will now be able to find the interfaces.

## 6 Confirm msg and srv creation

Run the following command from within your workspace (ros2 ws) to source it in a new terminal:

```
. install/setup.bash
```

The ros2 interface show command can now be used to verify that your interface creation was successful:

```
ros2 interface show tutorial_interfaces/msg/Num
```
should return:
```
int64 num
```
And
```
ros2 interface show tutorial_interfaces/msg/Sphere
```
should return:
```
geometry_msgs/Point center
        float64 x
        float64 y
        float64 z
float64 radius
```
And
```
ros2 interface show tutorial_interfaces/srv/AddThreeInts
```
should return:

```
int64 a
int64 b
int64 c
---
int64 sum
```
![image](https://github.com/sanjiblama28/Github/blob/main/sss4.PNG)

## 7 Test the new interfaces

You can utilize the packages you made in earlier instructions for this step. You may use your new interfaces by making a few straightforward changes to the nodes, CMakeLists, and package files.

## 7.1 Testing Num.msg with pub/sub

You may see Num.msg in action by making a few minor adjustments to the publisher/subscriber package developed in a prior tutorial (C++ or Python). The output will be slightly different because you'll be switching from the default string message to a numerical one.

Publisher:

```
import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Num    # CHANGE


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Num, 'topic', 10)     # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Num()                                           # CHANGE
        msg.num = self.i                                      # CHANGE
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.num)  # CHANGE
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Subscriber:

```
import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Num        # CHANGE


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Num,                                              # CHANGE
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
            self.get_logger().info('I heard: "%d"' % msg.num) # CHANGE


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

CMakeLists.txt:

Add the following lines (C++ only):

```
#...

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)                         # CHANGE

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp tutorial_interfaces)         # CHANGE

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp tutorial_interfaces)     # CHANGE

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

package.xml:

Add the following line:

```
<exec_depend>tutorial_interfaces</exec_depend>
```

After making the above edits and saving all the changes, build the package:

```
colcon build --packages-select py_pubsub
```
On Windows:

```
colcon build --merge-install --packages-select py_pubsub
```

Then open two new terminals, source ros2_ws in each, and run:

```
ros2 run py_pubsub listener

```

![image](https://github.com/sanjiblama28/Github/blob/main/sss5.PNG)

```
ros2 run py_pubsub listener

```

![image](https://github.com/sanjiblama28/Github/blob/main/sss7.PNG)

The talker should only be publishing integer values as opposed to the string it previously published as Num.msg only relays an integer:

```
[INFO] [minimal_publisher]: Publishing: '0'
[INFO] [minimal_publisher]: Publishing: '1'
[INFO] [minimal_publisher]: Publishing: '2'
```

## 7.2 Testing AddThreeInts.srv with service/client

You may use AddThreeInts.srv by making a few minor adjustments to the service/client package developed in a prior tutorial (in C++ or Python). The output will alter significantly because you'll be switching from the initial two integer request srv to a three integer request srv.

Service:

```
from tutorial_interfaces.srv import AddThreeInts     # CHANGE

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)        # CHANGE

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c                                                  # CHANGE
        self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c)) # CHANGE

        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Client:

```
from tutorial_interfaces.srv import AddThreeInts       # CHANGE
import sys
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddThreeInts.Request()                                   # CHANGE

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.req.c = int(sys.argv[3])                  # CHANGE
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_three_ints: for %d + %d + %d = %d' %                               # CHANGE
                    (minimal_client.req.a, minimal_client.req.b, minimal_client.req.c, response.sum)) # CHANGE
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

CMakeLists.txt:

Add the following lines (C++ only):

```
#...

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)        # CHANGE

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server
  rclcpp tutorial_interfaces)                      #CHANGE

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client
  rclcpp tutorial_interfaces)                      #CHANGE

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

package.xml:

Add the following line:

```
<exec_depend>tutorial_interfaces</exec_depend>
```

After making the above edits and saving all the changes, build the package:

```
colcon build --packages-select py_srvcli
```

On Windows:

```
colcon build --merge-install --packages-select py_srvcli
```

Then open two new terminals, source ros2_ws in each, and run:

```
ros2 run py_srvcli service
```

![image](https://github.com/sanjiblama28/Github/blob/main/sss8.PNG)

```
ros2 run py_srvcli client 2 3 1
```

![image](https://github.com/sanjiblama28/Github/blob/main/sss9.PNG)





















