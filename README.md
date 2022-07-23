**参考自：[【古月居】自定义nodelet插件](https://www.guyuehome.com/33861)**

---

ROS的数据通信是以XML-RPC的方式,在graph结构中以topic,service和param的方式传输数据，天生的数据交互存在一定的延时和阻塞。Nodelet 包就是改善这一状况设计的， 使得多个算法运行在同一个过程中，并且算法间数据传输无需拷贝就可实现。 详见[http://wiki.ros.org/nodelet](http://wiki.ros.org/nodelet)。 简单的讲就是可以将以前启动的多个node捆绑在一起manager，使得同一个manager里面的topic的数据传输更快，数据通讯中roscpp采用boost shared pointer方式进行publish调用，实现zero copy。

下面将使用nodelet 创建一个ros的测试用例。

### 1. 创建工作空间

`mkdir -p nodelet_ws/src`

### 2. 创建 nodelet_test 包

`catkin_create_pkg nodelet_test roscpp std_msgs nodelet`

### 3. 创建 math_plus.cpp 文件

```cpp
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace math_test {
    /* code */
    class math_plus: public nodelet::Nodelet {
      public:
        math_plus(){}
        virtual ~math_plus(){}

      private:
        virtual void onInit() {
            ros::NodeHandle &private_nh = getPrivateNodeHandle();
            private_nh.getParam("value", value_);   // 通过参数服务器获取参数
            plus_result_pub_ = private_nh.advertise<std_msgs::Float64>("out", 10);
            plus_param_sub_ = private_nh.subscribe("in", 10, &math_plus::plus_process_callback, this);  // 将订阅名为 "/math_plus/in" 的话题数据
        }

        void plus_process_callback(const std_msgs::Float64::ConstPtr &input) {
            std_msgs::Float64Ptr output(new std_msgs::Float64());
            output->data = input->data + value_;
            NODELET_DEBUG("Adding %f to get %f", value_, output->data);
            plus_result_pub_.publish(output);   // 将相加后的结果以话题发布出去
        }

        ros::Publisher plus_result_pub_;
        ros::Subscriber plus_param_sub_;
        double value_;
    };
}

 // 插件注册(两个参数分别对应插件xml中的 type 和 base_class_type)
PLUGINLIB_EXPORT_CLASS(math_test::math_plus, nodelet::Nodelet);  
```

### 4. 创建 math_plus_plugin.xml 文件

```xml
<library path="lib/libmath_plus">
	<class name="nodelet_test/math_plus" type="math_test::math_plus" base_class_type="nodelet::Nodelet">
		<description>
			A node to add a value and republish.
		</description>
	</class>
</library>
```

### 5. 修改 CMakeLists.txt 文件

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(nodelet_test)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
nodelet
roscpp
std_msgs
)

catkin_package(
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)

add_library(math_plus
src/math_plus.cpp
)

add_dependencies(math_plus ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(math_plus
${catkin_LIBRARIES}
)
```

### 6. 修改 package 信息

```xml
<?xml version="1.0"?>
<package format="2">
	<name>nodelet_test</name>
	<version>0.0.0</version>
	<description>The nodelet_test package</description>
	<maintainer email="yyc@todo.todo">yyc</maintainer>
	<license>TODO</license>

	<buildtool_depend>catkin</buildtool_depend>
	<build_depend>nodelet</build_depend>
	<build_depend>roscpp</build_depend>
	<build_depend>std_msgs</build_depend>
	<build_export_depend>nodelet</build_export_depend>
	<build_export_depend>roscpp</build_export_depend>
	<build_export_depend>std_msgs</build_export_depend>
	<exec_depend>nodelet</exec_depend>
	<exec_depend>roscpp</exec_depend>
	<exec_depend>std_msgs</exec_depend>

	<export>
		<!-- 在此处添加 math_plus 的插件信息  -->
		<nodelet plugin="${prefix}/math_plus_plugin.xml"/>
	</export>
</package>
```

### 7. 创建 launch 文件

```xml
<launch>
	<node pkg="nodelet" type="nodelet" name="math_plus_manage" args="manager" output="screen"/>

	<node pkg="nodelet" type="nodelet" name="math_plus" args="load nodelet_test/math_plus math_plus_manage" output="screen">
		<param name="value" type="double" value="10"/>
	</node>
</launch>
```

### 8. 运行测试

#### 8.1. 编译

`catkin_make`

#### 8.2. 启动 nodelet

`roslaunch nodelet_test math_plus_launch.launch`

#### 8.3. 发布话题

`rostopic pub -r 1 /math_plus/in std_msgs/Float64 1`

#### 8.4. 查看输出

`rostopic echo /math_plus/out`
