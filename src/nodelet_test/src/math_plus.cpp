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


PLUGINLIB_EXPORT_CLASS(math_test::math_plus, nodelet::Nodelet);     // 插件注册(两个参数分别对应插件xml中的 type 和 base_class_type)