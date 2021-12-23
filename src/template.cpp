#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>>

namespace project_dil {

class ROS2Template : public rclcpp::Node {
public:
  ROS2Template();
  ~ROS2Template();
  
  void run();

private:
  void initParams();
  void initTopic();
  void initService();

  std::thread *thread_to_spin_;

  // Publisherの登録
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_pose_;
};

ROS2Template:: ROS2Template(): Node("dil_template"){
  initParams();
  initTopic();
  initService();

  // メインループ用スレッドの作成
  thread_to_spin_ = new std::thread([this]{
    rclcpp::spin(this->get_node_base_interface());
  });
  thread_to_spin_->detach();
}

ROS2Template::~ROS2Template(){
  // メインループ用スレッドの削除
  delete thread_to_spin_;
}

/**********
 * メインループ
 **********/
void ROS2Template::run(){
  // ループ周期の登録
  rclcpp::WallRate loop_rate(30);

  while(rclcpp::ok()){
    geometry_msgs::msg::Pose pose;
    
    // 前方
    pose.position.x = 1.0;
    pose.position.y = 0.0;

    // // 右前方
    // pose.position.x = 1.0;
    // pose.position.y = -0.5;

    // // 左前方
    // pose.position.x = 1.0;
    // pose.position.y = 0.5;

    pub_pose_->publish(pose);

    // 一定周期にするためのウェイト
    loop_rate.sleep();
  }
}

/**********
 * パラメータの登録
 **********/
void ROS2Template::initParams(){

}

/**********
 * トピックの登録
 **********/
void ROS2Template::initTopic(){
  pub_pose_ = this->create_publisher<geometry_msgs::msg::Pose>("/rs_path_following/target_pose", 10);
}

/**********
 * サービスの登録
 **********/
void ROS2Template::initService(){}

} // project_dil

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<project_dil::ROS2Template>();
    node->run();

    rclcpp::shutdown();
    return 0;
}