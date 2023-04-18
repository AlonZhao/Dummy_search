#include<ros/ros.h>
//使用其他lib中定义的类
#include "occ_grid/occ_map.h"
//使用重新整合的显示函数
#include "visualization/visualization.hpp"
//建立客户端来发送请求
#include "self_msgs_and_srvs/GlbObsRcv.h"
//接收目标点
#include <geometry_msgs/PoseStamped.h>
#include <path_search/rrt.h>
//创建路径搜索类
class PathSearcher
{
    private:
    ros::NodeHandle nh_;
    //建立客户端发送地图发布的请求
    ros::ServiceClient rcv_glb_map_client_; 
    ros::Subscriber goal_sub_;
    ros::Timer execution_timer_;//定时发送请求直到有回复
    //路径搜索：地图
    env::OccMap::Ptr env_ptr;//shared_ptr<OccMap> Ptr;
    //路径搜索：可视化
    std::shared_ptr<visualization::Visualization> visual_ptr;
    //路径搜索：算法
    std::shared_ptr<path_search::RRT> rrt_ptr_;

    //路径搜索：全局目标
    Eigen::Vector3d start_,goal_;
    bool run_rrt_;
    
    public:

    PathSearcher(const ros::NodeHandle &nh):nh_(nh)
    {
        //初始化路径搜索成员
        env_ptr = std::make_shared<env::OccMap>();
        //路径搜索 地图成员初始化
        env_ptr->init(nh_);

        //路径搜索 可视化初始化
        visual_ptr = std::make_shared<visualization::Visualization>(nh_);
       //创建显示发布话题
        visual_ptr->registe<visualization_msgs::Marker>("start");
        visual_ptr->registe<visualization_msgs::Marker>("goal");
        
        //搜索算法指针 初始化参数包括ros句柄和地图指针
        rrt_ptr_ = std::make_shared<path_search::RRT>(nh_,env_ptr);
        //路径搜索中定义的发布和接收
        rcv_glb_map_client_ = nh_.serviceClient<self_msgs_and_srvs::GlbObsRcv>("/please_pub_map");
        //goal 是rviz的3d goal
        goal_sub_ = nh_.subscribe("/goal",1,&PathSearcher::goalCallback,this);
        //定时器请求地图
        execution_timer_ = nh_.createTimer(ros::Duration(1),&PathSearcher::executionCallback,this);
        
        start_.setZero();


    }
    ~PathSearcher(){};
    //点击触发
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal_msg)
    {
        //eigen类型 = msgs类型
        goal_[0] = goal_msg->pose.position.x;
        goal_[1] = goal_msg->pose.position.y;
        goal_[2] = goal_msg->pose.position.z;

        //hint
        // /goal是3dgoal点击对应的数据，goal是相对空间话题名 
      
        ROS_INFO_STREAM("\n-----------------------------\ngoal rcved at " << goal_.transpose());
        visual_ptr->visualize_a_ball(start_, 0.3, "start", visualization::Color::pink);
        visual_ptr->visualize_a_ball(goal_, 0.3, "goal", visualization::Color::steelblue);


    } 
    //定时请求地图
    void executionCallback(const ros::TimerEvent &e)
    {
        ROS_INFO("Timer\n");
        if(!env_ptr->mapValid())
        {
            //r无效
            ROS_INFO("no map received yet");
            self_msgs_and_srvs::GlbObsRcv srv;//新建服务信息
            if(!rcv_glb_map_client_.call(srv))
            {
                ROS_INFO("Fail to call map service");
            }
            else
            {
                //如果地图有效
                execution_timer_.stop();//终止定时器 终止该回调函数
            }

        }

    }


};

int main(int argc, char **argv)
{
    //建立ros
    ros::init(argc, argv, "Path_Search_Node");
    ros::NodeHandle nh("~");
    PathSearcher path_search_demo(nh);
    ros::AsyncSpinner spinner(0);//每个cpu一个线程
    spinner.start();
    ros::waitForShutdown();
    return 0;
}