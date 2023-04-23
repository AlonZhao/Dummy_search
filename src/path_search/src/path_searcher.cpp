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
#include <geometry_msgs/PoseStamped.h>
#include <path_search/rrt_star.h>
#include <path_search/brrt_star.h>
#include <path_search/brrt.h>
#include <path_search/rrt_sharp.h>

#include<path_search/astar.h>
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
    std::shared_ptr<path_search::RRTStar> rrt_star_ptr_;
    std::shared_ptr<path_search::RRTSharp> rrt_sharp_ptr_;
    std::shared_ptr<path_search::BRRT> brrt_ptr_;
    std::shared_ptr<path_search::BRRTStar> brrt_star_ptr_;
    std::shared_ptr<fast_planner::Astar> astar_ptr_;

    //路径搜索：全局目标
    Eigen::Vector3d start_,goal_;
    bool run_rrt_, run_rrt_star_,run_rrt_sharp_,run_brrt_,run_brrt_star_,run_astar_;
    
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
    //-----------------rrt------------------------//
        //搜索算法指针 初始化参数包括ros句柄和地图指针
        rrt_ptr_ = std::make_shared<path_search::RRT>(nh_,env_ptr);
        rrt_ptr_->setVisualizer(visual_ptr);
        //路径搜索中定义的发布和接收
        visual_ptr->registe<nav_msgs::Path>("rrt_final_path");
        visual_ptr->registe<sensor_msgs::PointCloud2>("rrt_final_wpts");
    //------------------rrt star-------------------------//
        rrt_star_ptr_ = std::make_shared<path_search::RRTStar>(nh_, env_ptr);
        rrt_star_ptr_->setVisualizer(visual_ptr);
        visual_ptr->registe<nav_msgs::Path>("rrt_star_final_path");
        visual_ptr->registe<sensor_msgs::PointCloud2>("rrt_star_final_wpts");
        visual_ptr->registe<visualization_msgs::MarkerArray>("rrt_star_paths");
    //---------rrt sharp-------
        rrt_sharp_ptr_ = std::make_shared<path_search::RRTSharp>(nh_, env_ptr);
        rrt_sharp_ptr_->setVisualizer(visual_ptr);
        visual_ptr->registe<nav_msgs::Path>("rrt_sharp_final_path");
        visual_ptr->registe<sensor_msgs::PointCloud2>("rrt_sharp_final_wpts");
        visual_ptr->registe<visualization_msgs::MarkerArray>("rrt_sharp_paths");
//----brrt----------------------------
        brrt_ptr_ = std::make_shared<path_search::BRRT>(nh_, env_ptr);
        brrt_ptr_->setVisualizer(visual_ptr);
        visual_ptr->registe<nav_msgs::Path>("brrt_final_path");
        visual_ptr->registe<sensor_msgs::PointCloud2>("brrt_final_wpts");
        visual_ptr->registe<visualization_msgs::MarkerArray>("brrt_paths");

//----brrt_star----------------------------
        brrt_star_ptr_ = std::make_shared<path_search::BRRTStar>(nh_, env_ptr);
        brrt_star_ptr_->setVisualizer(visual_ptr);
        visual_ptr->registe<nav_msgs::Path>("brrt_star_final_path");
        visual_ptr->registe<sensor_msgs::PointCloud2>("brrt_star_final_wpts");
        visual_ptr->registe<visualization_msgs::MarkerArray>("brrt_star_paths");
//--------------------astar---------------
        astar_ptr_.reset(new fast_planner::Astar);
        astar_ptr_->setParam(nh_);
        astar_ptr_->setEnvironment(env_ptr);
        astar_ptr_->init();

        astar_ptr_->setVisualizer(visual_ptr);
        visual_ptr->registe<nav_msgs::Path>("astar_final_path");
        visual_ptr->registe<sensor_msgs::PointCloud2>("astar_final_wpts");
        visual_ptr->registe<visualization_msgs::MarkerArray>("astar_paths");


        rcv_glb_map_client_ = nh_.serviceClient<self_msgs_and_srvs::GlbObsRcv>("/please_pub_map");
        //goal 是rviz的3d goal  绝对空间命名
        goal_sub_ = nh_.subscribe("/goal",1,&PathSearcher::goalCallback,this);
        //定时器请求地图
        execution_timer_ = nh_.createTimer(ros::Duration(1),&PathSearcher::executionCallback,this);
        
       // start_.setZero();
       start_<<-6.3101,-7.1391,1;
        //规划策略选择
        nh_.param("run_rrt",run_rrt_,true);
        nh_.param("run_rrt_star",run_rrt_star_,true);
        nh_.param("run_rrt_sharp",run_rrt_sharp_,true);
        nh_.param("run_brrt_star",run_brrt_star_,true);
        nh_.param("run_brrt",run_brrt_,true);
        nh_.param("run_astar",run_astar_,true);
 

    }
    ~PathSearcher(){};
    //点击触发
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal_msg)
    {
        //eigen类型 = msgs类型
        goal_[0] = goal_msg->pose.position.x;
        goal_[1] = goal_msg->pose.position.y;
        goal_[2] = 1;
        //goal_[2] = goal_msg->pose.position.z;

        //hint
        // /goal是3dgoal点击对应的数据，goal是相对空间话题名 
      
        ROS_INFO_STREAM("\n-----------------------------\ngoal rcved at " << goal_.transpose());
        visual_ptr->visualize_a_ball(start_, 0.3, "start", visualization::Color::pink);
        visual_ptr->visualize_a_ball(goal_, 0.3, "goal", visualization::Color::steelblue);
        
        if(run_rrt_)
        {
            
            //路径搜索
            bool rrt_result = rrt_ptr_->plan(start_, goal_);
            if (rrt_result)
            {
                vector<vector<Eigen::Vector3d>> routes = rrt_ptr_->getAllPaths();
                //visual_ptr->visualize_path_list(routes, "rrt_paths", visualization::blue);
                vector<Eigen::Vector3d> final_path = rrt_ptr_->getPath();
                visual_ptr->visualize_path(final_path,"rrt_final_path");
                visual_ptr->visualize_pointcloud(final_path,"rrt_final_wpts");
                vector<std::pair<double, double>>slns = rrt_ptr_->getSolutions();
                ROS_INFO_STREAM("[RRT] final path len: " << slns.back().first);
            }
            else
            {
                ROS_INFO("RRT Plan Failed");
            }
        }

        if(run_rrt_star_)
        {
            
            //路径搜索
            bool rrt_star_result = rrt_star_ptr_->plan(start_, goal_);
            if (rrt_star_result)
            {
                vector<vector<Eigen::Vector3d>> routes = rrt_star_ptr_->getAllPaths();
                visual_ptr->visualize_path_list(routes, "rrt_star_paths", visualization::blue);
                vector<Eigen::Vector3d> final_path = rrt_star_ptr_->getPath();
                visual_ptr->visualize_path(final_path,"rrt_star_final_path");
                visual_ptr->visualize_pointcloud(final_path,"rrt_star_final_wpts");
                vector<std::pair<double, double>>slns = rrt_star_ptr_->getSolutions();
                ROS_INFO_STREAM("[RRT*] final path len: " << slns.back().first);
            }
            else
            {
                ROS_INFO("RRT_STAR Plan Failed2");
            }
        }

        if(run_rrt_sharp_)
        {
            
            //路径搜索
            bool rrt_sharp_result = rrt_sharp_ptr_->plan(start_, goal_);
            if (rrt_sharp_result)
            {
                vector<vector<Eigen::Vector3d>> routes = rrt_sharp_ptr_->getAllPaths();
                visual_ptr->visualize_path_list(routes, "rrt_star_paths", visualization::blue);
                vector<Eigen::Vector3d> final_path = rrt_sharp_ptr_->getPath();
                visual_ptr->visualize_path(final_path,"rrt_sharp_final_path");
                visual_ptr->visualize_pointcloud(final_path,"rrt_sharp_final_wpts");
                vector<std::pair<double, double>>slns = rrt_sharp_ptr_->getSolutions();
                ROS_INFO_STREAM("[RRT#] final path len: " << slns.back().first);
            }
            else
            {
                ROS_INFO("RRT_STAR Plan Failed2");
            }
        }
        if(run_brrt_)
        {
          
        //路径搜索
            bool brrt_result = brrt_ptr_->plan(start_, goal_);
            if (brrt_result)
            {
                vector<vector<Eigen::Vector3d>> routes = brrt_ptr_->getAllPaths();
                //visual_ptr->visualize_path_list(routes, "rrt_paths", visualization::blue);
                vector<Eigen::Vector3d> final_path = brrt_ptr_->getPath();
                visual_ptr->visualize_path(final_path,"brrt_final_path");
                visual_ptr->visualize_pointcloud(final_path,"brrt_final_wpts");
                vector<std::pair<double, double>>slns = brrt_ptr_->getSolutions();
                ROS_INFO_STREAM("[BRRT] final path len: " << slns.back().first);
            }
            else
            {
                ROS_INFO("BRRT Plan Failed");
            }
        }

        if(run_brrt_star_)
        {
            
            //路径搜索
            bool brrt_star_result = brrt_star_ptr_->plan(start_, goal_);
            if (brrt_star_result)
            {
                vector<vector<Eigen::Vector3d>> routes = brrt_star_ptr_->getAllPaths();
                visual_ptr->visualize_path_list(routes, "brrt_star_paths", visualization::blue);
                vector<Eigen::Vector3d> final_path = brrt_star_ptr_->getPath();
                visual_ptr->visualize_path(final_path,"brrt_star_final_path");
                visual_ptr->visualize_pointcloud(final_path,"brrt_star_final_wpts");
                vector<std::pair<double, double>>slns = brrt_star_ptr_->getSolutions();
                ROS_INFO_STREAM("[BRRT*] final path len: " << slns.back().first);
            }
            else
            {
                ROS_INFO("BRRT_STAR Plan Failed2");
            }
        }

        if(run_astar_)
        {
            
            //路径搜索
            int astar_result = astar_ptr_->search(start_, goal_,false,0);
            //std::cout<<" ### "<<astar_result<<std::endl;
            
            if (astar_result==1)//{ REACH_END = 1, NO_PATH = 2 };
            {
                std::vector<Eigen::Vector3d> final_path = astar_ptr_->getPath();
                //visual_ptr->visualize_path_list(routes, "astar_paths", visualization::blue);
           
                visual_ptr->visualize_path(final_path,"astar_final_path");
                visual_ptr->visualize_pointcloud(final_path,"astar_final_wpts");
                //vector<std::pair<double, double>>slns = astar_ptr_->getSolutions();
               // ROS_INFO_STREAM("[RRT] final path len: " << slns.back().first);
            }
            else
            {
                ROS_ERROR_STREAM("Fail  == "<<astar_result);
            }
        }
        

        
        start_ = goal_;//

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