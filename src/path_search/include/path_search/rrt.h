
#ifndef RRT_H
#define RRT_H

//rrt算法需要
#include "ros/ros.h"
//地图需要
#include "occ_grid/occ_map.h"
namespace path_search
{
    class RRT
    {
        private:
        ros::NodeHandle nh_;//用于定义话题
        env::OccMap::Ptr map_ptr_;//用于地图操作

        public:
        RRT(){};
        
        RRT(const ros::NodeHandle &nh, const env::OccMap::Ptr &mapptr):nh_(nh), map_ptr_(mapptr)
        {


        };

        ~RRT(){};

    };
}

#endif