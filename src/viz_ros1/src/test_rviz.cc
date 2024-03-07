#include "vis_ros1.hpp"
#include<tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
namespace vis
{

class testDisplayRviz
{
private:
    ros::NodeHandle nh_;    
    tf2_ros::TransformBroadcaster tfb_;
    geometry_msgs::TransformStamped transform_stamp_;

    std::shared_ptr<displayRviz> vis_ptr_;

public:
    testDisplayRviz(const ros::NodeHandle &nh) : nh_(nh)
    {
        vis_ptr_=std::make_shared<displayRviz>(nh_);
        vis_ptr_->enroll<nav_msgs::Path>("global_path");
        vis_ptr_->enroll<sensor_msgs::PointCloud2>("global_point_cloud");
        vis_ptr_->enroll<vMarkers>("global_marker");
        vis_ptr_->enroll<vMarkers>("robot_path");
    }
    ~testDisplayRviz(){};
    int run(){
        
        Vec3ds global_path,global;
        Vec3d global_path_point;
        for(int i=1;i<8;i++){
            global_path_point[0]=i;
            global_path_point[1]=i;
            global_path_point[2]=0;
            global_path.emplace_back(global_path_point);
        }
        std::vector<Vec3ds> globals;
        for(int j=-1;j<2;j++)
        {
            for(int i=0;i<8;i++){
                global_path_point[0]=i*j;
                global_path_point[1]=i;
                global_path_point[2]=0;
                global.emplace_back(global_path_point);
            }          
            globals.emplace_back(global);  
        }

       
        transform_stamp_.header.frame_id="map";
        transform_stamp_.child_frame_id="test";
        transform_stamp_.transform.translation.x=0.0;
        transform_stamp_.transform.translation.y=0.0;
        transform_stamp_.transform.translation.z=0.0;
        tf2::Quaternion q;
        q.setRPY(0,0,0);
        transform_stamp_.transform.rotation.x=q.x();
        transform_stamp_.transform.rotation.y=q.y();
        transform_stamp_.transform.rotation.z=q.z();
        transform_stamp_.transform.rotation.w=q.w();
        ros::Rate rate(10);
        ROS_INFO_STREAM("globals size = "<<globals.size());
        while(ros::ok()){
            transform_stamp_.header.stamp=ros::Time::now();
            tfb_.sendTransform(transform_stamp_);
               
            // vis_ptr_->vis_pointcloud(global_path,"global_pc","test");
            // vis_ptr_->vis_path(global_path,"global_path");
            // vis_ptr_->vis_path_point<Vec3ds,std::string>(global_path,"global_path");
            // vis_ptr_->vis_path_strip(globals,"global_marker","map",0.02,vis::Color::pink);
            vis_ptr_->vis_path_robot<Vec3ds,std::string>(global_path,"robot_path",0.5,0.15,0.1,0.1);
            ros::spinOnce();
            rate.sleep();
        }
        return 0;
    }
};


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_path_finder_node");
    ros::NodeHandle nh("~");
    vis::testDisplayRviz test_display_rviz(nh);
    test_display_rviz.run();

    // ros::AsyncSpinner spinner(0);
    // spinner.start();
    // ros::waitForShutdown();
    return 0;
}






































