#include "ros/ros.h"
#include "navigation/nav_srv.h"
#include "navigation/ints.h"
#include "Navcore.h"
#include <json/json.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>



class Nav_server{
    std::vector<std::string> target_type = {"shopping", "cooking", "serving"};
    std::vector<targetPose> target_pose;
    NavCore* navCore;
    
    bool new_goal{true};
    void readPoseFromJson(std::string);
    bool doNav(navigation::nav_srv::Request& req,navigation::nav_srv::Response& resp);
    bool doadd(navigation::ints::Request& req,navigation::ints::Response& resp);
    ros::ServiceServer server;
    ros::ServiceServer test_server;
    ros::Subscriber action_result_sub;
    std::string MAP_FRAME , BASE_FOOT_PRINT;
    std::string JSON_FILE;
    void actionResultCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);
    bool isArrival(const geometry_msgs::Pose2D &goal2d);
public:
    Nav_server(ros::NodeHandle &nh_, const std::string& base_foot_print, std::string map_frame, std::string json_file);
    ~Nav_server();
};
void Nav_server::actionResultCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
    
    for (const actionlib_msgs::GoalStatus& status : msg->status_list)
    {
        // ROS_INFO("MoveBase status: %d",status.status);
        if (status.status ==3)
        {
            navCore->moveBaseActionResult_ = NavCore::MoveBaseActionResult::SUCCEEDED;
            ROS_INFO("MoveBase status: Goal succeeded");
        }
    }
    // std::cout<<"resullllllllllllllt<"<<msg.status.status<<std::endl;
    // if (msg.status == 3)
    // {
    //     ROS_INFO("GOAL!!!!!!!!!!reach!!!!!!!");
    // }
}

void Nav_server::readPoseFromJson(std::string type)
{
    // std::cout<<"in nav_server!!"<< type <<std::endl;
	Json::Reader reader;/*用于按照JSON数据格式进行解析*/
	Json::Value root;/*用于保存JSON类型的一段数据*/

	// std::ifstream srcFile("~/ICRA_PUB/src/Mapping/config/target_map.json", std::ios::binary);/*定义一个ifstream流对象，与文件demo.json进行关联*/
    
    std::ifstream srcFile(JSON_FILE, std::ios::binary);
    
    // std::ifstream srcFile("/home/hpf/ICRA_PUB/src/Mapping/config/target_map.json", std::ios::binary);
	if (!srcFile.is_open())
	{
		std::cout << "Fail to open target_map.json" << std::endl;
		return;
	}
    
	if (reader.parse(srcFile, root))
	{
        // target.x = root[type][0].asDouble();
        // target.y = root[type][1].asDouble();
        // target.theta = root[type][2].asDouble();
        for(int i = 0; i < 3; i++){
            if (type == target_type[i])
            {
                targetPose P;
                P.targetAction = targetPose::TargetAction(i);
                P.pose.x = root[type][0].asDouble();
                P.pose.y = root[type][1].asDouble();
                P.pose.theta = root[type][2].asDouble();
                target_pose.push_back(P);
                // ROS_INFO_STREAM("target_pose.th: "<<target_pose[i].targetAction);
                ROS_INFO_STREAM("target_pose.targetAction: "<<target_pose[i].targetAction);
                ROS_INFO_STREAM("target_pose.x: "<<target_pose[i].pose.x);
                ROS_INFO_STREAM("target_pose.y: "<<target_pose[i].pose.y);
                ROS_INFO_STREAM("target_pose.th: "<<target_pose[i].pose.theta);
            }
        }
	}
	srcFile.close();
}

bool Nav_server::doadd(navigation::ints::Request& req,
navigation::ints::Response& resp){
    resp.sum = req.num1 + req.num2;
    return true;
}

bool Nav_server::doNav(navigation::nav_srv::Request& req,
navigation::nav_srv::Response& resp){
    ROS_INFO("<<<<<<<<<<<<<<<<<<<<<<<<<in service call");
    ros::Rate loop_rate(30);
    int type = req.tar_type;


    // if (type == 9){
    //     ROS_INFO("in nav 9");
    //     geometry_msgs::Pose2D pose_ori;
        
    //     pose_ori.x = 0;
    //     pose_ori.y = 0;
    //     navCore->setGoal(pose_ori);
    //     loop_rate.sleep();
    //     resp.nav_flag = true;
    //     return true;
    // }
    // else{
    //     navCore->setGoal(target_pose[type].pose);
    //     ROS_INFO("Set pose to x:%d y:%d th:%d:", target_pose[type].pose.x, target_pose[type].pose.y, target_pose[type].pose.theta);
    // }
    // if (type == 2){
    //     if (chdir("/home/hpf/wukong-robot") != 0) {
    //         std::cerr << "Error changing directory" << std::endl;
    //     }
    //     system("./start.sh");
    // }
    navCore->setGoal(target_pose[type].pose);
    bool flag =false;
    while( ros::ok() ){
        geometry_msgs::Pose2D currentpose;
        currentpose = navCore->getCurrentPose(MAP_FRAME,BASE_FOOT_PRINT);
        // int nav_result = navCore->getMoveBaseActionResult();
        // ROS_INFO("<<<nav_result<<<%d ",nav_result);
        // if (nav_result == 3){
        //     ROS_INFO(" $$$$$$$$$$$$$$$$ movebase_result get success");
        // }
        flag = isArrival(target_pose[type].pose);
        if (flag) {
            resp.nav_flag = true;
            resp.tar_string = target_type[req.tar_type];
            break;
        }
        // if (nav_result == NavCore::SUCCEEDED){
        //     ROS_INFO("getting TARGET point SUCCESSFUL!!!");

        //    resp.nav_flag = true;
        //    resp.tar_string = target_type[req.tar_type];
        //     break;
        // }
        loop_rate.sleep();
        //todo: else if (failed to get target, ...)
    }
    return true;
}

Nav_server::Nav_server(ros::NodeHandle &nh_, const std::string& base_foot_link, std::string map_frame, std::string json_file){
    
    // target_type.push_back("shopping");
    // target_type.push_back("cooking");
    // target_type.push_back("serving");
    // std::cout<<"hi!!"<< target_type[0] <<std::endl;
    // Nav nav(base_foot_print, map_frame, serial_addr, file_name);
    JSON_FILE = json_file;
    MAP_FRAME = map_frame;
    BASE_FOOT_PRINT =base_foot_link;
    for(int i = 0; i < 3; i++){
        readPoseFromJson(target_type[i]);
    }
    navCore = new NavCore(base_foot_link, std::move(map_frame));
    server = nh_.advertiseService("navi_service",&Nav_server::doNav,this);
    test_server = nh_.advertiseService("add",&Nav_server::doadd,this);
    action_result_sub = nh_.subscribe("/move_base/status", 10, &Nav_server::actionResultCallback,this);
}

Nav_server::~Nav_server(){
    delete navCore;
}


bool Nav_server::isArrival(const geometry_msgs::Pose2D &goal2d){
    // navCore->getMoveBaseActionResult();
    // // navCore->action_result_sub
    // // if(navCore->)
    // ROS_INFO("MoveBase_result: %d ", );

    float thx = 0.1; float tht = 0.05;
    geometry_msgs::Pose2D currentpose;
    currentpose = navCore->getCurrentPose(MAP_FRAME,BASE_FOOT_PRINT);
    // std::cout << "current pose: " << currentpose.x << currentpose.y <<currentpose.theta<<std::endl;
    // std::cout << "goal2d pose: " << goal2d.x << goal2d.y <<goal2d.theta<<std::endl;
    float dx = std::abs(currentpose.x - goal2d.x);
    float dy = std::abs(currentpose.y - goal2d.y);
    float dth = std::abs(currentpose.theta - goal2d.theta);
    // std::cout << " pose error: " << dx << dy <<dth<<std::endl;
    if (dx+dy < thx &&  dth < tht){
        ROS_INFO("Goal is Arrival!!!");
        navCore->cancelAllGoals();
        return true;
    }
    else return false;
    
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"nav_server");
    ros::NodeHandle nh;
    std::string base_foot_print, odom_frame, map_frame, serial_addr, file_name, json_file;
    nh.param("base_foot_print",base_foot_print,(std::string)"base_link");
    nh.param("map_frame",map_frame,(std::string)"3dmap");
    nh.param("json_file",json_file,(std::string)"/home111/hpf/ICRA_PUB/src/Mapping/config/target_map.json");
   
    Nav_server nav_server(nh, base_foot_print, map_frame, json_file);
    ROS_INFO("导航服务已经启动....");
    ros::spin();
}