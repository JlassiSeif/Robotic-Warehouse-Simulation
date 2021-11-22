#include <unordered_map>
#include <cstdlib>
#include <sstream>
#include <math.h>
#include <string.h>
#include <vector>
#include <ctime>
#include <array>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"

#include "robot_path_planning/init_final.h"
#include "point.h"


class pal{
    public :
        std::array<std::string,6> names={"rack1","rack2","rack3","rack4","rack5","rack6"};
        ros::Publisher pos_pub;
        ros::Subscriber pal_sub;
        ros::Subscriber state_sub;
        ros::NodeHandle n;
        
        robot_path_planning::init_final for_path;

        point rack;
        point robot;
        point support;
        std::string available_robot;
        std::string available_support;

        std::unordered_map<std::string , point> racks;
        std::unordered_map<std::string , point> supports_coord;
        std::unordered_map<std::string , std::string> supports_state_left;
        std::unordered_map<std::string , std::string> supports_state_right;
        std::unordered_map<std::string , point> robots_initial;
        std::unordered_map<std::string , std::string> robots_left_state;
        std::unordered_map<std::string , std::string> robots_right_state;

        pal();
        void init_racks();
        void init_supports_coord();
        void init_robots_coord();
        void spawn_random();
        std::string find_available_support(int);
        std::string find_available_robot(point);
        void rack_random_select();
        point select_available_support(point);
        void select_and_spawn();
        void publish_coords(std::string);
        void select_all();
        void init_robots_state();
        void stateCallback(const std_msgs::String::ConstPtr&);
};

pal::pal(){
    init_racks();
    init_supports_coord();
    init_robots_coord();
    init_robots_state();
    this->pos_pub=this->n.advertise<robot_path_planning::init_final>("/pos", 10);
    this->state_sub = this->n.subscribe("/state", 10 , &pal::stateCallback, this);
};
void pal::stateCallback(const std_msgs::String::ConstPtr& msg){
    std::string name=msg->data;
    for(auto& i : robots_left_state){
        if(i.first == name){
            i.second="available";
        }
    }
    for(auto& i : robots_right_state){
        if(i.first == name){
            i.second="available";
        }
    }
};
void pal::init_robots_state(){
    this->robots_left_state["robot1"]="available";
    this->robots_left_state["robot2"]="available";
    this->robots_right_state["robot3"]="available";
    this->robots_right_state["robot4"]="available";
};

void pal::init_robots_coord(){
    this->robots_initial.insert(std::make_pair("robot2",point(-2,0,0)));
    this->robots_initial.insert(std::make_pair("robot1",point(-2,1,0)));

    this->robots_initial.insert(std::make_pair("robot3",point(2,0,0)));
    this->robots_initial.insert(std::make_pair("robot4",point(2,1,0)));
};

void pal::init_racks(){
    this->racks.insert(std::make_pair("rack1",point(-2,2,0.35,1.57)));
    this->racks.insert(std::make_pair("rack2",point(2,2,0.35,1.57)));
    this->racks.insert(std::make_pair("rack3",point(-2,4,0.35,1.57)));
    this->racks.insert(std::make_pair("rack4",point(2,4,0.35,1.57)));
    this->racks.insert(std::make_pair("rack5",point(-2,6,0.35,1.57)));
    this->racks.insert(std::make_pair("rack6",point(2,6,0.35,1.57)));
};

void pal::init_supports_coord(){
    std::stringstream ss;
    for(int i = 1;i<13;++i){
        ss<<i;
        std::string o ="support"+ss.str();
        this->supports_state_left[o]= "available";
        ss.str("");
    };
    for(int i = 13;i<25;++i){
        ss<<i;
        std::string o ="support"+ss.str();
        this->supports_state_right[o]= "available";
        ss.str("");
    };
    this->supports_coord.insert(std::make_pair("support1",point(-2,7,0)));
    this->supports_coord.insert(std::make_pair("support2",point(-3,7,0)));
    this->supports_coord.insert(std::make_pair("support3",point(-4,7,0)));
    this->supports_coord.insert(std::make_pair("support4",point(-5,7,0)));

    this->supports_coord.insert(std::make_pair("support5",point(-2,9,0)));
    this->supports_coord.insert(std::make_pair("support6",point(-3,9,0)));
    this->supports_coord.insert(std::make_pair("support7",point(-4,9,0)));
    this->supports_coord.insert(std::make_pair("support8",point(-5,9,0)));

    this->supports_coord.insert(std::make_pair("support9",point(-2,11,0)));
    this->supports_coord.insert(std::make_pair("support10",point(-3,11,0)));
    this->supports_coord.insert(std::make_pair("support11",point(-4,11,0)));
    this->supports_coord.insert(std::make_pair("support12",point(-5,11,0)));

    this->supports_coord.insert(std::make_pair("support13",point(2,7,0)));
    this->supports_coord.insert(std::make_pair("support14",point(3,7,0)));
    this->supports_coord.insert(std::make_pair("support15",point(4,7,0)));
    this->supports_coord.insert(std::make_pair("support16",point(5,7,0)));

    this->supports_coord.insert(std::make_pair("support17",point(2,9,0)));
    this->supports_coord.insert(std::make_pair("support18",point(3,9,0)));
    this->supports_coord.insert(std::make_pair("support19",point(4,9,0)));
    this->supports_coord.insert(std::make_pair("support20",point(5,9,0)));

    this->supports_coord.insert(std::make_pair("support21",point(5,11,0)));
    this->supports_coord.insert(std::make_pair("support22",point(3,11,0)));
    this->supports_coord.insert(std::make_pair("support23",point(4,11,0)));
    this->supports_coord.insert(std::make_pair("support24",point(5,11,0)));
};

std::string int2string(double a){
    std::stringstream ss;
    ss<<(a);
    return(ss.str());
};

void pal::rack_random_select(){
    int roll =  rand() % 6;
    this->rack=this->racks[this->names[roll]];
};

std::string random_name(){
    return int2string(rand() % 1985674);
};

void spawn(point rack){
    std::string x=int2string(rack.x*2.5);
    std::string y=int2string(rack.y*2.5);
    std::string z=int2string(rack.z);
    std::string roll_name= random_name();
    std::string pos=" -x "+x+" -y "+y+" -z "+z+" -R 0 -P 0 -Y 1.57";
    std::string cmd = "rosrun gazebo_ros spawn_model -file /home/seif/Documents/pfa/src/gazebo_world/model/palete2/palette.sdf -sdf -model ";
    cmd=cmd+roll_name+pos;
    //system(cmd.c_str());
};

void pal::select_all(){
    rack_random_select();
    spawn(this->rack);
    if(this->rack.x<0){
        for(auto& i: this->robots_left_state){
            if(i.second =="available"){
                this->available_robot=i.first;
                i.second="working";
                break;
            }
            else{
                this->available_robot="";
            }
        };
        for(auto& i : this->supports_state_left){
            if(i.second=="available"){
                this->available_support=i.first;
                i.second="not available";
                break;
            }
            else {
                this->available_support="";
            }
        };
        this->rack.x+=1;
    }
    else if (this->rack.x>0){
        for(auto& i: this->robots_right_state){
            if(i.second =="available"){
                this->available_robot=i.first;
                i.second="working";
                break;
            }        
            else {
                this->available_robot="";
            }
        }

        this->robot=this->robots_initial[this->available_robot];
        for(auto& i : this->supports_state_right){
            if(i.second=="available"){
                this->available_support=i.first;
                i.second="not available";
                break;
            }
            else {
                this->available_support="";
            }
        }; 
        this->rack.x-=1;
    };
    this->robot=this->robots_initial[this->available_robot];
    this->support=this->supports_coord[this->available_support];
};

void pal::publish_coords(std::string name_){
    this->for_path.x= this->rack.x+7;
    this->for_path.y= 14-this->rack.y;
    this->for_path.a=this->support.x+7;
    this->for_path.b=14-this->support.y;
    this->for_path.l=this->robot.x+7;
    this->for_path.m=14-this->robot.y;
    this->for_path.name=name_;
    this->pos_pub.publish(this->for_path);
};

void pal::spawn_random(){
    select_all();
    if(!(this->available_robot=="")){
    publish_coords(this->available_robot);
    }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "spawnpal");
    pal k=pal();
    ros::Rate loop_rate(1);
    srand ( time(NULL) );

    char a='0';

    while(ros::ok()){
        a=getchar();
        if(a=='1'){
            k.spawn_random();}
        ros::spinOnce();
    };

    return 0;
};
