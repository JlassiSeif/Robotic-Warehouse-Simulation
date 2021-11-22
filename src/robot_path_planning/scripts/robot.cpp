#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"

#include <math.h>
#include "tf/tf.h"
#include <time.h>
#include <string.h>
#include <vector>
#include <sstream>

class Robot
{
	private:
		
		float yaw , odom_x , odom_y, x, y;
		ros::Subscriber odom_sub;
		ros::Subscriber cmd_sub;
		ros::Subscriber instr_sub;
		ros::Publisher cmd_pub;
		ros::Publisher pos_pub ;
		ros::NodeHandle n;
		ros::Publisher state_pub ;

	public:
		std::string instruction;
		std_msgs::String name_;
		ros::Publisher F_Grip_pub;
		ros::Publisher piston_pub;
		geometry_msgs::Twist command;
		void odomCallback(const nav_msgs::Odometry::ConstPtr& msg );

		void pickup();
		void letdown();
		void emchi_x(double x);
		void emchi_y(double y);
		void dour(double angle);
		void rotate(double target_angle);
		void move(double target_distance,double orientation);
		void instrCallback(const std_msgs::String::ConstPtr& msg);
		void forward(double distance);
		void position();
		void Forward_Backward(double x,double y);
		void gotoxy(std::string seq);
		float getter_odom_x(){
			return this->odom_x;
		};
		void setter_odom_x(float a){
			this->odom_x=a;
		};
		void setter_instruction(float a){
			this->instruction=a;
		}
		float getter_odom_y(){
			return this->odom_y;
		};
		void setter_odom_y(float a){
			this->odom_y=a;
		}
		Robot(std::string name){
			this->name_.data = name ;
			this->odom_sub=n.subscribe("/"+name+"/odom", 10 , & Robot::odomCallback, this);
			this->instr_sub=n.subscribe("/"+name+"/instr", 10 , &Robot::instrCallback, this);
			this->cmd_pub=n.advertise<geometry_msgs::Twist>("/"+name+"/cmd_vel", 10);
			this->F_Grip_pub=n.advertise<std_msgs::Float64>("/"+name+"/F_Grip_position_controller/command", 10);
			this->piston_pub=n.advertise<std_msgs::Float64>("/"+name+"/piston_position_controller/command", 10);
			this->state_pub=n.advertise<std_msgs::String> ("/state",10);
		};
};

double degree_to_rad(double angle_deg);
double rad_to_degree(double angle_rad);
float discrete(double x);

void Robot::instrCallback(const std_msgs::String::ConstPtr& msg){
	this->instruction=(char*) msg->data.c_str();
	gotoxy(this->instruction);
	this->state_pub.publish(this->name_);
}


int string2int(std::string ch){
	std::stringstream ss;
	int a;
    ss<<ch;
	ss>>a;
    return(a);
};

void Robot::gotoxy(std::string seq){
	std::vector < int > tab;

	int i=0;
	int flag=0;
	std::string ch="";

	while(i<seq.size()){
		std::cout << "i= "<< i << " seq[i]= "<< seq[i] << "\n" ;
		if( seq[i]=='p'){
			tab.push_back(-100);
			tab.push_back(-100);
			ch="";
			flag=1;

		}
		else if (seq[i]=='d'){
			tab.push_back(-200);
			tab.push_back(-200);
			ch="";
			flag=1;

		}
		else if( seq[i]=='u'){
			tab.push_back(100);
			tab.push_back(100);
			ch="";
			flag=1;

		}
		else if (seq[i]=='l'){
			tab.push_back(200);
			tab.push_back(200);
			ch="";
			flag=1;

		}
		else if(seq[i]==','){
			if(flag){flag=0;}
			else{
			tab.push_back(string2int(ch));
			ch="";
			}
		}

		else if(seq[i]=='/'){
			if(flag){flag=0;}
			else {
			tab.push_back(string2int(ch));;
			ch="";
			
			}

		}
		else{
			std::cout<<ch<<std::endl;
			ch=ch+seq[i];
		}

		i++;


	}


	//   std::cout <<seq << "\n" ;
	//    std::cout <<" TAAAAAAAAAAAAAAAB =" <<"\n";

	for(int j=0;j<tab.size();j+=2){
    //std::cout<< tab[j] << " "<< tab[j+1] << "\n" ;
		Forward_Backward(tab[j],tab[j+1]);
		ros::spinOnce();
	}

};

float discrete(double x){
	float q=0 ;
	if(x>0){
		x=x+0.2;
		q= x/2.5 ;
	}
	if(x<0){
		x=x-0.2;
		q= x/2.5 ;
	}

	return q;
};

void Robot::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	this->setter_odom_x(msg->pose.pose.position.x);
	this->setter_odom_y(msg->pose.pose.position.y);
	this->yaw=tf::getYaw(msg->pose.pose.orientation);
	this->x=(int)discrete(this->odom_x) ;
	this->y=(int)discrete(this->odom_y) ;
};



void Robot::position(){

	std::cout << "x= " << this->odom_x << "\n" ;
	std::cout << "y= " << this->odom_y << "\n" ;
	std::cout << "angle= " << rad_to_degree(this->yaw) << "\n" ;
	std::cout << "( " << this->x <<" , "<< this->y <<" )"  << "\n" ;


}
void Robot::forward(double target_distance){
	double init_x=this->odom_x, init_y=this->odom_y,  init_yaw=rad_to_degree(this->yaw);
	double current_x=init_x, current_y=init_y ;
	double current_distance=0;
	double current_angle=init_yaw;
	double distance_error = target_distance-current_distance ;
	double angle_error=0;
	double kp=10 ;
	double kpOr=0.2;
	double speed=0;
	double correction=0;
	int signe=0 ;

	if(target_distance < 0)
		signe=-1;
	else
		signe=1 ;

	while(abs(distance_error)>=0.0005){

		current_x=this->odom_x;
		current_y=this->odom_y ;
		current_angle=rad_to_degree(this->yaw);
		current_distance = sqrt(((current_x-init_x)*(current_x-init_x))+((current_y- init_y)*(current_y- init_y))) * signe ;
    //   std::cout << "current distance= " << current_distance << "\n" ;

		distance_error = target_distance-current_distance ;
		angle_error=init_yaw-current_angle;

      // std::cout << "target distance= " << target_distance << "\n remaining distance= " << distance_error << "\n" ;

		speed=kp * distance_error ;
		correction=kpOr*angle_error;


		if (speed >= 1 ) speed = 1 ;
		if (speed <= -1 ) speed = -1 ;
    //   std::cout <<"calculated speed= "<<speed <<"\n" ;

		if (correction >=0.5 ) correction=0.5 ;
		if (correction <=-0.5 ) correction=-0.5 ;


		this->command.angular.z=correction;
		this->command.linear.x=speed;
		this->cmd_pub.publish(this->command);
		ros::spinOnce();
     //  std::cout << "***************************************" << "\n" ;
	}
    // std::cout << "Stopping .." << "\n" ;
	this->command.angular.z=0;
	this->command.linear.x=0;
	sleep(1);
	this->cmd_pub.publish(this->command);
  //   std::cout << "Stopped " <<"\n " ;
}


void Robot::move(double target_distance,double orientation){

	double init_x=this->odom_x, init_y=this->odom_y,  init_yaw=orientation ;
	double current_x=init_x, current_y=init_y ;
	double current_distance=0;
	double current_angle=rad_to_degree(this->yaw);
	double distance_error = target_distance-current_distance ;
	double angle_error=0;
	double kp=10 ;
	double kpOr=0.2;
	double speed=0;
	double correction=0;
	int signecor=1;
	int signe=0 ;

	if(( current_angle<0)&&(init_yaw==180))
		current_angle=360+current_angle;

	if(target_distance < 0)
		signe=-1;
	else
		signe=1 ;
	std::cout <<" init yaw "<< init_yaw <<"\n";

   /* if (init_yaw==180)
        signecor=-1;*/
	angle_error=init_yaw-current_angle;

	while(abs(distance_error)>=0.0005){

		current_x=this->odom_x;
		current_y=this->odom_y ;
		current_angle=rad_to_degree(this->yaw);
		std::cout <<" current_angle old  "<< current_angle <<"\n";



		if(( current_angle<0)&&(init_yaw==180)) {
			current_angle=360+current_angle;

		}


		std::cout <<" current_angle new  "<< current_angle <<"\n";

		current_distance = sqrt(((current_x-init_x)*(current_x-init_x))+((current_y- init_y)*(current_y- init_y))) * signe ;
		distance_error = target_distance-current_distance ;
		angle_error=init_yaw-current_angle;

		std::cout <<" angle_error   "<< angle_error <<"\n";


      // std::cout << "target distance=  " << target_distance << "\n remaining distance= " << distance_error << "\n" ;

		speed=kp * distance_error ;
		correction=kpOr*angle_error;

		std::cout <<" correction   "<< correction <<"\n";

		if (speed >= 1 ) speed = 1 ;
		if (speed <= -1 ) speed = -1 ;
    //   std::cout <<"calculated speed= "<<speed <<"\n" ;

		if (correction >=2 ) correction=2 ;
		if (correction <=-2 ) correction=-2 ;


		this->command.angular.z=correction;
		this->command.linear.x=speed;
		this->cmd_pub.publish(this->command);
		ros::spinOnce();
     //  std::cout << "***************************************" << "\n" ;
	}
    // std::cout << "Stopping .." << "\n" ;
	this->command.angular.z=0;
	this->command.linear.x=0;
	sleep(1);
	this->cmd_pub.publish(this->command);
  //   std::cout << "Stopped " <<"\n " ;
}

void Robot::Forward_Backward(double x,double y){
	
	if((x==-100)&&(y==-100)){
		rotate(-90);
		forward(2.5);
		pickup();
		forward(-2.5);
		rotate(90);



	}
	else if ((x==-200)&&(y==-200)){
		rotate(90);
		forward(2.5);
		pickup();
		forward(-2.5);
		rotate(-90);

	}
	else if((x==100)&&(y==100)){
		
		rotate(90);
		forward(2.5);
		pickup();
		forward(-2.5);
		rotate(-90);



	}
	else if ((x==200)&&(y==200)){
		rotate(-90);
		forward(2.5);
		pickup();
		forward(-2.5);
		rotate(90);

	}
	else {
		float delta_x=x-this->x;
		float delta_y=y-this->y;
		float delta=0;
		float orientation=0;

		if (delta_x!=0){
			delta=delta_x;
			if(delta>0)
				orientation=-90;
			else
				orientation=90 ;
		}
		else{
			delta=delta_y;
			if (delta>0)
				orientation=0;
			else
				orientation=180;

		}
		float current_x=this->odom_x;
		float current_y=this->odom_y;
		float target_x = x *2.5 ;
		float target_y = y*2.5 ;
		float distance = sqrt(((target_x-current_x)*(target_x-current_x) )+((target_y-current_y)*(target_y-current_y)));
		rotate(orientation);
		move(distance,orientation);
	}

}
void Robot::rotate(double target_angle){
	double error=0;
	double speed=0;
	double kp= 0.15;
	double current_angle=rad_to_degree(this->yaw);

	if(( current_angle<0)&&(target_angle==180))
		current_angle=360+current_angle;


	error=target_angle-current_angle ;
  //  std::cout << "initial error= " << error << "\n" ;

	while(abs(error) >= 0.05 ){

		current_angle=rad_to_degree(this->yaw);
		if(( current_angle<0)&&(target_angle==180))
			current_angle=360+current_angle;

		error=target_angle-current_angle ;
   //   std::cout << "current error= " << (error) << "\n" ;
		speed=kp*error;
    //  std::cout << "calculated speed= " << speed << "\n" ;

		if (speed>= 2) speed=2 ;
		if (speed<= -2) speed=-2 ;

 //     std::cout << "speed= " << speed << "\n" ;
		this->command.angular.z=speed;
		this->cmd_pub.publish(this->command);
		ros::spinOnce();
  //    std::cout << "***************************************" << "\n" ;
	}
  //  std::cout << " stopping.. " <<" \n";
	this->command.angular.z=0;
	sleep(1);
	this->cmd_pub.publish(this->command);
  //  std::cout << "Stopped \n " ;

}



void Robot::pickup(){
	std_msgs::Float64 cmd ;
	cmd.data=2.5;
	this->piston_pub.publish(cmd);
	sleep(1);
	cmd.data=10.0;
	this->F_Grip_pub.publish(cmd);
	sleep(1);


};
void Robot::letdown(){
	std_msgs::Float64 cmd ;
	cmd.data=-1.0;
	this->piston_pub.publish(cmd);
	this->F_Grip_pub.publish(cmd);
	sleep(1);
};
double degree_to_rad(double angle_deg){
	double pi = 3.14159265359;
	return(angle_deg *(pi / 180));
};
double rad_to_degree(double angle_rad){
	double pi = 3.14159265359;
	return(angle_rad *(180 / pi));
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot");

	std::string param;
	ros::NodeHandle nh("~");
	nh.getParam("param", param);



	Robot robot=Robot(param);
	ROS_INFO("chui la");
	sleep(1) ;
	ros::Rate loop_rate(1);
	ros::spinOnce();

	while (ros::ok())

	{
		ros::spinOnce();
	}
	return 0;
}
