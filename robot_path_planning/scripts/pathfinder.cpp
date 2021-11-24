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
#include "PathFinder.h"
class finder{
    public :
        ros::Subscriber pos_sub;
        ros::Publisher path_pub;
        ros::NodeHandle n;
        std_msgs::String path_to_send;
        std::unordered_map<std::string , ros::Publisher> publishers;
        PathFinder obj=PathFinder();
        finder();
        void posCallback(const robot_path_planning::init_final::ConstPtr&);
        void init_publishers();
};

finder::finder(){
    init_publishers();
    this->pos_sub=this->n.subscribe("/pos", 10 , &finder::posCallback, this);
    this->path_pub=this->n.advertise<std_msgs::String>("/instr", 10);
};

void finder::init_publishers(){
    std::stringstream ss;
        for(int i = 1;i<5;++i){
            ss<<i;
            std::string o ="robot"+ss.str();
            this->publishers.insert(std::make_pair(o,n.advertise<std_msgs::String>("/"+o+"/instr", 10)));
            ss.str("");
    };
}
int string2int(std::string ch){
	std::stringstream ss;
	int a;
    ss<<ch;
	ss>>a;
    return(a);
};

std::string int2string(double a){
    std::stringstream ss;
    ss<<(a);
    return(ss.str());
};

std::vector<int>decode(std::string seq){
    std::vector < int > tab;
    std::string a;
    int i=0;
	int flag=0;
	std::string ch="";
	while(i<seq.size()){
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
			ch=ch+seq[i];
		}

		i++;
	}
    return(tab);
};
std::vector<int> compress(std::vector<int> tab){
    std::vector<int> a;
    int j=0,n=0,i=0;
    while(i<tab.size()){

    		i+=j;
    		j=0;

    	    a.push_back(tab[i]);
			a.push_back(tab[i+1]);

			if((tab[i]==tab[i+j+2]) && ( (i+j+2)<tab.size() )){
            	while((tab[i]==tab[i+j+2]) && ( (i+j+2) < tab.size() ) ){
                	j+=2;
            	}
            }
			else if( (tab[i+1]==tab[i+j+3]) && ( (i+j+3) < tab.size() ) ){
           		 while((tab[i+1]==tab[i+j+3]&& ( (i+j+3) < tab.size() )) ){
               		 j+=2;
            	}
			}
			else {
				i+=2;
				}
        }
    return a;
}

std::string encode(std::vector<int> k){
    std::string a="";
    for(int i=0;i<k.size();i+=2){
        if(k[i]==-100){
            a+="p/";
            i+=2;
        }
        else if(k[i]==-200){
            a+="d/";
            i+=2;
        }
        else if(k[i]==100){
            a+="u/";
            i+=2;
        }
        else if(k[i]==200){
            a+="l/";
            i+=2;
        }
        else {
        a+=int2string(k[i])+","+int2string(k[i+1])+"/";
        }
    }
    return a;
}

std::string work(std::string seq){
	std::vector < int > tab;
    std::vector < int > tab1;

    std::string a;
	tab=decode(seq);
    tab1=compress(tab);
    a=encode(tab1);
    return a;
    
}
void finder::posCallback(const robot_path_planning::init_final::ConstPtr& msg){
    std::string name=msg->name;
    std::cout<<name<<std::endl;
    std::cout<<msg->m<<"|"<<msg->l<<"|"<<msg->y<<"|"<<msg->x<<"|"<<msg->b<<"|"<<msg->a<<"|"<<std::endl;
    std::string firstpart = obj.findPath(msg->m, msg->l, msg->y, msg->x, 1);
    std::string secondpart = obj.findPath(msg->y, msg->x, msg->b, msg->a, 1);
    std::string all;
    std::string thirdpart = obj.findPath(msg->b, msg->a, msg->m, msg->l, 0);
    if((name=="robot4" )|| (name=="robot3")){
        all= firstpart+"p/"+secondpart+"d/"+thirdpart;
    }
    else {
        all= firstpart+"u/"+secondpart+"l/"+thirdpart;
    }
    all=work(all);
    this->path_to_send.data=all;
    if(name!=""){
        for(auto& i : this->publishers){
            if(i.first==name){
                i.second.publish(this->path_to_send);
            }
        }
    }

}

int main(int argc, char **argv){
 
    ros::init(argc, argv, "pathfinder");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);
	ros::spinOnce();
    
    finder fin=finder();
    while(ros::ok()){
        ros::spinOnce();
    };
    return 0;
};
