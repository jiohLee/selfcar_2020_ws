// node에 관련된 모든 정보
#include "ros/ros.h"       
 
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Time.h"
ros::Time tic, tac;
void timeCallback( const std_msgs::Time::ConstPtr& msg )
{
    tac = ros::Time::now();
    ros::Duration cost_time = tac-msg->data;
    
    std::cout << "cost : " << cost_time << "\n";

    //-- print
}
 
int main( int argc, char **argv )
{
    //-- node 초기화
    ros::init( argc, argv, "sub" );
   
    //-- 프로세스 핸들러 추가  
    ros::NodeHandle n;
    //-- 마스터한테 토픽 이름과 타입을 전달
    //   message : 토픽 이름
    //   1000    : 버퍼 사이즈
    ros::Subscriber sub = n.subscribe("tic", 1, timeCallback);

    //-- 데이터 전송 주기 10 Hz
    
    ros::spin();
    return 0;
}


