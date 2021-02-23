// node에 관련된 모든 정보
#include "ros/ros.h"       
 
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Time.h"
ros::Time tic, tac;

 
int main( int argc, char **argv )
{
    //-- node 초기화
    ros::init( argc, argv, "pub" );
   
    //-- 프로세스 핸들러 추가  
    ros::NodeHandle n;
    //-- 마스터한테 토픽 이름과 타입을 전달
    //   message : 토픽 이름
    //   1000    : 버퍼 사이즈
    ros::Publisher time_pub = n.advertise<std_msgs::Time>("tic", 1);

    //-- 데이터 전송 주기 10 Hz
    ros::Rate loop_rate(50);
  
    while( ros::ok() )
    {
        //-- 메세지 변수 선언 및 값 설정
        std_msgs::Time ticPub;
        int ticTemp;
        tic = ros::Time::now();
        ticPub.data = tic;
        //-- 메세지 전송
        time_pub.publish(ticPub);
        ros::spinOnce();
        loop_rate.sleep();
       
    }
   
    return 0;
}


