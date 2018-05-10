/**
 *
 */
#include <chrono>

#ifndef LATENCY_TEST_TALKER_H_
#define LATENCY_TEST_TALKER_H_


class LatencyTestSubscriber
{
    public:
        LatencyTestSubscriber();
        virtual ~LatencyTestSubscriber();

        /*variable declaration;*/
        ros::NodeHandle n;
        
        ros::Subscriber lat_sub;
        ros::Publisher  lat_pub;
        
        int sub_index_;
        void laytencySubCallback(const std_msgs::UInt8MultiArray::Ptr& msg);
        void init(uint32_t unreliable);
};


#endif
