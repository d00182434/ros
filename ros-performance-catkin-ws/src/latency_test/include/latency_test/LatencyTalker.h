/**
 *
 */
#ifndef LATENCY_TEST_TALKER_H_
#define LATENCY_TEST_TALKER_H_

#include <mutex>
#include <condition_variable>
#include <chrono>

class TimeStats {
    public:
        TimeStats(){}
        ~TimeStats(){}  
        std::chrono::steady_clock::time_point t_start_;
};
class LatencyTestPublisher
{
    public:
        LatencyTestPublisher();
        virtual ~LatencyTestPublisher();

        /*variable declaration;*/
        ros::NodeHandle n;

        std::mutex mutex_;
        
        ros::Publisher lat_pub;
        ros::Subscriber lat_sub;
        
        
        uint64_t n_bandwidth_;
        uint32_t n_subscribers_;
        uint32_t n_rcv_cnt_;
        uint32_t n_snd_samples_;
        uint32_t data_size_;
        uint32_t test_stopped_;

        /*time statistic*/
        std::vector<TimeStats> time_stat;
        std::vector<std::chrono::duration<double, std::micro>> times_;
        std::chrono::duration<double, std::micro> m_min, m_max, overlay;
        double average, variance;


        /*function declaration;*/
        void latAckSubCallback(const std_msgs::UInt8MultiArray::Ptr& msg);
        void init(uint32_t subscribers, uint32_t samples, uint32_t data_size, uint32_t unreliable);
        void test();
        void AnalysisAndReport();
};


#endif
