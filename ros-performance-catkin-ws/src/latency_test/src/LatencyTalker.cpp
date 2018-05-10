#include <stdio.h>
#include <string>
#include <iostream>
#include <iomanip>
#include <bitset>
#include <cstdint>
#include <numeric>
#include <cmath>
#include <fstream>
#include <inttypes.h>
#include <thread>
#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "latency_test/LatencyTalker.h"
#include "latency_test/MessageTypes.h"
#include "latency_test/optionparser.h"

using namespace std;

/**
 *
 */
struct Arg: public option::Arg{
    static void printError(const char* msg1, const option::Option& opt, const char* msg2){
        fprintf(stderr, "%s", msg1);
        fwrite(opt.name, opt.namelen, 1, stderr);
        fprintf(stderr, "%s", msg2);
    }

    static option::ArgStatus Unknown(const option::Option& option, bool msg){
        if (msg) printError("Unknown option '", option, "'\n");
        return option::ARG_ILLEGAL;
    }

    static option::ArgStatus Required(const option::Option& option, bool msg){
        if (option.arg != 0 && option.arg[0] != 0)
        return option::ARG_OK;

        if (msg) printError("Option '", option, "' requires an argument\n");
        return option::ARG_ILLEGAL;
    }

    static option::ArgStatus Numeric(const option::Option& option, bool msg){
        char* endptr = 0;
        if (option.arg != 0 && strtol(option.arg, &endptr, 10)){};
        if (endptr != option.arg && *endptr == 0)
        return option::ARG_OK;

        if (msg) printError("Option '", option, "' requires a numeric argument\n");
        return option::ARG_ILLEGAL;
    }
};

/**
 *
 */
enum optionIndex {
    UNKNOWN_OPT,
    HELP,
    SAMPLES,
    MSGLEN,
    SUBSCIRBERS,
    UNRELIABLE,
    BANDWIDTH,
};

/**
 *
 */
const option::Descriptor usage[] = {
    { UNKNOWN_OPT,  0,  "",     "",            Arg::None,       "Usage: ThroughputTest <publisher|subscriber>\n\nGeneral options:" },
    { HELP,         0,  "h",    "help",        Arg::None,       "  -h \t--help  \tProduce help message." },
    { UNRELIABLE,   0,  "u",    "unreliable",  Arg::None,       "  -u \t--unreliable  \tTransport in unreliable mode, default is reliable." },
    { SAMPLES,      0,  "s",    "samples",     Arg::Numeric,    "  -s <num>, \t--samples=<num>  \tNumber of samples." },
    { MSGLEN,       0,  "l",    "msglen",      Arg::Required,   "  -l <bytes>, \t--meglen=<bytes>  \tMessage data size in bytes." },
    { SUBSCIRBERS,  0,  "n",    "subscribers", Arg::Required,   "  -n <num>, \t--subscribers=<num> \tNumber of subscribers." },
    { BANDWIDTH,    0,  "b",    "bandwidth",   Arg::Numeric,    "  -b <permillage>, \t--bandwidth=<permillage> \tpermillage of 1Gbps."},
    { 0, 0, 0, 0, 0, 0 }
};


/**
 *
 */
LatencyTestPublisher::LatencyTestPublisher()
{
    //cout << "LatencyTestPublisher Constructure." << endl;
    n_subscribers_ = 0;
    n_rcv_cnt_=0;
    n_snd_samples_=0;
    n_bandwidth_ = 0;
    times_.clear();
}


/**
 *
 */
LatencyTestPublisher::~LatencyTestPublisher()
{
}

/**
 *
 */
void LatencyTestPublisher::init(uint32_t subscribers, uint32_t samples, uint32_t data_size, uint32_t unreliable)
{
    n_snd_samples_ = samples;
    n_subscribers_ = subscribers;
    data_size_ = data_size;
    test_stopped_ = 0;
    lat_pub = n.advertise<std_msgs::UInt8MultiArray>("latency", 2000);
    /*here wait for subscriber connection;*/
    ros::Rate loop_rate(1);
    uint32_t num_matched = lat_pub.getNumSubscribers();
    while ( n_subscribers_ !=  num_matched )
    {
        loop_rate.sleep();
        num_matched = lat_pub.getNumSubscribers();

        if ( n_subscribers_ < num_matched )
        {
             printf("EXPECT %u SUBSCRIBERS but MATCHED MORE(%u)\n", n_subscribers_, num_matched);
        }
    }
    cout << "TOPIC latency CONNECTION COMPLETED" << endl;

    if ( !unreliable )
    {
	cout << "TRANSPORT IN RELIABLE MODE" << endl;
        lat_sub = n.subscribe("latency_ack", 2000, &LatencyTestPublisher::latAckSubCallback, this);
    }
    else
    {
	cout << "TRANSPORT IN UNRELIABLE MODE" << endl;
        lat_sub = n.subscribe("latency_ack", 2000, &LatencyTestPublisher::latAckSubCallback, this, ros::TransportHints().unreliable().reliable().tcpNoDelay(true));
    }

    /*here wait for subscriber connection;*/
    while ( n_subscribers_ != lat_sub.getNumPublishers() )
    {
        loop_rate.sleep();
    }

    cout << "TOPIC latency_ack CONNECTION COMPLETED" << endl;
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void LatencyTestPublisher::latAckSubCallback(const std_msgs::UInt8MultiArray::Ptr& msg)
{

    //std::unique_lock<std::mutex> lock(mutex_);

    uint32_t index = *(uint32_t*)&msg->data[0];
    std::chrono::steady_clock::time_point t_end = std::chrono::steady_clock::now();

    /*reocrd the timestamp.*/
    if ( index < n_snd_samples_ )
    {
        times_.push_back(std::chrono::duration<double, std::micro>(t_end - time_stat[index].t_start_ - overlay)/2);
        n_rcv_cnt_++;
    }
    
#if 0
    if ( n_rcv_cnt_ >= (n_snd_samples_ * n_subscribers_) )
    {
        cout << n_rcv_cnt_ << " SAMPLES TRANSPORT SUCCESS FOR " << n_subscribers_ << " SUBSCRIBERS" << endl;

        /*analysis the testing results here*/
        AnalysisAndReport();
    }
#endif
    //lock.unlock()

    if ( index == STOP_CMD )
    {
        test_stopped_ = 1;
    }
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void LatencyTestPublisher::AnalysisAndReport()
{
    
    std::chrono::duration<double, std::micro> duration;

#if 0
    std::vector<std::chrono::duration<double, std::micro>> times_;

    /*link all time duration to vector times_*/
    times_.clear();

    for ( std::vector<TimeStats>::iterator it = time_stat.begin(); it != time_stat.end(); ++it )
    {
        duration = std::chrono::duration<double, std::micro>(it->t_end_ - it->t_start_ - overlay)/2;
        times_.push_back(duration);
        //cout << "Timestamp Details: "<< endl << duration.count() << endl;
    }
#endif

    m_min = *std::min_element(times_.begin(), times_.end());
    m_max = *std::max_element(times_.begin(), times_.end());
    average = std::accumulate(times_.begin(), times_.end(), std::chrono::duration<double, std::micro>(0)).count() / times_.size();

    double auxstdev=0;
    for(std::vector<std::chrono::duration<double, std::micro>>::iterator it=times_.begin(); it != times_.end(); ++it)
    {
        auxstdev += pow(((*it).count() - average), 2);
    }
    auxstdev = sqrt(auxstdev / times_.size());
    variance = static_cast<double>(round(auxstdev));

    printf("Printing round-trip times in us, statistics for %d samples\n", n_snd_samples_);
    printf("      Bytes,    Samples,      stdev,       mean,        min,        max\n");
    printf("-----------,-----------,-----------,-----------,-----------,-----------\n");
    printf("%11u,%11u,%11.2f,%11.2f,%11.2f,%11.2f \n",
            data_size_, n_snd_samples_, variance, average,m_min.count(), m_max.count());
}


/**
 *
 */
void LatencyTestPublisher::test()
{
    TimeStats TS;

    /**
     * Construction for message data;
     */
    std_msgs::UInt8MultiArray msg;
    msg.data.resize(data_size_);
    for ( int i=0; i < data_size_; i++)
    {
        msg.data[i] = (i%255);
    }

    /**
     * Caculate the overlay for timestamp's push operation.
     */
    time_stat.clear();
    std::chrono::steady_clock::time_point s, e;
    s = std::chrono::steady_clock::now();
    for ( int i =0; i < 1000; i++ )
    {
         time_stat.push_back(TS);
        e = std::chrono::steady_clock::now();
    }
    overlay = std::chrono::duration<double, std::micro>(e-s)/1001;

    /*clear time_stat for testing.*/      
    time_stat.clear();

    /*caculate the loop rate depend on the message size*/
    uint64_t rate = 10000;
    uint64_t time_ms = 0;
    uint64_t throughput_allow =0;

    if ( data_size_ > 512 )
    {
        throughput_allow = 800000000;
    }
    else if ( data_size_ >= 32 )
    {
        throughput_allow = 100000000;
    }
    else
    {
        throughput_allow = 10000000;
    }

    throughput_allow = (n_bandwidth_ ? n_bandwidth_ : throughput_allow) ;

    rate = throughput_allow/(n_subscribers_*data_size_*8);
    rate = (rate > 1 ? rate : 1);
    time_ms = (n_snd_samples_*1000)/rate;
     
    //printf("%u SAMPLES OF %u BYTES LENGTH IN ABOUT %" PRIu64 " MS, RATE is %" PRIu64 "\n", n_snd_samples_, data_size_, time_ms, rate);
    printf("DATA SIZE IN BYTES: %u\n", data_size_);
    printf("SEND SAMPLES: %u\n", n_snd_samples_);
    printf("MEASUER THROUGHPUT IN Mbps: %" PRIu64 "\n", throughput_allow/1000000);
    printf("ESTIMATE TIME IN MS : %" PRIu64 ", (rate = %" PRIu64 ")\n", time_ms, rate);

    ros::Rate loop_rate(rate);
    /*run the testing loops*/
    for ( int i =0; i< n_snd_samples_; i++ )
    {
        TS.t_start_ = std::chrono::steady_clock::now();
        time_stat.push_back(TS);
        /*the next expression was used for debug purpose.*/
        //latencyPub.t1 = std::chrono::steady_clock::now();

        /**
        * The publish() function is how you send messages. The parameter
        * is the message object. The type of this object must agree with the type
        * given as a template parameter to the advertise<>() call, as was done
        * in the constructor above.
        */
        *(uint32_t*)&msg.data[0] = i;
        msg.data[4] = (i%n_subscribers_);
        lat_pub.publish(msg);
        loop_rate.sleep();
    }

    /*wait for stop*/
    while(!test_stopped_)
    {
        ros::Rate tmp(10);
        *(uint32_t*)&msg.data[0] = STOP_CMD;
        lat_pub.publish(msg);
        tmp.sleep();
    }

    cout << n_rcv_cnt_ << " SAMPLES TRANSPORT SUCCESS FOR " << n_subscribers_ << " SUBSCRIBERS" << endl;
    AnalysisAndReport(); 
}

/**
 *
 */
static int msg_rcv_thread( void )
{
    cout << "MESSAGE RECIEVE THREAD CREATED" << endl;
    ros::spin();
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    /**
     * Parser the arguments.
     */
    int columns;
    columns = getenv("COLUMNS")?atoi(getenv("COLUMNS")):80;
    uint32_t samples = 0 ;
    uint32_t data_size = 0;
    uint32_t subscribers = 1;
    uint32_t unreliable = 0;
    uint32_t premillage=0;

    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "talker");
    
    /*skip program name argv[0] if present*/
    argc -= (argc > 0);
    argv += (argc > 0);
  
    option::Stats stats(usage, argc, argv);
    std::vector<option::Option> options(stats.options_max);
    std::vector<option::Option> buffer(stats.buffer_max);
    option::Parser parse(usage, argc, argv, &options[0], &buffer[0]);
    if (parse.error())
    {
        cout << "Parse arguments error!" << endl;
        return 1;
    }

    if (options[HELP])
    {
        option::printUsage(fwrite, stdout, usage, columns);
        return 0;
    }

    for ( int i=0; i < parse.optionsCount(); ++i)
    {
        option::Option& opt = buffer[i];
        switch (opt.index())
        {
            case HELP:
                /*Never get here, it was already handled above.*/
                break;
            case UNRELIABLE:
                unreliable = 1;
                break;
            case SAMPLES:
                samples = strtol(opt.arg, nullptr, 10);
                //cout << "samples: " << samples << endl;
                break;
            case MSGLEN:
                data_size = strtol(opt.arg, nullptr, 10);
                //cout << "data_size: " << data_size << endl;
                break;
            case SUBSCIRBERS:
                subscribers = strtol(opt.arg, nullptr, 10);
                //cout << "subscribers: " << subscribers << endl;
                break;
            case BANDWIDTH: 
                premillage = strtol(opt.arg, nullptr, 10);
                break;
            case UNKNOWN_OPT:
                option::printUsage(fwrite, stdout, usage, columns);
                break;
        }
    }
    
    /**
     * Reciever, create a thread for ros::spin();
     */
    std::thread rcvthrd(msg_rcv_thread);
  
    /**
     * Transaction;
     */
    LatencyTestPublisher latencyPub;
    latencyPub.n_bandwidth_ = premillage*1000000ull; //bps = premillage x 1Gbps;
    latencyPub.init(subscribers, samples, data_size, unreliable);
    latencyPub.test();
    exit(0);
}
