#include <stdio.h>
#include <string>
#include <iostream>
#include <iomanip>
#include <bitset>
#include <cstdint>
#include <iostream>
#include <thread>
#include <regex>
#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "latency_test/LatencyListener.h"
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
    UNRELIABLE,    
};

/**
 *
 */
const option::Descriptor usage[] = {
    { UNKNOWN_OPT,  0,  "",     "",            Arg::None,       "Usage: ThroughputTest <publisher|subscriber>\n\nGeneral options:" },
    { HELP,         0,  "h",    "help",        Arg::None,       "  -h \t--help  \tProduce help message." },
    { UNRELIABLE,   0,  "u",    "unreliable",  Arg::None,       "  -u\t--unreliable  \tTransport in unreliable mode, default is reliable." },
    { 0, 0, 0, 0, 0, 0 }
};

/**
 *
 */
LatencyTestSubscriber::LatencyTestSubscriber()
{
    //cout << "LatencyTestSubscriber Constructure." << endl;
}

LatencyTestSubscriber::~LatencyTestSubscriber()
{
}

/**
 *
 */
void LatencyTestSubscriber::init(uint32_t unreliable)
{
    lat_pub = n.advertise<std_msgs::UInt8MultiArray>("latency_ack", 2000);
    if ( !unreliable )
    {
        cout << "TRANSPORT IN RELIABLE MODE" << endl;
        lat_sub = n.subscribe("latency", 2000, &LatencyTestSubscriber::laytencySubCallback, this);
    }
    else
    {
        cout << "TRANSPORT IN UNRELIABLE MODE" << endl;
        lat_sub = n.subscribe("latency", 2000, &LatencyTestSubscriber::laytencySubCallback, this, ros::TransportHints().unreliable().reliable().tcpNoDelay(true));
    }
    
    sub_index_ = 0;
    std::string s = ros::this_node::getName();
    std::regex  e = std::regex("[^0-9]*([0-9]+).*");
    std::smatch m;
    if ( std::regex_search(s, m, e))
    {
        auto x = m[m.size()-1];
        std::string tmp = x.str();
        sub_index_ = std::stoi(tmp);
    }
    cout << "SUBSCRIBER INDEX: " << sub_index_ << endl;
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void LatencyTestSubscriber::laytencySubCallback(const std_msgs::UInt8MultiArray::Ptr& msg)
{
    //cout << "laytencySubCallback msg->data[0] = " << *(uint32_t*)&msg->data[0] << endl;
    uint32_t stop_cmd = *(uint32_t*)&msg->data[0];
    if (( stop_cmd == STOP_CMD ) || ( msg->data[4] == (uint8_t)sub_index_))
    {
        lat_pub.publish(msg);
    }
}

void msg_rcv_thread( void )
{
    cout <<"MESSAGE RECIEVE THREAD CREATED" << endl;
    ros::spin();
}


int main(int argc, char **argv)
{
    /**
     * Parser the arguments.
     */
    uint32_t unreliable=0;
    int columns;
    columns = getenv("COLUMNS")?atoi(getenv("COLUMNS")):80;

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
    ros::init(argc, argv, "listener");
    
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
            case UNKNOWN_OPT:
                option::printUsage(fwrite, stdout, usage, columns);
                break;
        }
    }
    
    /**
     * Create thread for ros::spin();
     */
    std::thread rcvthrd(msg_rcv_thread);
  
    /**
     *
     */
    LatencyTestSubscriber latencySub;
    latencySub.init(unreliable);
    while(ros::ok()){;}
    return 0;
}
