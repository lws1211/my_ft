#include "full_coverage.hpp"
#include <signal.h>
#include <iostream>

void mySigintHandler(int sig)
{
    std::cout << "-----------------------------------------------------" << std::endl;
    std::cout << "[END] Ending LaneRecog Node ... " << std::endl;
    ros::shutdown();
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "coverage_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    signal(SIGINT, mySigintHandler);

    std::cout << "[START] Starting coverage Node ... " << std::endl;
    
    lws_nav::full_coverage FC;
    try
    {
        lws_nav::full_coverage FC(&nh);
    }
    catch(const std::exception& e)
    {   
        std::cerr << "ending................." << '\n';
        std::cerr << e.what() << '\n';
        mySigintHandler(SIGINT);
    }

    return 0;
}