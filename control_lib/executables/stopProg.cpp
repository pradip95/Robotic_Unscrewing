#include "control_lib/TaskInterface.h"

//short programm to stop robot and reset

int main(int argc, char** argv)
{
    TaskInterface doTasks;
    doTasks.Robot_Task->rtde_control->stopScript();

    std::vector<double> TCPspeed;
    std::this_thread::sleep_for(std::chrono::seconds{1});
    TCPspeed = doTasks.Robot_Task->rtde_receive->getActualTCPSpeed();
    if(TCPspeed.at(0) != 0 && TCPspeed.at(1) != 0 && TCPspeed.at(2) != 0 && TCPspeed.at(3) != 0 && TCPspeed.at(4) != 0 && TCPspeed.at(5) != 0)
    {
        doTasks.Robot_Task->rtde_control->triggerProtectiveStop();
        std::this_thread::sleep_for(std::chrono::seconds{1});
        doTasks.Robot_Task->rtde_control->unlockProtectiveStop();
    }
}
