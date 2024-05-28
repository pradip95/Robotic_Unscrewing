// Skript to validate the search strategies

#define SK  // SCREW = SK / T20 / T25
#define defSpiralsucheWithLateralForce   // SEARCHSTRATEGIE = defSpiralsuche / defQuadrantenSpiralsuche / defSpiralsucheWithLateralForce /
                                //                   defSukzessiveApproximationWithLifting / defSukkzessiveApproximationWithoutLifting /
                                //                   defLinearschwingungUmMittelpunkt / defLissajousscheFiguren
const int testrunCount = 1;


#include "control_lib/TaskInterface.h"
#include "control_lib/UsefullFunctions.h"

std::vector<double> produceRandomErrorPos(std::vector<double> exactPos, double maxError, std::vector<double>& xOffset, std::vector<double>& zOffset, TaskInterface* taskInterface)
{   
    //random numbers between 0 and 1
    double randomScale1 = (double)rand() / (double)RAND_MAX;
    double randomScale2 = (double)rand() / (double)RAND_MAX;
    
    //create random Angle between 0 and 2*PI, random magnitude of vector between 0 and maxError
    double randomAngle = 2 * PI * randomScale1;
    double randomMagnitude = maxError * randomScale2;

    //calculate vectorcomponents
    double xComp = randomMagnitude * cos(randomAngle);
    xOffset.push_back(xComp);
    double zComp = randomMagnitude * sin(randomAngle);
    zOffset.push_back(zComp);

    ROS_INFO_STREAM("Current error:  " << sqrt(pow(xComp, 2) + pow(zComp, 2)) << " m");

    //calculate error Pose of UR
    std::vector<double> transVec = {xComp, 0, zComp, 0, 0, 0};
    std::vector<double> errorPos = taskInterface->Robot_Task->rtde_control->poseTrans(exactPos, transVec);

    return errorPos;
}

int main(int argc, char** argv)
{ 
    ROS_INFO_STREAM("Name of File: (contain info: Versuchsreihe/Parameter/Schraubkopf))");
    std::string filename;
    std::cin >> filename;

    //Testparameter
    double maxError = 0.004;              
    #ifdef defQuadrantenSpiralsuche
        double radiusSearchfield = 0.002;
    #else
        double radiusSearchfield = 0.004;
    #endif
    double strengthMax = 7;
    double strengthMin = 4;

    #ifdef defSpiralsucheWithLateralForce
        double velocity = 0.015;
        double abortAtDepth = 0.005;
    #else
        #ifdef SK
            double velocity = 0.0074256;
            double abortAtDepth = 0.002;
        #endif
        #ifdef T20
            double velocity = 0.00168;
            double abortAtDepth = 0.0015;
        #endif
        #ifdef T25
            double velocity = 0.00228;
            double abortAtDepth = 0.001;
        #endif
    #endif

    
    srand(time(NULL));
    TaskInterface doTasks;
    //doTasks.Robot_Task->gotoTP(CLAMPDEVICE, true, 0.05, 0.02);
    
    // move robot by hand to the screwposition (tool in contact with screwhead)
    ROS_INFO_STREAM("Guide robot to screwposition (in contact) and press any key ...");
    doTasks.Robot_Task->freeDrive(1);

    // start position: robot moves up 2 cm
    std::vector<double> startPos = doTasks.Robot_Task->rtde_control->poseTrans(doTasks.Robot_Task->rtde_receive->getTargetTCPPose(), {0, 0.02, 0, 0, 0, 0});
    doTasks.Robot_Task->gotoTP(startPos, false, 0.05, 0.02);

    std::vector<double> executeTime;
    std::vector<double> failedVec;
    std::vector<double> xOffset, zOffset;
    
    for(int i = 0; i < testrunCount; i++)
    {   
        // robot moves in random dircetion parallel to screwsurface (simulate error of positiondetection of camera)
        std::vector<double> errorPos = produceRandomErrorPos(doTasks.Robot_Task->rtde_receive->getTargetTCPPose(), maxError, xOffset, zOffset, &doTasks);
        doTasks.Robot_Task->gotoTP(errorPos, false, 0.05, 0.02);

        doTasks.setGeneralVariables(radiusSearchfield, strengthMax, strengthMin, velocity, abortAtDepth);
        
        // execute search strategie with logging
        auto startExecuteTime = std::chrono::system_clock::now();

        #ifdef defSpiralsuche
            #ifdef SK
                int failed = doTasks.sp_search_.execute(0.0006188, &doTasks); 
            #endif
            #ifdef T20
                int failed = doTasks.sp_search_.execute(0.00014, &doTasks);
            #endif
            #ifdef T25
                int failed = doTasks.sp_search_.execute(0.00019, &doTasks);
            #endif
        #endif

        #ifdef defQuadrantenSpiralsuche
            #ifdef SK
                int failed = doTasks.sp_search_.executeQuadrantSearch(0.0006188, 0.002, &doTasks); 
            #endif
            #ifdef T20
                int failed = doTasks.sp_search_.executeQuadrantSearch(0.00014, 0.002, &doTasks);
            #endif
            #ifdef T25
                int failed = doTasks.sp_search_.executeQuadrantSearch(0.00019, 0.002, &doTasks);
            #endif
        #endif

        #ifdef defSpiralsucheWithLateralForce
            int failed = doTasks.sp_search_.executeWithLateralControl(0.0015, &doTasks);
        #endif

        #ifdef defSukzessiveApproximationWithLifting
            int failed = doTasks.saBrian_search_.execute(0.001155, 30, &doTasks);       //SK - TEST (zu lange Laufzeit)
        #endif

        #ifdef defSukkzessiveApproximationWithoutLifting
            #ifdef SK
                int failed = doTasks.saNave_search_.execute(0.0014939, 40, &doTasks); 
            #endif
            #ifdef T20
                int failed = doTasks.saNave_search_.execute(0.00033799, 200, &doTasks);
            #endif
            #ifdef T25
                int failed = doTasks.saNave_search_.execute(0.000458701, 200, &doTasks);
            #endif
        #endif

        #ifdef defLinearschwingungUmMittelpunkt
            #ifdef SK
                int failed = doTasks.lo_search_.execute(0.0006188, &doTasks); 
            #endif
            #ifdef T20
                int failed = doTasks.lo_search_.execute(0.00014, &doTasks);
            #endif
            #ifdef T25
                int failed = doTasks.lo_search_.execute(0.00019, &doTasks);
            #endif
        #endif

        #ifdef defLissajousscheFiguren
            #ifdef SK
                int failed = doTasks.lf_search_.execute(0.0006188, &doTasks); 
            #endif
            #ifdef T20
                int failed = doTasks.lf_search_.execute(0.00014, &doTasks);
            #endif
            #ifdef T25
                int failed = doTasks.lf_search_.execute(0.00019, &doTasks);
            #endif
        #endif

        auto endExecuteTime = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsedSecondsExecute;
        elapsedSecondsExecute = endExecuteTime-startExecuteTime;
        executeTime.push_back((double)elapsedSecondsExecute.count());
        failedVec.push_back((double)failed);

        // move up 3 cm then back to startPos
        std::vector<double> upPos = doTasks.Robot_Task->rtde_control->poseTrans(doTasks.Robot_Task->rtde_receive->getTargetTCPPose(), {0, 0.03, 0, 0, 0, 0});
        doTasks.Robot_Task->gotoTP(upPos, false, 0.05, 0.02);
        doTasks.Robot_Task->gotoTP(startPos, false, 0.05, 0.02);
        ROS_INFO_STREAM("Finished run " << i + 1);

        // safe to .csv file
        std::string directory = "/home/nucuser/repos/wbk-effector-group/urcontroller2/log_searchstrategies/" + filename + ".csv";
        std::vector<std::pair<std::string, std::vector<double>>> vals = {{"x-offset [m]", xOffset}, {"z-offset [m]", zOffset}, {"execute Time [s]", executeTime},
                                                                        {"planning time [s]", doTasks.planningTime_}, {"failed?", failedVec}};
        //write_csv(directory, vals);
    }

    ROS_INFO_STREAM("End of main(): Test Done");
    ros::waitForShutdown();
}

//Notes delete later!
/*
d² + d² = (d - f)²
2d² = d² - 2df + f²
d² + 2df - f² = 0
f² - 2fd - d² = 0
(f - d)² - 2d² = 0
f = (+-)sqrt(2)d + d
f = d * ((+-)sqrt(2) + 1)
f = d * (sqrt(2) + 1)
f = d * 2.414213562
*/
// cos = a/h --> h = a/cos