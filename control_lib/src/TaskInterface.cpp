#include "control_lib/TaskInterface.h"


// Function to introduce a random error with a 4mm radius from the current position
void addRandomErrorsToXAndY(std::vector<double> &vector, double errorRange) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distribution(-errorRange, errorRange);

    if (vector.size() >= 2) {
        vector[0] += distribution(gen); // Add error to x
        vector[1] += distribution(gen); // Add error to y
    }
}

// Function to calculate a new pose with an offset in the Z-direction
std::vector<double> offsetInZDirection(const std::vector<double> &pose, double offset) {
    if (pose.size() != 6) {
        // Ensure the input pose vector has the expected size (6 elements for [x, y, z, roll, pitch, yaw])
        throw std::invalid_argument("Input pose vector should have 6 elements.");
    }

    // Create a new pose vector with the same values as the current pose
    std::vector<double> newPoseZ = pose;

    // Update the Z-coordinate by adding the offset
    newPoseZ[2] += offset;

    return newPoseZ;
}

int Spiralsuche::plan(TaskInterface* taskInterface)
{
  if (taskInterface->radius_searchfield_ < distance_trajectory_)
  {
    ROS_ERROR_STREAM(
      "ERROR: planning not possible radius_searchfield_ must be greater than distance_trajectory_");
    return 1;
  }
  else if (approximation_increment_ > distance_steps_)
  {
    ROS_ERROR_STREAM("ERROR: planning not accurate enougth distance_steps_ must be greater than "
                     "approximation_increment_");
    return 1;
  }

  double phi1                     = 0;
  double phi2                     = 0;
  double a                        = distance_trajectory_ / (2 * PI);
  std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();

  std::vector<std::vector<double> > path;
  std::vector<double> xPoints, zPoints;
  int i = 0;
  while (true)
  {
    double phi2Constraint =
      pow(tan(-phi2 + phi1) * phi1 * a, 2) + pow(a * phi2 - cos(-phi2 + phi1) * phi1 * a, 2);

    if (phi2Constraint > pow(distance_steps_, 2) ||
        (i == 0 && phi2Constraint > pow(distance_trajectory_, 2)))
    {
      i++;

      double x1 = a * phi1 * cos(phi1);
      double z1 = a * phi1 * sin(phi1);

      double x2 = a * phi2 * cos(phi2);
      double z2 = a * phi2 * sin(phi2);

      double xComponent = x2 - x1;
      double zComponent = z2 - z1;

      xPoints.push_back(x2);
      zPoints.push_back(z2);

      std::vector<double> transformationVector = {xComponent, 0, zComponent, 0, 0, 0};
      std::vector<double> waypoint =
        taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, transformationVector);
      currentPose = waypoint;
      path.push_back(waypoint);

      phi1 = phi2;

      if (taskInterface->radius_searchfield_ <= sqrt(pow(x2, 2) + pow(z2, 2)))
      {
        break;
      }
    }
    phi2 = phi2 + approximation_increment_;
  }
  path_ = path;

  xPoints.insert(xPoints.begin(), 0);
  zPoints.insert(zPoints.begin(), 0);
  std::vector<std::pair<std::string, std::vector<double> > > vals = {{"xValue", xPoints},
                                                                     {"zValue", zPoints}};
  write_csv("/home/nucuser/repos/wbk-effector-group/urcontroller2/plot/sp_search.csv", vals);

  return 0;
}


int Spiralsuche::testScrewAlignment_hex(double distanceTrajectory, TaskInterface* taskInterface) {
  
  // Define constants for readability
  const double initialVelocity = 0.3; 
  const double spiralVelocity = 0.1; // 0.015 slower
  const double searchRadius = 0.004;
  const double maxForce = taskInterface->strength_max_; // 7.0 N
  const double minForce = taskInterface->strength_min_; // 4.0 N
  const double forceThreshold = 2.0; // Adjust as needed or 4
  double acceleration = taskInterface->acceleration_;
  double velocity = taskInterface->velocity_;
  double deceleration = 10;
  double angleIncrement = 0.001; 
  double errorRange = 0.0015; // ±4 mm error range
  std::chrono::duration<double> elapsedTime;

  // Initialize the force/torque sensor
  taskInterface->Robot_Task->rtde_control->zeroFtSensor();

  //std::vector<double> screwHeadPointOriginal = {-0.15724, -0.51879, 0.33346, 1.181, -1.269, -1.206}; //original center screw1
  //std::vector<double> screwHeadPointOriginal = {-0.13370, -0.46071, 0.33346, 1.181, -1.269, -1.206}; //original center screw2
  //std::vector<double> screwHeadPointOriginal = {-0.18821, -0.47083, 0.33346, 1.051, -1.396, -1.364}; //original center screw1 near robot center
  std::vector<double> screwHeadPointOriginal = {-0.18521, -0.52954, 0.33346, 1.062, -1.392, -1.380}; //original center screw2 away

  std::cout << "Original screwHeadPoint: ";
  for (const double &value : screwHeadPointOriginal) {
      std::cout << value << " ";
  }
  std::cout << std::endl;

  std::vector<double> screwHeadPoint = screwHeadPointOriginal;
  // Add random errors of upto ±4 mm to the pose vector element in the X and Y directions  
  addRandomErrorsToXAndY(screwHeadPoint, errorRange);

  // Display the updated pose vector
  std::cout << "Updated screwHeadPoint: ";
  for (const double &value : screwHeadPoint) {
      std::cout << value << " ";
  }
  std::cout << std::endl;

  // Calculate error in x and y
  std::vector<double> errorScrewHead(2);
  errorScrewHead[0] = std::abs(screwHeadPointOriginal[0] - screwHeadPoint[0]);
  errorScrewHead[1] = std::abs(screwHeadPointOriginal[1] - screwHeadPoint[1]);
  // Scale the error values by 1000
  errorScrewHead[0] *= 1000;
  errorScrewHead[1] *= 1000;
  // Print the error
  std::cout << "Error in x and y:" << std::endl;
  std::cout << "Error in x: " << errorScrewHead[0] << std::endl;
  std::cout << "Error in y: " << errorScrewHead[1] << std::endl;

  // Move to the updated screw head point with error
  taskInterface->Robot_Task->rtde_control->moveL(screwHeadPoint, initialVelocity, acceleration, false);
  std::cout << "Screw point reached" << std::endl;
  
  // Move down till the screw driver tool is in contact with the screw head with specified speed
  std::vector<double> toolSpeed = {0, 0, -0.005, 0, 0, 0};
  taskInterface->Robot_Task->rtde_control->moveUntilContact(toolSpeed);

  // Get the current pose of the robot when in contact with the screw head and print it
  std::vector<double> screw_contact = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
  std::cout << "contactPose" << std::endl;
  for (double value : screw_contact){
                std::cout << value << " ";
            }
  std::cout << std::endl;
  
  // Update the Z-axis coordinate of the pose to move down by 0.5 mm to give lateral force
  std::vector<double> screwContactLateralForce = screw_contact;
  double zOffset = -0.0001; // 0.5 mm
  std::vector<double> interpolateStartPoint = offsetInZDirection(screwContactLateralForce, zOffset);
  
  // Get the current force values and print them
  std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();
  std::cout << "refTCPForce" << std::endl;
  for (double value : refTCPForce){
                std::cout << value << " ";
            }
  std::cout << std::endl;

  // Perform interpolation on screwContact_interpolate to generate a spiral search path and start planning time
  auto startSearchTime = std::chrono::system_clock::now();
  std::vector<std::vector<double>> interpolatedPoses = taskInterface->Spiral->interpolateSpiral(angleIncrement, distanceTrajectory, searchRadius, interpolateStartPoint);
  
  bool engaged = false; // Flag to indicate if the screw is engaged
  auto startfastenTime = std::chrono::system_clock::now(); // Start fastening time
  
  // Iterate over the interpolated poses
  for (int p = 0; p < interpolatedPoses.size(); p++) {
    // Trigger the fastening action if the drill is ready and not engaged
    if (taskInterface->Drill_Task->isDrillReady() && !engaged) {
      taskInterface->Drill_Task->triggerProg(FASTEN);
    }

    // Printing step number of the spiral search interpolated poses
    std::cout << "Step " << p << std::endl;

    // Move to the interpolated pose with the specified velocity and acceleration
    taskInterface->Robot_Task->rtde_control->moveL(interpolatedPoses[p], spiralVelocity, acceleration, true);  

    // Get the current force values after moving to the interpolated pose
    std::vector<double> TCPForce = taskInterface->Robot_Task->getToolFrameForce();

    std::cout << "Force values: ";
    for (double value : TCPForce) {
      std::cout << value << " ";
    }
    std::cout << std::endl;

    // Check for a engaging condition and break and trigger a loosening action if the condition is met
    if ((TCPForce.at(1) < minForce && TCPForce.at(0) > forceThreshold) ||
        (TCPForce.at(1) < minForce && TCPForce.at(2) > forceThreshold)){
      taskInterface->Robot_Task->rtde_control->stopL(deceleration); // Stop the robot
      engaged = true;
      std::cout << "----------------------------------LOOSENING FORCE DETECTED----------------------------------" << std::endl;
      for (double value : TCPForce) {
        std::cout << value << " ";
      }
      std::cout << std::endl;

      // Fetch current pose of the robot and adjust the Z-axis coordinate to move down by 2 mm
      std::vector<double> engaged_pose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
      std::vector<double> adjust_pose = engaged_pose;
      double pushdown = -0.002; // 2 mm 
      std::vector<double> pushed_pose = offsetInZDirection(adjust_pose, pushdown);
      taskInterface->Robot_Task->rtde_control->moveL(pushed_pose, initialVelocity, acceleration, false);
      taskInterface->Robot_Task->rtde_control->stopL(deceleration);    
      
      if (engaged == true && taskInterface->Drill_Task->isDrillReady()) {
        
        taskInterface->unscrew();
       
        ros::Duration(1.0).sleep(); 
      
   
        // Get the end time of the spiral search and calculate the elapsed time
        auto endSearchTime = std::chrono::system_clock::now();
        elapsedTime = endSearchTime - startSearchTime;
        std::cout << "Time elapsed: " << elapsedTime.count() << " seconds" << std::endl;

        // Fetch the current pose of the robot and adjust the Z-axis coordinate to move up by 15 mm
        std::vector<double> unscrewed_pose = pushed_pose;
        double pushUp = 0.025; // 15 mm in meters
        std::vector<double> pushedUp_pose = offsetInZDirection(unscrewed_pose, pushUp);
        taskInterface->Robot_Task->rtde_control->moveL(pushedUp_pose, velocity, acceleration, false);
        taskInterface->Robot_Task->rtde_control->stopL(deceleration);    

        // Get the current timestamp
        std::time_t now = std::time(nullptr);
        struct std::tm localTime;
        localtime_r(&now, &localTime); 

        // Create a filename based on the timestamp
        char buffer[80];
        std::strftime(buffer, sizeof(buffer), "output_%Y%m%d%H%M%S.csv", &localTime);

        std::string csvFilename(buffer);

        // Directory location to save the CSV file
        std::string directory = "/home/pradip/catkin_KIT/src/disassembly-station/screw_alignment/screw_alignment_data/";

        // Construct the full path for the CSV file
        std::string fullFilePath = directory + csvFilename;

        // Open the CSV file for writing
        std::ofstream csvFile(fullFilePath);

        if (csvFile.is_open()) {
            // Write the header row and data as previously shown
            csvFile << "Elapsed Time (s), Screw Head Point, Screw Head Point with Error, Error (X Direction), Error (Y Direction), Interpolated Pose, Engaged Pose \n";

            // Write the elapsed time
            csvFile << elapsedTime.count() << ",";
            
            // Write the screw head point original
            std::string screwHeadPointOriginalStr;
            for (double value : screwHeadPointOriginal) {
                screwHeadPointOriginalStr += std::to_string(value) + ",";
            }
            // Remove the trailing comma
            if (!screwHeadPointOriginalStr.empty()) {
                screwHeadPointOriginalStr.pop_back();
            }
            csvFile << "\"" << screwHeadPointOriginalStr << "\",";

            // Write the screw head point with error
            std::string screwHeadPointStr;
            for (double value : screwHeadPoint) {
                screwHeadPointStr += std::to_string(value) + ",";
            }
            // Remove the trailing comma
            if (!screwHeadPointStr.empty()) {
                screwHeadPointStr.pop_back();
            }
            csvFile << "\"" << screwHeadPointStr << "\",";

            // Error in X and Y
            csvFile << errorScrewHead[0] << "," << errorScrewHead[1] << ",";

            // Write the interpolated poses
            std::string interpolationStr;
            for (double value : interpolatedPoses[p]) {
                interpolationStr += std::to_string(value) + ",";
            }
            // Remove the trailing comma
            if (!interpolationStr.empty()) {
                interpolationStr.pop_back();
            }
            csvFile << "\"" << interpolationStr << "\",";

            // Write the engages screw pose
            std::string engagedPoseStr;
            for (double value : pushed_pose) {
                engagedPoseStr += std::to_string(value) + ",";
            }
            // Remove the trailing comma
            if (!engagedPoseStr.empty()) {
                engagedPoseStr.pop_back();
            }
            csvFile << "\"" << engagedPoseStr << "\"\n";

            csvFile.close(); // You can omit this part if you don't need to explicitly close the file.
        } else {
            std::cerr << "Failed to save the CSV file at the specified location." << std::endl;
        }
      }
      
      break;
    }

    // Get the end time of the fastening action and calculate the elapsed fastening time
    auto endfastenTime = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedFasteningTime;
    elapsedFasteningTime = endfastenTime - startfastenTime;
    // If the elapsed fastening time is greater than 13 seconds and the drill is ready, trigger the reset action
    if (elapsedFasteningTime.count() >= 13 && taskInterface->Drill_Task->isDrillReady()){
      taskInterface->Drill_Task->triggerProg(RESET); 
    } 
    
  }
  return 1;
}

int Spiralsuche::testScrewAlignment_hex_withSensor(double distanceTrajectory, TaskInterface* taskInterface) {
  
  // Define constants for readability
  const double initialVelocity = 0.4; 
  const double spiralVelocity = 0.4; // 0.015 slower
  double errorRange = 0.0011; // ±4 mm error range
  const double searchRadius = taskInterface->radius_searchfield_;
  //const double searchRadius = errorRange + 0.001;
  const double maxForce = taskInterface->strength_max_; // 7.0 N
  const double minForce = taskInterface->strength_min_; // 4.0 N
  const double forceThreshold = 1.0; // Adjust as needed or 4
  const double lateralForce = 15;
  double acceleration = taskInterface->acceleration_;
  double velocity = taskInterface->velocity_;
  double deceleration = 10;
  double angleIncrement = 0.001; 
  double fastenTimer = 15;

  // Initialize the force/torque sensor
  taskInterface->Robot_Task->rtde_control->zeroFtSensor();

  //std::vector<double> screwHeadPointOriginal = {-0.19239, -0.46079, 0.335, 1.063, -1.417, -1.377}; //original center screw1 near robot center
  std::vector<double> screwHeadPointOriginal = {-0.19744, -0.52484, 0.335, 1.064, -1.413, -1.385}; //original center screw2 away
  
  std::cout << "Original screwHeadPoint: ";
  for (const double &value : screwHeadPointOriginal) {
      std::cout << value << " ";
  }
  std::cout << std::endl;

  std::vector<double> screwHeadPoint = screwHeadPointOriginal;
  // Add random errors of upto ±4 mm to the pose vector element in the X and Y directions  
  addRandomErrorsToXAndY(screwHeadPoint, errorRange);

  // Display the updated pose vector
  std::cout << "Updated screwHeadPoint: ";
  for (const double &value : screwHeadPoint) {
      std::cout << value << " ";
  }
  std::cout << std::endl;

  // Calculate error in x and y
  std::vector<double> errorScrewHead(2);
  errorScrewHead[0] = std::abs(screwHeadPointOriginal[0] - screwHeadPoint[0]);
  errorScrewHead[1] = std::abs(screwHeadPointOriginal[1] - screwHeadPoint[1]);
  // Scale the error values by 1000
  errorScrewHead[0] *= 1000;
  errorScrewHead[1] *= 1000;
  // Print the error
  std::cout << "Error in x and y:" << std::endl;
  std::cout << "Error in x: " << errorScrewHead[0] << std::endl;
  std::cout << "Error in y: " << errorScrewHead[1] << std::endl;

  // Move to the updated screw head point with error
  taskInterface->Robot_Task->rtde_control->moveL(screwHeadPoint, initialVelocity, acceleration, false);
  std::cout << "Screw point reached" << std::endl;
  
  // Move down till the screw driver tool is in contact with the screw head with specified speed
  std::vector<double> toolSpeed = {0, 0, -0.005, 0, 0, 0};
  taskInterface->Robot_Task->rtde_control->moveUntilContact(toolSpeed);

  // Get the current pose of the robot when in contact with the screw head and print it
  std::vector<double> screw_contact = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
  std::cout << "contactPose" << std::endl;
  for (double value : screw_contact){
                std::cout << value << " ";
            }
  std::cout << std::endl;
  
  // Update the Z-axis coordinate of the pose to move down by 0.5 mm to give lateral force
  std::vector<double> screwContactLateralForce = screw_contact;
  double zOffset = -0.0006; // 0.5 mm
  std::vector<double> interpolateStartPoint = offsetInZDirection(screwContactLateralForce, zOffset);
  
  // Get the current force values and print them
  std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();
  std::cout << "refTCPForce" << std::endl;
  for (double value : refTCPForce){
                std::cout << value << " ";
            }
  std::cout << std::endl;

  // Perform interpolation on screwContact_interpolate to generate a spiral search path and start planning time
  auto startSearchTime = std::chrono::system_clock::now();
  std::vector<std::vector<double>> interpolatedPoses = taskInterface->Spiral->interpolateSpiral(angleIncrement, distanceTrajectory, searchRadius, interpolateStartPoint);
  
  bool engaged = false; // Flag to indicate if the screw is engaged
  bool unscrewed = false; // Flag to indicate unscrew task success or not
  auto startfastenTime = std::chrono::system_clock::now(); // Start fastening time
  taskInterface->Drill_Task->momentSuccess_ = false;
  // Iterate over the interpolated poses
  taskInterface->Drill_Task->triggerProg(RESET);
  for (int p = 0; p < interpolatedPoses.size(); p++) {
    // Trigger the fastening action if the drill is ready and not engaged
    if (taskInterface->Drill_Task->isDrillReady() && !engaged) {
      taskInterface->Drill_Task->triggerProg(FASTEN);
      //taskInterface->Drill_Task->triggerProg(RESET);
    }

    // Printing step number of the spiral search interpolated poses
    std::cout << "Step " << p << std::endl;

    // Move to the interpolated pose with the specified velocity and acceleration
    taskInterface->Robot_Task->rtde_control->moveL(interpolatedPoses[p], spiralVelocity, acceleration, true);  

    // Get the current force values after moving to the interpolated pose
    std::vector<double> TCPForce = taskInterface->Robot_Task->getToolFrameForce();

    std::cout << "Force values: ";
    for (double value : TCPForce) {
      std::cout << value << " ";
    }
    std::cout << std::endl;

    // Check for a engaging condition and break and trigger a loosening action if the condition is met
    if (taskInterface->Drill_Task->momentSuccessChanged){
      std::cout << "changed " << std::endl;
    if ((taskInterface->Drill_Task->getMomentSuccess())) {
      std::cout << "moment detected " << std::endl;

      taskInterface->Robot_Task->rtde_control->stopL(deceleration); // Stop the robot

      // Get the current force values after moving to the interpolated pose
      std::vector<double> TCPForce = taskInterface->Robot_Task->getToolFrameForce();

      std::cout << "Force values: ";
      for (double value : TCPForce) {
        std::cout << value << " ";
      }
      std::cout << std::endl;

      if ((TCPForce.at(1) < lateralForce && std::abs(TCPForce.at(0)) >= forceThreshold) ||
        (TCPForce.at(1) < lateralForce && std::abs(TCPForce.at(2)) >= forceThreshold)){
        engaged = true;
        std::cout << "engaged" << std::endl;
        taskInterface->Robot_Task->rtde_control->stopL(deceleration); // Stop the robot

        std::cout << "----------------------------------LOOSENING FORCE DETECTED----------------------------------" << std::endl;
        for (double value : TCPForce) {
        std::cout << value << " ";
        }
        std::cout << std::endl;

        // Fetch current pose of the robot and adjust the Z-axis coordinate to move down by 2 mm
        std::vector<double> engaged_pose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
        std::vector<double> adjust_pose = engaged_pose;
        double pushdown = -0.002; // 2 mm 
        std::vector<double> pushed_pose = offsetInZDirection(adjust_pose, pushdown);
        taskInterface->Robot_Task->rtde_control->moveL(pushed_pose, initialVelocity, acceleration, false);
        taskInterface->Robot_Task->rtde_control->stopL(deceleration);    
        
        if (engaged == true && taskInterface->Drill_Task->isDrillReady()) {
          taskInterface->unscrew();
       
          ros::Duration(1.0).sleep(); 
          // Get the end time of the spiral search and calculate the elapsed time
          auto endSearchTime = std::chrono::system_clock::now();
          std::chrono::duration<double> elapsedTime = endSearchTime - startSearchTime;
          std::cout << "Time elapsed: " << elapsedTime.count() << " seconds" << std::endl;

          // Fetch the current pose of the robot and adjust the Z-axis coordinate to move up by 15 mm
          std::vector<double> unscrewed_pose = pushed_pose;
          double pushUp = 0.015; // 15 mm in meters
          std::vector<double> pushedUp_pose = offsetInZDirection(unscrewed_pose, pushUp);
          taskInterface->Robot_Task->rtde_control->moveL(pushedUp_pose, velocity, acceleration, false);
          taskInterface->Robot_Task->rtde_control->stopL(deceleration);    
          unscrewed = true;
          // Get the current timestamp
          std::time_t now = std::time(nullptr);
          struct std::tm localTime;
          localtime_r(&now, &localTime); 

          // Create a filename based on the timestamp
          char buffer[80];
          std::strftime(buffer, sizeof(buffer), "output_%Y%m%d%H%M%S.csv", &localTime);

          std::string csvFilename(buffer);

          // Directory location to save the CSV file
          std::string directory = "/home/pradip/catkin_KIT/src/disassembly-station/screw_alignment/screw_alignment_data/";

          // Construct the full path for the CSV file
          std::string fullFilePath = directory + csvFilename;

          // Open the CSV file for writing
          std::ofstream csvFile(fullFilePath);

          if (csvFile.is_open()) {
              // Write the header row and data as previously shown
              csvFile << "Elapsed Time (s), Screw Head Point, Screw Head Point with Error, Error (X Direction), Error (Y Direction), Interpolated Pose, Engaged Pose \n";

              // Write the elapsed time
              csvFile << elapsedTime.count() << ",";
              
              // Write the screw head point original
              std::string screwHeadPointOriginalStr;
              for (double value : screwHeadPointOriginal) {
                  screwHeadPointOriginalStr += std::to_string(value) + ",";
              }
              // Remove the trailing comma
              if (!screwHeadPointOriginalStr.empty()) {
                  screwHeadPointOriginalStr.pop_back();
              }
              csvFile << "\"" << screwHeadPointOriginalStr << "\",";

              // Write the screw head point with error
              std::string screwHeadPointStr;
              for (double value : screwHeadPoint) {
                  screwHeadPointStr += std::to_string(value) + ",";
              }
              // Remove the trailing comma
              if (!screwHeadPointStr.empty()) {
                  screwHeadPointStr.pop_back();
              }
              csvFile << "\"" << screwHeadPointStr << "\",";

              // Error in X and Y
              csvFile << errorScrewHead[0] << "," << errorScrewHead[1] << ",";

              // Write the interpolated poses
              std::string interpolationStr;
              for (double value : interpolatedPoses[p]) {
                  interpolationStr += std::to_string(value) + ",";
              }
              // Remove the trailing comma
              if (!interpolationStr.empty()) {
                  interpolationStr.pop_back();
              }
              csvFile << "\"" << interpolationStr << "\",";

              // Write the engages screw pose
              std::string engagedPoseStr;
              for (double value : pushed_pose) {
                  engagedPoseStr += std::to_string(value) + ",";
              }
              // Remove the trailing comma
              if (!engagedPoseStr.empty()) {
                  engagedPoseStr.pop_back();
              }
              csvFile << "\"" << engagedPoseStr << "\"\n";

              csvFile.close(); // You can omit this part if you don't need to explicitly close the file.
          } else {
            std::cerr << "Failed to save the CSV file at the specified location." << std::endl;
          }
        }
      if (unscrewed) {
          return 1;
      }  
      break;
      }
    }
    // Reset the flag
    taskInterface->Drill_Task->momentSuccessChanged = false;
    }
    // Get the end time of the fastening action and calculate the elapsed fastening time
    auto endfastenTime = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedFasteningTime;
    elapsedFasteningTime = endfastenTime - startfastenTime;
    // If the elapsed fastening time is greater than 13 seconds and the drill is ready, trigger the reset action
    if (elapsedFasteningTime.count() >= fastenTimer && !engaged){
      taskInterface->Drill_Task->triggerProg(RESET); 
      // Reset the startfastenTime to the current time
      startfastenTime = std::chrono::high_resolution_clock::now();
      std::cout << "fastentime: " << elapsedFasteningTime.count() << std::endl;
    }  
  }
}


int Spiralsuche::testScrewAlignment_torx20_withSensor(double distanceTrajectory, TaskInterface* taskInterface) {
  
  // Define constants for readability
  const double initialVelocity = 0.4; 
  const double spiralVelocity = 0.4; // 0.015 slower
  const double searchRadius = taskInterface->radius_searchfield_;
  const double maxForce = taskInterface->strength_max_; // 7.0 N
  const double minForce = taskInterface->strength_min_; // 4.0 N
  const double forceThreshold = 1.0; // Adjust as needed or 4
  const double lateralForce = 15;
  double acceleration = taskInterface->acceleration_;
  double velocity = taskInterface->velocity_;
  double deceleration = 10;
  double angleIncrement = 0.001; 
  double errorRange = 0.00; // ±4 mm error range
  double fastenTimer = 15;

  // Initialize the force/torque sensor
  taskInterface->Robot_Task->rtde_control->zeroFtSensor();

  std::vector<double> screwHeadPointOriginal = {-0.24270, -0.59790, 0.30246, 1.188, -1.230, -1.199}; //original center screw1 near robot center
  //std::vector<double> screwHeadPointOriginal = {-0.23759, -0.58970, 0.28046, 1.052, -1.395, -1.365}; //original center screw2 away
  
  std::cout << "Original screwHeadPoint: ";
  for (const double &value : screwHeadPointOriginal) {
      std::cout << value << " ";
  }
  std::cout << std::endl;

  std::vector<double> screwHeadPoint = screwHeadPointOriginal;
  // Add random errors of upto ±4 mm to the pose vector element in the X and Y directions  
  addRandomErrorsToXAndY(screwHeadPoint, errorRange);

  // Display the updated pose vector
  std::cout << "Updated screwHeadPoint: ";
  for (const double &value : screwHeadPoint) {
      std::cout << value << " ";
  }
  std::cout << std::endl;

  // Calculate error in x and y
  std::vector<double> errorScrewHead(2);
  errorScrewHead[0] = std::abs(screwHeadPointOriginal[0] - screwHeadPoint[0]);
  errorScrewHead[1] = std::abs(screwHeadPointOriginal[1] - screwHeadPoint[1]);
  // Scale the error values by 1000
  errorScrewHead[0] *= 1000;
  errorScrewHead[1] *= 1000;
  // Print the error
  std::cout << "Error in x and y:" << std::endl;
  std::cout << "Error in x: " << errorScrewHead[0] << std::endl;
  std::cout << "Error in y: " << errorScrewHead[1] << std::endl;

  // Move to the updated screw head point with error
  taskInterface->Robot_Task->rtde_control->moveL(screwHeadPoint, initialVelocity, acceleration, false);
  std::cout << "Screw point reached" << std::endl;
  
  // Move down till the screw driver tool is in contact with the screw head with specified speed
  std::vector<double> toolSpeed = {0, 0, -0.009, 0, 0, 0};
  taskInterface->Robot_Task->rtde_control->moveUntilContact(toolSpeed);

  // Get the current pose of the robot when in contact with the screw head and print it
  std::vector<double> screw_contact = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
  std::cout << "contactPose" << std::endl;
  for (double value : screw_contact){
                std::cout << value << " ";
            }
  std::cout << std::endl;
  
  // Update the Z-axis coordinate of the pose to move down by 0.5 mm to give lateral force
  std::vector<double> screwContactLateralForce = screw_contact;
  double zOffset = -0.0005; // 0.5 mm
  std::vector<double> interpolateStartPoint = offsetInZDirection(screwContactLateralForce, zOffset);
  
  // Get the current force values and print them
  std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();
  std::cout << "refTCPForce" << std::endl;
  for (double value : refTCPForce){
                std::cout << value << " ";
            }
  std::cout << std::endl;

  // Perform interpolation on screwContact_interpolate to generate a spiral search path and start planning time
  auto startSearchTime = std::chrono::system_clock::now();
  std::vector<std::vector<double>> interpolatedPoses = taskInterface->Spiral->interpolateSpiral(angleIncrement, distanceTrajectory, searchRadius, interpolateStartPoint);
  
  bool engaged = false; // Flag to indicate if the screw is engaged
  bool unscrewed = false; // Flag to indicate unscrew task success or not
  auto startfastenTime = std::chrono::system_clock::now(); // Start fastening time
  taskInterface->Drill_Task->momentSuccess_ = false;
  // Iterate over the interpolated poses
  for (int p = 0; p < interpolatedPoses.size(); p++) {
    // Trigger the fastening action if the drill is ready and not engaged
    if (taskInterface->Drill_Task->isDrillReady() && !engaged) {
      taskInterface->Drill_Task->triggerProg(FASTEN);
      //taskInterface->Drill_Task->triggerProg(RESET);
    }

    // Printing step number of the spiral search interpolated poses
    std::cout << "Step " << p << std::endl;

    // Move to the interpolated pose with the specified velocity and acceleration
    taskInterface->Robot_Task->rtde_control->moveL(interpolatedPoses[p], spiralVelocity, acceleration, true);  

    // Get the current force values after moving to the interpolated pose
    std::vector<double> TCPForce = taskInterface->Robot_Task->getToolFrameForce();

    std::cout << "Force values: ";
    for (double value : TCPForce) {
      std::cout << value << " ";
    }
    std::cout << std::endl;

    // Check for a engaging condition and break and trigger a loosening action if the condition is met
    if (taskInterface->Drill_Task->momentSuccessChanged){
      std::cout << "changed " << std::endl;
    if ((taskInterface->Drill_Task->getMomentSuccess())) {
      std::cout << "moment detected " << std::endl;

      taskInterface->Robot_Task->rtde_control->stopL(deceleration); // Stop the robot

      // Get the current force values after moving to the interpolated pose
      std::vector<double> TCPForce = taskInterface->Robot_Task->getToolFrameForce();

      std::cout << "Force values: ";
      for (double value : TCPForce) {
        std::cout << value << " ";
      }
      std::cout << std::endl;

      if ((TCPForce.at(1) < lateralForce && std::abs(TCPForce.at(0)) >= forceThreshold) ||
        (TCPForce.at(1) < lateralForce && std::abs(TCPForce.at(2)) >= forceThreshold)){
        engaged = true;
        std::cout << "engaged" << std::endl;
        taskInterface->Robot_Task->rtde_control->stopL(deceleration); // Stop the robot

        std::cout << "----------------------------------LOOSENING FORCE DETECTED----------------------------------" << std::endl;
        for (double value : TCPForce) {
        std::cout << value << " ";
        }
        std::cout << std::endl;

        // Fetch current pose of the robot and adjust the Z-axis coordinate to move down by 2 mm
        std::vector<double> engaged_pose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
        std::vector<double> adjust_pose = engaged_pose;
        double pushdown = -0.001; // 2 mm 
        std::vector<double> pushed_pose = offsetInZDirection(adjust_pose, pushdown);
        taskInterface->Robot_Task->rtde_control->moveL(pushed_pose, initialVelocity, acceleration, false);
        taskInterface->Robot_Task->rtde_control->stopL(deceleration);    
        
        if (engaged == true && taskInterface->Drill_Task->isDrillReady()) {
          taskInterface->unscrew();
       
          //ros::Duration(1.0).sleep(); 
          // Get the end time of the spiral search and calculate the elapsed time
          auto endSearchTime = std::chrono::system_clock::now();
          std::chrono::duration<double> elapsedTime = endSearchTime - startSearchTime;
          std::cout << "Time elapsed: " << elapsedTime.count() << " seconds" << std::endl;

          // Fetch the current pose of the robot and adjust the Z-axis coordinate to move up by 15 mm
          std::vector<double> unscrewed_pose = pushed_pose;
          double pushUp = 0.015; // 15 mm in meters
          std::vector<double> pushedUp_pose = offsetInZDirection(unscrewed_pose, pushUp);
          taskInterface->Robot_Task->rtde_control->moveL(pushedUp_pose, velocity, acceleration, false);
          taskInterface->Robot_Task->rtde_control->stopL(deceleration);    
          unscrewed = true;
          // Get the current timestamp
          std::time_t now = std::time(nullptr);
          struct std::tm localTime;
          localtime_r(&now, &localTime); 

          // Create a filename based on the timestamp
          char buffer[80];
          std::strftime(buffer, sizeof(buffer), "output_%Y%m%d%H%M%S.csv", &localTime);

          std::string csvFilename(buffer);

          // Directory location to save the CSV file
          std::string directory = "/home/pradip/catkin_KIT/src/disassembly-station/screw_alignment/screw_alignment_data_T20/";

          // Construct the full path for the CSV file
          std::string fullFilePath = directory + csvFilename;

          // Open the CSV file for writing
          std::ofstream csvFile(fullFilePath);

          if (csvFile.is_open()) {
              // Write the header row and data as previously shown
              csvFile << "Elapsed Time (s), Screw Head Point, Screw Head Point with Error, Error (X Direction), Error (Y Direction), Interpolated Pose, Engaged Pose \n";

              // Write the elapsed time
              csvFile << elapsedTime.count() << ",";
              
              // Write the screw head point original
              std::string screwHeadPointOriginalStr;
              for (double value : screwHeadPointOriginal) {
                  screwHeadPointOriginalStr += std::to_string(value) + ",";
              }
              // Remove the trailing comma
              if (!screwHeadPointOriginalStr.empty()) {
                  screwHeadPointOriginalStr.pop_back();
              }
              csvFile << "\"" << screwHeadPointOriginalStr << "\",";

              // Write the screw head point with error
              std::string screwHeadPointStr;
              for (double value : screwHeadPoint) {
                  screwHeadPointStr += std::to_string(value) + ",";
              }
              // Remove the trailing comma
              if (!screwHeadPointStr.empty()) {
                  screwHeadPointStr.pop_back();
              }
              csvFile << "\"" << screwHeadPointStr << "\",";

              // Error in X and Y
              csvFile << errorScrewHead[0] << "," << errorScrewHead[1] << ",";

              // Write the interpolated poses
              std::string interpolationStr;
              for (double value : interpolatedPoses[p]) {
                  interpolationStr += std::to_string(value) + ",";
              }
              // Remove the trailing comma
              if (!interpolationStr.empty()) {
                  interpolationStr.pop_back();
              }
              csvFile << "\"" << interpolationStr << "\",";

              // Write the engages screw pose
              std::string engagedPoseStr;
              for (double value : pushed_pose) {
                  engagedPoseStr += std::to_string(value) + ",";
              }
              // Remove the trailing comma
              if (!engagedPoseStr.empty()) {
                  engagedPoseStr.pop_back();
              }
              csvFile << "\"" << engagedPoseStr << "\"\n";

              csvFile.close(); // You can omit this part if you don't need to explicitly close the file.
          } else {
            std::cerr << "Failed to save the CSV file at the specified location." << std::endl;
          }
        }
      if (unscrewed) {
          return 1;
      }  
      break;
      }
    }
    // Reset the flag
    taskInterface->Drill_Task->momentSuccessChanged = false;
    }
    // Get the end time of the fastening action and calculate the elapsed fastening time
    auto endfastenTime = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedFasteningTime;
    elapsedFasteningTime = endfastenTime - startfastenTime;
    // If the elapsed fastening time is greater than 13 seconds and the drill is ready, trigger the reset action
    if (elapsedFasteningTime.count() >= fastenTimer && !engaged){
      taskInterface->Drill_Task->triggerProg(RESET); 
      // Reset the startfastenTime to the current time
      startfastenTime = std::chrono::high_resolution_clock::now();
      std::cout << "fastentime: " << elapsedFasteningTime.count() << std::endl;
    }  
  }
}



int Spiralsuche::testScrewAlignment_torx25_withSensor(double distanceTrajectory, TaskInterface* taskInterface) {
  
  // Define constants for readability
  const double initialVelocity = 0.4; 
  const double spiralVelocity = 0.4; // 0.015 slower
  const double searchRadius = taskInterface->radius_searchfield_;
  const double maxForce = taskInterface->strength_max_; // 7.0 N
  const double minForce = taskInterface->strength_min_; // 4.0 N
  const double forceThreshold = 1.0; // Adjust as needed or 4
  const double lateralForce = 15;
  double acceleration = taskInterface->acceleration_;
  double velocity = taskInterface->velocity_;
  double deceleration = 10;
  double angleIncrement = 0.001; 
  double errorRange = 0.001; // ±4 mm error range
  double fastenTimer = 15;

  // Initialize the force/torque sensor
  taskInterface->Robot_Task->rtde_control->zeroFtSensor();

  std::vector<double> screwHeadPointOriginal = {-0.21779, -0.477790, 0.360, 0.786, -1.703, -1.657}; //original center screw1 near robot center
  
  
  std::cout << "Original screwHeadPoint: ";
  for (const double &value : screwHeadPointOriginal) {
      std::cout << value << " ";
  }
  std::cout << std::endl;

  std::vector<double> screwHeadPoint = screwHeadPointOriginal;
  // Add random errors of upto ±4 mm to the pose vector element in the X and Y directions  
  addRandomErrorsToXAndY(screwHeadPoint, errorRange);

  // Display the updated pose vector
  std::cout << "Updated screwHeadPoint: ";
  for (const double &value : screwHeadPoint) {
      std::cout << value << " ";
  }
  std::cout << std::endl;

  // Calculate error in x and y
  std::vector<double> errorScrewHead(2);
  errorScrewHead[0] = std::abs(screwHeadPointOriginal[0] - screwHeadPoint[0]);
  errorScrewHead[1] = std::abs(screwHeadPointOriginal[1] - screwHeadPoint[1]);
  // Scale the error values by 1000
  errorScrewHead[0] *= 1000;
  errorScrewHead[1] *= 1000;
  // Print the error
  std::cout << "Error in x and y:" << std::endl;
  std::cout << "Error in x: " << errorScrewHead[0] << std::endl;
  std::cout << "Error in y: " << errorScrewHead[1] << std::endl;

  // Move to the updated screw head point with error
  taskInterface->Robot_Task->rtde_control->moveL(screwHeadPoint, initialVelocity, acceleration, false);
  std::cout << "Screw point reached" << std::endl;
  
  // Move down till the screw driver tool is in contact with the screw head with specified speed
  std::vector<double> toolSpeed = {0, 0, -0.009, 0, 0, 0};
  taskInterface->Robot_Task->rtde_control->moveUntilContact(toolSpeed);

  // Get the current pose of the robot when in contact with the screw head and print it
  std::vector<double> screw_contact = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
  std::cout << "contactPose" << std::endl;
  for (double value : screw_contact){
                std::cout << value << " ";
            }
  std::cout << std::endl;
  
  // Update the Z-axis coordinate of the pose to move down by 0.5 mm to give lateral force
  std::vector<double> screwContactLateralForce = screw_contact;
  double zOffset = -0.0007; // 0.4 mm
  std::vector<double> interpolateStartPoint = offsetInZDirection(screwContactLateralForce, zOffset);
  
  // Get the current force values and print them
  std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();
  std::cout << "refTCPForce" << std::endl;
  for (double value : refTCPForce){
                std::cout << value << " ";
            }
  std::cout << std::endl;

  // Perform interpolation on screwContact_interpolate to generate a spiral search path and start planning time
  auto startSearchTime = std::chrono::system_clock::now();
  std::vector<std::vector<double>> interpolatedPoses = taskInterface->Spiral->interpolateSpiral(angleIncrement, distanceTrajectory, searchRadius, interpolateStartPoint);
  
  bool engaged = false; // Flag to indicate if the screw is engaged
  bool unscrewed = false; // Flag to indicate unscrew task success or not
  auto startfastenTime = std::chrono::system_clock::now(); // Start fastening time
  taskInterface->Drill_Task->momentSuccess_ = false;
  // Iterate over the interpolated poses
  for (int p = 0; p < interpolatedPoses.size(); p++) {
    // Trigger the fastening action if the drill is ready and not engaged
    if (taskInterface->Drill_Task->isDrillReady() && !engaged) {
      taskInterface->Drill_Task->triggerProg(FASTEN);
      //taskInterface->Drill_Task->triggerProg(RESET);
    }

    // Printing step number of the spiral search interpolated poses
    std::cout << "Step " << p << std::endl;

    // Move to the interpolated pose with the specified velocity and acceleration
    taskInterface->Robot_Task->rtde_control->moveL(interpolatedPoses[p], spiralVelocity, acceleration, true);  

    // Get the current force values after moving to the interpolated pose
    std::vector<double> TCPForce = taskInterface->Robot_Task->getToolFrameForce();

    std::cout << "Force values: ";
    for (double value : TCPForce) {
      std::cout << value << " ";
    }
    std::cout << std::endl;

    // Check for a engaging condition and break and trigger a loosening action if the condition is met
    if (taskInterface->Drill_Task->momentSuccessChanged){
      std::cout << "changed " << std::endl;
    if ((taskInterface->Drill_Task->getMomentSuccess())) {
      std::cout << "moment detected " << std::endl;

      taskInterface->Robot_Task->rtde_control->stopL(deceleration); // Stop the robot

      // Get the current force values after moving to the interpolated pose
      std::vector<double> TCPForce = taskInterface->Robot_Task->getToolFrameForce();

      std::cout << "Force values: ";
      for (double value : TCPForce) {
        std::cout << value << " ";
      }
      std::cout << std::endl;

      if ((TCPForce.at(1) < lateralForce && std::abs(TCPForce.at(0)) >= forceThreshold) ||
        (TCPForce.at(1) < lateralForce && std::abs(TCPForce.at(2)) >= forceThreshold)){
        engaged = true;
        std::cout << "engaged" << std::endl;
        taskInterface->Robot_Task->rtde_control->stopL(deceleration); // Stop the robot

        std::cout << "----------------------------------LOOSENING FORCE DETECTED----------------------------------" << std::endl;
        for (double value : TCPForce) {
        std::cout << value << " ";
        }
        std::cout << std::endl;

        // Fetch current pose of the robot and adjust the Z-axis coordinate to move down by 2 mm
        std::vector<double> engaged_pose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
        std::vector<double> adjust_pose = engaged_pose;
        double pushdown = -0.002; // 2 mm 
        std::vector<double> pushed_pose = offsetInZDirection(adjust_pose, pushdown);
        taskInterface->Robot_Task->rtde_control->moveL(pushed_pose, initialVelocity, acceleration, false);
        taskInterface->Robot_Task->rtde_control->stopL(deceleration);    
        
        if (engaged == true && taskInterface->Drill_Task->isDrillReady()) {
          taskInterface->unscrew();
       
          ros::Duration(3).sleep(); 
          // Get the end time of the spiral search and calculate the elapsed time
          auto endSearchTime = std::chrono::system_clock::now();
          std::chrono::duration<double> elapsedTime = endSearchTime - startSearchTime;
          std::cout << "Time elapsed: " << elapsedTime.count() << " seconds" << std::endl;

          // Fetch the current pose of the robot and adjust the Z-axis coordinate to move up by 15 mm
          std::vector<double> unscrewed_pose = pushed_pose;
          double pushUp = 0.015; // 15 mm in meters
          std::vector<double> pushedUp_pose = offsetInZDirection(unscrewed_pose, pushUp);
          taskInterface->Robot_Task->rtde_control->moveL(pushedUp_pose, initialVelocity, acceleration, false);
          taskInterface->Robot_Task->rtde_control->stopL(deceleration);    
          unscrewed = true;
          // Get the current timestamp
          std::time_t now = std::time(nullptr);
          struct std::tm localTime;
          localtime_r(&now, &localTime); 

          // Create a filename based on the timestamp
          char buffer[80];
          std::strftime(buffer, sizeof(buffer), "output_%Y%m%d%H%M%S.csv", &localTime);

          std::string csvFilename(buffer);

          // Directory location to save the CSV file
          std::string directory = "/home/pradip/catkin_KIT/src/disassembly-station/screw_alignment/screw_alignment_data_T25/";

          // Construct the full path for the CSV file
          std::string fullFilePath = directory + csvFilename;

          // Open the CSV file for writing
          std::ofstream csvFile(fullFilePath);

          if (csvFile.is_open()) {
              // Write the header row and data as previously shown
              csvFile << "Elapsed Time (s), Screw Head Point, Screw Head Point with Error, Error (X Direction), Error (Y Direction), Interpolated Pose, Engaged Pose \n";

              // Write the elapsed time
              csvFile << elapsedTime.count() << ",";
              
              // Write the screw head point original
              std::string screwHeadPointOriginalStr;
              for (double value : screwHeadPointOriginal) {
                  screwHeadPointOriginalStr += std::to_string(value) + ",";
              }
              // Remove the trailing comma
              if (!screwHeadPointOriginalStr.empty()) {
                  screwHeadPointOriginalStr.pop_back();
              }
              csvFile << "\"" << screwHeadPointOriginalStr << "\",";

              // Write the screw head point with error
              std::string screwHeadPointStr;
              for (double value : screwHeadPoint) {
                  screwHeadPointStr += std::to_string(value) + ",";
              }
              // Remove the trailing comma
              if (!screwHeadPointStr.empty()) {
                  screwHeadPointStr.pop_back();
              }
              csvFile << "\"" << screwHeadPointStr << "\",";

              // Error in X and Y
              csvFile << errorScrewHead[0] << "," << errorScrewHead[1] << ",";

              // Write the interpolated poses
              std::string interpolationStr;
              for (double value : interpolatedPoses[p]) {
                  interpolationStr += std::to_string(value) + ",";
              }
              // Remove the trailing comma
              if (!interpolationStr.empty()) {
                  interpolationStr.pop_back();
              }
              csvFile << "\"" << interpolationStr << "\",";

              // Write the engages screw pose
              std::string engagedPoseStr;
              for (double value : pushed_pose) {
                  engagedPoseStr += std::to_string(value) + ",";
              }
              // Remove the trailing comma
              if (!engagedPoseStr.empty()) {
                  engagedPoseStr.pop_back();
              }
              csvFile << "\"" << engagedPoseStr << "\"\n";

              csvFile.close(); // You can omit this part if you don't need to explicitly close the file.
          } else {
            std::cerr << "Failed to save the CSV file at the specified location." << std::endl;
          }
        }
      if (unscrewed) {
          return 1;
      }  
      break;
      }
    }
    // Reset the flag
    taskInterface->Drill_Task->momentSuccessChanged = false;
    }
    // Get the end time of the fastening action and calculate the elapsed fastening time
    auto endfastenTime = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedFasteningTime;
    elapsedFasteningTime = endfastenTime - startfastenTime;
    // If the elapsed fastening time is greater than 13 seconds and the drill is ready, trigger the reset action
    if (elapsedFasteningTime.count() >= fastenTimer && !engaged){
      taskInterface->Drill_Task->triggerProg(RESET); 
      // Reset the startfastenTime to the current time
      startfastenTime = std::chrono::high_resolution_clock::now();
      std::cout << "fastentime: " << elapsedFasteningTime.count() << std::endl;
    }  
  }
}



int Spiralsuche::testScrewAlignment_torx(double distanceTrajectory, TaskInterface* taskInterface) {
  // Initialize the force/torque sensor
  taskInterface->Robot_Task->rtde_control->zeroFtSensor();
  
  // Define constants for readability
  const double initialVelocity = 0.095;
  const double spiralVelocity = 0.015;
  const double searchRadius = 0.004;
  const double maxForce = taskInterface->strength_max_;
  const double minForce = taskInterface->strength_min_;
  const double forceThreshold = 4; // Adjust as needed
  double acceleration = taskInterface->acceleration_;
  double velocity = taskInterface->velocity_;
  double deceleration = 10;
  double angleIncrement = 0.001; 

  // Move to the screw head
  //screw 1
  //std::vector<double> screwHeadPoint = {-0.17385, -0.52939, 0.33346, 1.146, -1.291, -1.243};
  //std::vector<double> screwHeadPoint = {-0.17285, -0.52839, 0.33346, 1.146, -1.291, -1.243};
  //screw 2
  //std::vector<double> screwHeadPoint = {-0.16541, -0.46529, 0.33346, 1.146, -1.291, -1.243};
  //std::vector<double> screwHeadPoint = {-0.16752, -0.46183, 0.33346, 1.146, -1.291, -1.243};
  //std::vector<double> screwHeadPoint = {-0.17052, -0.46327, 0.33346, 1.146, -1.291, -1.243};

  //torx
  //screw 1
  //std::vector<double> screwHeadPoint = {-0.24137, -0.60356, 0.22016, 1.199, -1.274, -1.191}; //original center
  std::vector<double> screwHeadPoint = {-0.2424, -0.6046, 0.22016, 1.199, -1.274, -1.191}; //with error

  //std::vector<double> screwHeadPoint = {-0.18242, -0.58908, 0.22024, 1.199, -1.274, -1.191};
  //std::vector<double> screwHeadPoint = {-0.183, -0.5895, 0.22024, 1.199, -1.274, -1.191}; //with error



  taskInterface->Robot_Task->rtde_control->moveL(screwHeadPoint, initialVelocity, acceleration, false);
  std::cout << "Screw point reached" << std::endl;

  std::vector<double> toolSpeed = {0, 0, -0.005, 0, 0, 0};
  taskInterface->Robot_Task->rtde_control->moveUntilContact(toolSpeed);


  std::vector<double> contact_pose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
  std::cout << "contactPose" << std::endl;
  for (double value : contact_pose){
                std::cout << value << " ";
            }
  std::cout << std::endl;
  std::vector<double> co_po = contact_pose;
  double zOffset = -0.0001; // 1 mm in meters
  co_po[0] += zOffset;
  co_po[1] += zOffset;
  co_po[2] += zOffset;
  std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();
  std::cout << "refTCPForce" << std::endl;
  for (double value : refTCPForce){
                std::cout << value << " ";
            }
  std::cout << std::endl;

  // Perform the spiral search
  std::vector<std::vector<double>> interpolatedPoses = taskInterface->Spiral->interpolateSpiral(angleIncrement, distanceTrajectory, searchRadius, co_po);
  auto startPlanningTime = std::chrono::system_clock::now();

  bool engaged = false;
  auto startfastenTime = std::chrono::system_clock::now();
  for (int p = 0; p < interpolatedPoses.size(); p++) {
    if (taskInterface->Drill_Task->isDrillReady() && !engaged) {
      taskInterface->Drill_Task->triggerProg(FASTEN);
    }
    
    std::cout << "Step " << p << std::endl;

    taskInterface->Robot_Task->rtde_control->moveL(interpolatedPoses[p], spiralVelocity, acceleration, false);
    //taskInterface->Robot_Task->rtde_control->stopL(deceleration);

    //std::vector<double> pushPose = interpolatedPoses[p];
    //double zOffset = -0.001; // 1 mm in meters

    // Update the Z-axis coordinate of the pose
    //pushPose[2] += zOffset;

    //taskInterface->Robot_Task->rtde_control->moveL(co_po, spiralVelocity, acceleration, false);
    //taskInterface->Robot_Task->rtde_control->stopL(deceleration);    

    std::vector<double> TCPForce = taskInterface->Robot_Task->getToolFrameForce();
  
    if (TCPForce.at(1) <= taskInterface->strength_max_ &&
            TCPForce.at(1) >= taskInterface->strength_min_)
        { 
          std::cout << "force between strengthMax and strengthMin " << std::endl;
          // push 5mm down 
          for (double value : TCPForce){
                std::cout << value << " ";
            }
        }
    else
        {  
          std::cout << "force not in range" << std::endl;
          for (double value : TCPForce){
              std::cout << value << " ";
          }
        }

      // Check for a breaking condition
    if ((TCPForce.at(1) < 4 && TCPForce.at(0) < forceThreshold) &&
    (TCPForce.at(1) < 4 && TCPForce.at(2) < forceThreshold)){
      std::cout << "Force in favor of loosening screw" << std::endl;
      std::cout << "Force values: ";
      for (double value : TCPForce) {
        std::cout << value << " ";
      }
      std::cout << std::endl;
      // Trigger a loosening action
      std::vector<double> engaged_pose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
      std::vector<double> adjust_pose = engaged_pose;
      double pushdown = -0.003; // 2 mm in meters
      adjust_pose[2] += pushdown;
      taskInterface->Robot_Task->rtde_control->moveL(adjust_pose, initialVelocity, acceleration, false);
      taskInterface->Robot_Task->rtde_control->stopL(deceleration);    
      

      if (taskInterface->Drill_Task->isDrillReady()) {
        //taskInterface->Drill_Task->triggerProg(RESET); 
        taskInterface->unscrew();
        //taskInterface->Drill_Task->triggerProg(LOSEN);
        ros::Duration(1.0).sleep(); 
        engaged = true;
      }

      auto endPlanningTime = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsedSecondsPlan;
      elapsedSecondsPlan = endPlanningTime - startPlanningTime;
      std::cout << "Time elapsed: " << elapsedSecondsPlan.count() << " seconds" << std::endl;

      if (engaged == true) {
        std::vector<double> engaged_pose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
        std::vector<double> unscrewed_pose = engaged_pose;
        double offset = 0.015; // 15 mm in meters
        unscrewed_pose[2] += offset;
        taskInterface->Robot_Task->rtde_control->moveL(unscrewed_pose, velocity, acceleration, false);
        taskInterface->Robot_Task->rtde_control->stopL(deceleration);    
      }
      
      break;
    }
    auto endfastenTime = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedFasteningTime;
    elapsedFasteningTime = endfastenTime - startfastenTime;
    if (elapsedFasteningTime.count() >= 13 && taskInterface->Drill_Task->isDrillReady()){
      taskInterface->Drill_Task->triggerProg(RESET); 
    }
    }
  return 1;
  }


  

int Spiralsuche::executeSpiralSearch(double distanceTrajectory, TaskInterface* taskInterface) {

  taskInterface->Robot_Task->rtde_control->zeroFtSensor();
  
  //distance_trajectory_   = distanceTrajectory;
  distance_steps_        = distanceTrajectory;
  double velocity     = taskInterface->velocity_;
  double acceleration = taskInterface->acceleration_;
  double velocity_init     = 0.095;
  double angleIncrement = 0.001; 
  double distance_trajectory_ = distanceTrajectory;
  double radiusSearch = 0.002;
  std::vector<double> screw_head_point = {-0.20896, -0.50489, 0.33346, 1.162, -1.268, -1.208};
  taskInterface->Robot_Task->rtde_control->moveL(screw_head_point, velocity_init, acceleration, false);
  std::cout << "screw_point reached" << std::endl;
  for (double value : screw_head_point){
                std::cout << value << " ";
            }
  std::cout << std::endl;
  //taskInterface->Drill_Task->triggerProg(LOSEN);
  //taskInterface->Drill_Task->triggerProg(FASTEN);
  //taskInterface->Drill_Task->triggerStart();
  //taskInterface->Robot_Task->rtde_control->stopL(10);
  std::vector<double> toolSpeed = {0, 0, -0.005, 0, 0, 0};
  taskInterface->Robot_Task->rtde_control->moveUntilContact(toolSpeed);

  std::vector<double> contact_pose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
  
  std::vector<std::vector<double>> interpolatedPoses = taskInterface->Spiral->interpolateSpiral(angleIncrement, distance_trajectory_, radiusSearch, contact_pose);

  std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();
  std::cout << "refTCPForce" << std::endl;
  for (double value : refTCPForce){
                std::cout << value << " ";
            }
  std::cout << std::endl;

  std::vector<double> contactPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
  std::cout << "contactPose" << std::endl;
  for (double value : contactPose){
                std::cout << value << " ";
            }
  std::cout << std::endl;

  bool movedUp       = false;
  int blockForcemode = 0;
  for (int p = 0; p < interpolatedPoses.size(); p++)
  {
    // ROS_INFO_STREAM((double)h/path_.size() << " %");

    std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();
    
    std::vector<double> TCPForce = refTCPForce;
    bool targetNewWp             = false;
    int progress;
    do
    { 
      std::cout << "entered 1 checkpoint" << std::endl;
      double angle, velocity, acceleration;
      acceleration        = taskInterface->acceleration_;
      double deceleration = 10;
      double strengthMid  = taskInterface->strength_min_ +
                           (taskInterface->strength_max_ - taskInterface->strength_min_) / 2;

      if (movedUp)
      {
        std::cout << "entered 2 checkpoint" << std::endl;
        angle        = 0;
        velocity     = taskInterface->velocity_;
        acceleration = taskInterface->acceleration_;
      }
      else
      {
       
        if (TCPForce.at(1) <= taskInterface->strength_max_ &&
            TCPForce.at(1) >= taskInterface->strength_min_)
        { 
          std::cout << "entered 3 checkpoint" << std::endl;
          angle        = 0;
          velocity     = taskInterface->velocity_;
          acceleration = taskInterface->acceleration_;
        }
        else
        {  
          std::cout << "entered 4 checkpoint" << std::endl;
          refTCPForce.at(1) = TCPForce.at(1);

          double strengthRange = 4;
          double maxAngle      = 45;
          if (TCPForce.at(1) > taskInterface->strength_max_)
          {
            std::cout << "entered 5 checkpoint" << std::endl;
            double percentage = ((TCPForce.at(1) - taskInterface->strength_max_) / strengthRange);
            angle             = percentage * maxAngle;
            if (angle > maxAngle)
            {
              angle        = 89.999;
              velocity     = 0.004;
              acceleration = 0.002;
            }
          }
          else if (TCPForce.at(1) < taskInterface->strength_min_)
          {
            std::cout << "entered 6 checkpoint" << std::endl;
            double percentage = ((taskInterface->strength_min_ - TCPForce.at(1)) / strengthRange);
            angle             = -percentage * maxAngle;
            if (angle < maxAngle)
            {
              angle        = -89.999;
              velocity     = 0.004;
              acceleration = 0.002;
            }

            blockForcemode = 0;
          }

          velocity = taskInterface->velocity_ / cos(abs(angle) * (PI / 180));
          if (velocity > 0.004)
            velocity = 0.004;
        }
      }
      // ROS_ERROR_STREAM("Angel: " << angle << "\nAcceleration: " << acceleration << "\nVelocity: "
      // << velocity << "\nDeceleration: " << deceleration << "\nForce: " << TCPForce.at(1));

      std::vector<double> currentPose;
      std::vector<double> newWp;
      bool isWrenchLimit;

      do
      {
        std::cout << "entered 7 checkpoint" << std::endl;
        std::vector<double> pathVecCurrentPose =
          taskInterface->Robot_Task->TransformBaseToToolFrame(
            {interpolatedPoses[p][0], interpolatedPoses[p][1], interpolatedPoses[p][2]});
        double pathVecCurrentPoseMag =
          sqrt(pow(pathVecCurrentPose.at(0), 2) + pow(pathVecCurrentPose.at(2), 2));
        double wpOffset = pathVecCurrentPoseMag * tan(angle * (PI / 180));

        currentPose              = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
        pathVecCurrentPose.at(1) = wpOffset;
        newWp = taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, pathVecCurrentPose);
        taskInterface->Robot_Task->rtde_control->moveL(newWp, velocity, acceleration, true);


        progress    = taskInterface->Robot_Task->rtde_control->getAsyncOperationProgress();
        targetNewWp = false;
        while (movedUp && (progress >= 0))
        {
          std::cout << "entered 8 checkpoint" << std::endl;
          progress = taskInterface->Robot_Task->rtde_control->getAsyncOperationProgress();
          if (progress < 0 && p < interpolatedPoses.size() - 1)
          {std::cout << "entered 9 checkpoint" << std::endl;
            p++;
            targetNewWp = true;
          }

          TCPForce      = taskInterface->Robot_Task->getToolFrameForce();
          isWrenchLimit = abs(sqrt(pow(TCPForce.at(0), 2) + pow(TCPForce.at(2), 2))) > 5;
          std::cout << " isWrenchLimit: " << isWrenchLimit << std::endl;
          if (isWrenchLimit)
          {
            targetNewWp = false;
            break;
          }
        }
        if (movedUp)
        {
          taskInterface->Robot_Task->rtde_control->stopL(deceleration);
        }
      } while (targetNewWp);
      movedUp = false;

      bool adjustAngle = false;
      do
      { 
        std::cout << "entered 10 checkpoint" << std::endl;
        bool checkWhileForcemode = false;
        std::chrono::system_clock::time_point startCheckLateralTimer;
        do
        {  
          std::cout << "entered 11 checkpoint" << std::endl;
          TCPForce      = taskInterface->Robot_Task->getToolFrameForce();
          isWrenchLimit = abs(sqrt(pow(TCPForce.at(0), 2) + pow(TCPForce.at(2), 2))) > 5;
          // ROS_INFO_STREAM(isWrenchLimit);
          // ROS_INFO_STREAM(abs(sqrt(pow(TCPForce.at(0), 2) + pow(TCPForce.at(2), 2))));
          if (isWrenchLimit && !movedUp && blockForcemode == 0)
          { std::cout << "entered 12 checkpoint" << std::endl;
            taskInterface->Robot_Task->rtde_control->stopL();
            currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
            taskInterface->Robot_Task->startForcemode(WRENCH_DOWN, 8);
            startCheckLateralTimer = std::chrono::system_clock::now();
            checkWhileForcemode    = true;
            blockForcemode++;
          }

          do
          {
            if (taskInterface->Drill_Task->isDrillReady())
            { std::cout << "entered 13 checkpoint" << std::endl;
              std::cout << "entered drill mode" << std::endl;
              taskInterface->Robot_Task->rtde_control->stopL(50);
                //taskInterface->Drill_Task->triggerProg(FASTEN);
                taskInterface->Drill_Task->triggerProg(LOSEN);
                //taskInterface->Drill_Task->triggerStart();

                if (!checkWhileForcemode)
                {
                  std::cout << "entered force mode" << std::endl;
                  taskInterface->Robot_Task->rtde_control->moveL(
                    newWp, velocity, acceleration, true);
                }
              }
              else
              {
                taskInterface->Robot_Task->rtde_control->stopL();
                taskInterface->Robot_Task->rtde_control->forceModeStop();
                // taskInterface->unscrew();

                ROS_INFO_STREAM("SUCCESS: srcew found");
                return 0;
              }
            }
            
            while (checkWhileForcemode);

          if (isWrenchLimit && !movedUp)
          {
            taskInterface->Robot_Task->rtde_control->stopL();

            currentPose               = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
            std::vector<double> upPos = taskInterface->Robot_Task->rtde_control->poseTrans(
              currentPose, {0, 0.05, 0, 0, 0, 0});
            taskInterface->Robot_Task->rtde_control->moveL(upPos, 0.01, 0.015, true);
            movedUp     = true;
            adjustAngle = true;
          }
          if (!isWrenchLimit && movedUp)
          {
            taskInterface->Robot_Task->rtde_control->stopL();
            currentPose               = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
            std::vector<double> upPos = taskInterface->Robot_Task->rtde_control->poseTrans(
              currentPose, {0, 0.001, 0, 0, 0, 0});
            taskInterface->Robot_Task->rtde_control->moveL(upPos, 0.01, 0.015, false);
          }

        } while (isWrenchLimit);

        double currentDepth    = taskInterface->Robot_Task->calcDepth(contactPose);
        bool isMaxDepthReached = abs(currentDepth) > (taskInterface->abort_at_depth_ + 0.001);
        if (isMaxDepthReached)
        {
          taskInterface->Robot_Task->rtde_control->stopL();
          ROS_ERROR_STREAM("ERROR: Slipped of Screw!");
          return 2;
        }


        progress = taskInterface->Robot_Task->rtde_control->getAsyncOperationProgress();

        if (!movedUp)
        {
          if (angle > 0)
          {
            adjustAngle = TCPForce.at(1) < strengthMid;
            if (TCPForce.at(1) > refTCPForce.at(1))
            {
              adjustAngle       = true;
              refTCPForce.at(1) = TCPForce.at(1);
            }
          }
          else if (angle < 0)
          {
            adjustAngle = TCPForce.at(1) > strengthMid;
            if (TCPForce.at(1) < refTCPForce.at(1))
            {
              adjustAngle       = true;
              refTCPForce.at(1) = TCPForce.at(1);
            }
          }
          else
            adjustAngle = TCPForce.at(1) > taskInterface->strength_max_ ||
                          TCPForce.at(1) < taskInterface->strength_min_;
        }
      } while (progress >= 0 && !adjustAngle);
      taskInterface->Robot_Task->rtde_control->stopL(deceleration);

      if (progress < 0)
        targetNewWp = true;
    } while (!targetNewWp);
  }
  taskInterface->Robot_Task->rtde_control->stopL();

  ROS_ERROR_STREAM("ERROR: screw not found!");
  return 1;
}











int Spiralsuche::execute(double distanceTrajectory, TaskInterface* taskInterface)
{
  taskInterface->Robot_Task->rtde_control->zeroFtSensor();

  auto startPlanningTime = std::chrono::system_clock::now();
  distance_trajectory_   = distanceTrajectory;
  if (plan(taskInterface) == 1)
    return 1;
  auto endPlanningTime = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsedSecondsPlan;
  elapsedSecondsPlan = endPlanningTime - startPlanningTime;
  taskInterface->planningTime_.push_back((double)elapsedSecondsPlan.count());

  taskInterface->Drill_Task->triggerProg(FASTEN);
  taskInterface->Drill_Task->triggerStart();
  auto startScrewingTime = std::chrono::system_clock::now();


  std::vector<double> toolSpeed = {0, 0, -0.005, 0, 0, 0};
  taskInterface->Robot_Task->rtde_control->moveUntilContact(toolSpeed);
  std::vector<double> contactPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();

  for (int h = 0; h < path_.size(); h++)
  {
    // ROS_INFO_STREAM((double)h/path_.size() << " %");

    // strength_max_= 7, strength_min_ = 4, double velocity = 0.015; double abortAtDepth = 0.005;

    std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();

    std::vector<double> TCPForce = refTCPForce;
    bool targetNewWp             = false;
    do
    {
      double angle, velocity, acceleration;
      double deceleration = 5;
      acceleration        = taskInterface->acceleration_;
      if (TCPForce.at(1) <= taskInterface->strength_max_ &&           
          TCPForce.at(1) >= taskInterface->strength_min_)
      {
        angle    = 0;
        velocity = taskInterface->velocity_;
      }
      else
      {
        refTCPForce.at(1) = TCPForce.at(1);

        double strengthRange = 4;
        double maxAngle      = 45;
        if (TCPForce.at(1) > taskInterface->strength_max_)
        {
          double percentage = ((TCPForce.at(1) - taskInterface->strength_max_) / strengthRange);
          angle             = percentage * maxAngle;
          if (angle > maxAngle)
          {
            angle        = 89.999;
            velocity     = 0.004;
            acceleration = 0.002;
          }
        }
        else if (TCPForce.at(1) < taskInterface->strength_min_)
        {
          double percentage = ((taskInterface->strength_min_ - TCPForce.at(1)) / strengthRange);
          angle             = -percentage * maxAngle;
          if (angle < maxAngle)
          {
            angle        = -89.999;
            velocity     = 0.004;
            acceleration = 0.002;
          }
        }

        velocity = taskInterface->velocity_ / cos(abs(angle) * (PI / 180));
        if (velocity > 0.004)
          velocity = 0.004;
      }

      std::vector<double> pathVecCurrentPose = taskInterface->Robot_Task->TransformBaseToToolFrame(
        {path_[h][0], path_[h][1], path_[h][2]});
      double pathVecCurrentPoseMag =
        sqrt(pow(pathVecCurrentPose.at(0), 2) + pow(pathVecCurrentPose.at(2), 2));
      double wpOffset = pathVecCurrentPoseMag * tan(angle * (PI / 180));

      std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
      pathVecCurrentPose.at(1)        = wpOffset;
      std::vector<double> newWp =
        taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, pathVecCurrentPose);
      taskInterface->Robot_Task->rtde_control->moveL(newWp, velocity, acceleration, true);

      bool adjustAngle = false;
      int progress;
      do
      {
        bool isWrenchSafetylimit = abs(sqrt(pow(TCPForce.at(0), 2) + pow(TCPForce.at(2), 2))) > 15;
        double currentDepth      = taskInterface->Robot_Task->calcDepth(contactPose);
        bool isMaxDepthReached   = abs(currentDepth) > (taskInterface->abort_at_depth_ + 0.001);
        if (taskInterface->Drill_Task->isDrillReady())
        {
          taskInterface->Robot_Task->rtde_control->stopL(50);

          auto endScrewingTime = std::chrono::system_clock::now();
          std::chrono::duration<double> elapsedSecondsScrewing;
          elapsedSecondsScrewing = endScrewingTime - startScrewingTime;
          if ((double)elapsedSecondsScrewing.count() > 2)
          {
            taskInterface->Drill_Task->triggerProg(FASTEN);
            taskInterface->Drill_Task->triggerStart();
            startScrewingTime = std::chrono::system_clock::now();
            break;
          }
          else
          {
            // taskInterface->unscrew();

            ROS_INFO_STREAM("SUCCESS: srcew found");
            return 0;
          }
        }
        else if (isMaxDepthReached || isWrenchSafetylimit)
        {
          taskInterface->Robot_Task->rtde_control->stopL();
          if (isMaxDepthReached)
          {
            ROS_ERROR_STREAM("ERROR: Slipped of Screw!");
            return 2;
          }
          else if (isWrenchSafetylimit)
          {
            ROS_ERROR_STREAM("ERROR: Wrench to high, aborting ...!");
            return 3;
          }
        }


        progress = taskInterface->Robot_Task->rtde_control->getAsyncOperationProgress();

        TCPForce = taskInterface->Robot_Task->getToolFrameForce();

        double strengthMid = taskInterface->strength_min_ +
                             (taskInterface->strength_max_ - taskInterface->strength_min_) / 2;
        if (angle > 0)
        {
          adjustAngle = TCPForce.at(1) < strengthMid;
          if (TCPForce.at(1) > refTCPForce.at(1))
          {
            adjustAngle       = true;
            refTCPForce.at(1) = TCPForce.at(1);
          }
        }
        else if (angle < 0)
        {
          adjustAngle = TCPForce.at(1) > strengthMid;
          if (TCPForce.at(1) < refTCPForce.at(1))
          {
            adjustAngle       = true;
            refTCPForce.at(1) = TCPForce.at(1);
          }
        }
        else
          adjustAngle = TCPForce.at(1) > taskInterface->strength_max_ ||
                        TCPForce.at(1) < taskInterface->strength_min_;
      } while (progress >= 0 && !adjustAngle);
      taskInterface->Robot_Task->rtde_control->stopL(deceleration);

      if (progress < 0)
        targetNewWp = true;
    } while (!targetNewWp);
  }
  taskInterface->Robot_Task->rtde_control->stopL();

  ROS_ERROR_STREAM("ERROR: screw not found!");
  return 1;
}

int Spiralsuche::executeQuadrantSearch(double distanceTrajectory,
                                       double offset,
                                       TaskInterface* taskInterface)
{
  double sumPlaningTime = 0;
  int returnValue       = -1;
  int counter           = 0;

  std::vector<double> startPos = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();

  do
  {
    returnValue = -1;
    taskInterface->Robot_Task->rtde_control->zeroFtSensor();

    auto startPlanningTime = std::chrono::system_clock::now();
    distance_trajectory_   = distanceTrajectory;
    if (plan(taskInterface) == 1)
      return 1;
    auto endPlanningTime = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSecondsPlan;
    elapsedSecondsPlan = endPlanningTime - startPlanningTime;
    sumPlaningTime     = sumPlaningTime + (double)elapsedSecondsPlan.count();

    taskInterface->Drill_Task->triggerProg(FASTEN);
    taskInterface->Drill_Task->triggerStart();
    auto startScrewingTime = std::chrono::system_clock::now();


    std::vector<double> toolSpeed = {0, 0, -0.005, 0, 0, 0};
    taskInterface->Robot_Task->rtde_control->moveUntilContact(toolSpeed);
    std::vector<double> contactPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();

    for (int h = 0; h < path_.size(); h++)
    {
      // ROS_INFO_STREAM((double)h/path_.size() << " %");

      std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();

      std::vector<double> TCPForce = refTCPForce;
      bool targetNewWp             = false;
      do
      {
        double angle, velocity, acceleration;
        double deceleration = 5;
        acceleration        = taskInterface->acceleration_;
        if (TCPForce.at(1) <= taskInterface->strength_max_ &&
            TCPForce.at(1) >= taskInterface->strength_min_)
        {
          angle    = 0;
          velocity = taskInterface->velocity_;
        }
        else
        {
          refTCPForce.at(1) = TCPForce.at(1);

          double strengthRange = 4;
          double maxAngle      = 45;
          if (TCPForce.at(1) > taskInterface->strength_max_)
          {
            double percentage = ((TCPForce.at(1) - taskInterface->strength_max_) / strengthRange);
            angle             = percentage * maxAngle;
            if (angle > maxAngle)
            {
              angle        = 89.999;
              velocity     = 0.004;
              acceleration = 0.002;
            }
          }
          else if (TCPForce.at(1) < taskInterface->strength_min_)
          {
            double percentage = ((taskInterface->strength_min_ - TCPForce.at(1)) / strengthRange);
            angle             = -percentage * maxAngle;
            if (angle < maxAngle)
            {
              angle        = -89.999;
              velocity     = 0.004;
              acceleration = 0.002;
            }
          }

          velocity = taskInterface->velocity_ / cos(abs(angle) * (PI / 180));
          if (velocity > 0.004)
            velocity = 0.004;
        }

        std::vector<double> pathVecCurrentPose =
          taskInterface->Robot_Task->TransformBaseToToolFrame(
            {path_[h][0], path_[h][1], path_[h][2]});
        double pathVecCurrentPoseMag =
          sqrt(pow(pathVecCurrentPose.at(0), 2) + pow(pathVecCurrentPose.at(2), 2));
        double wpOffset = pathVecCurrentPoseMag * tan(angle * (PI / 180));

        std::vector<double> currentPose =
          taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
        pathVecCurrentPose.at(1) = wpOffset;
        std::vector<double> newWp =
          taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, pathVecCurrentPose);
        taskInterface->Robot_Task->rtde_control->moveL(newWp, velocity, acceleration, true);

        bool adjustAngle = false;
        int progress;
        do
        {
          double isWrenchSafetylimit =
            abs(sqrt(pow(TCPForce.at(0), 2) + pow(TCPForce.at(2), 2))) > 15;
          double currentDepth    = taskInterface->Robot_Task->calcDepth(contactPose);
          bool isMaxDepthReached = abs(currentDepth) > (taskInterface->abort_at_depth_ + 0.001);
          if (taskInterface->Drill_Task->isDrillReady())
          {
            taskInterface->Robot_Task->rtde_control->stopL(50);

            auto endScrewingTime = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsedSecondsScrewing;
            elapsedSecondsScrewing = endScrewingTime - startScrewingTime;
            if ((double)elapsedSecondsScrewing.count() > 2)
            {
              taskInterface->Drill_Task->triggerProg(FASTEN);
              taskInterface->Drill_Task->triggerStart();
              startScrewingTime = std::chrono::system_clock::now();
              break;
            }
            else
            {
              // taskInterface->unscrew();

              ROS_INFO_STREAM("SUCCESS: srcew found");
              returnValue = 0;

              h           = path_.size();
              targetNewWp = true;
              break;
            }
          }
          else if (isMaxDepthReached || isWrenchSafetylimit)
          {
            if (isMaxDepthReached)
            {
              ROS_ERROR_STREAM("ERROR: Slipped of Screw!");
              returnValue = 2;
            }
            else if (isWrenchSafetylimit)
            {
              ROS_ERROR_STREAM("ERROR: Wrench to high, aborting ...!");
              returnValue = 3;
            }
            h           = path_.size();
            targetNewWp = true;
            break;
          }


          progress = taskInterface->Robot_Task->rtde_control->getAsyncOperationProgress();

          TCPForce = taskInterface->Robot_Task->getToolFrameForce();

          double strengthMid = taskInterface->strength_min_ +
                               (taskInterface->strength_max_ - taskInterface->strength_min_) / 2;
          if (angle > 0)
          {
            adjustAngle = TCPForce.at(1) < strengthMid;
            if (TCPForce.at(1) > refTCPForce.at(1))
            {
              adjustAngle       = true;
              refTCPForce.at(1) = TCPForce.at(1);
            }
          }
          else if (angle < 0)
          {
            adjustAngle = TCPForce.at(1) > strengthMid;
            if (TCPForce.at(1) < refTCPForce.at(1))
            {
              adjustAngle       = true;
              refTCPForce.at(1) = TCPForce.at(1);
            }
          }
          else
            adjustAngle = TCPForce.at(1) > taskInterface->strength_max_ ||
                          TCPForce.at(1) < taskInterface->strength_min_;
        } while (progress >= 0 && !adjustAngle);
        taskInterface->Robot_Task->rtde_control->stopL(deceleration);

        if (progress < 0)
          targetNewWp = true;
      } while (!targetNewWp);
    }
    taskInterface->Robot_Task->rtde_control->stopL();

    if (returnValue == -1)
    {
      ROS_ERROR_STREAM("ERROR: screw not found!");
      returnValue = 1;
    }

    if (returnValue != 0 && counter < 4)
    {
      std::vector<double> upPos = taskInterface->Robot_Task->rtde_control->poseTrans(
        taskInterface->Robot_Task->rtde_receive->getActualTCPPose(), {0, 0.02, 0, 0, 0, 0});
      taskInterface->Robot_Task->gotoTP(upPos, false, 0.05, 0.02);
      taskInterface->Robot_Task->gotoTP(startPos, false, 0.05, 0.02);

      if (counter == 0)
      {
        std::vector<double> currentPose =
          taskInterface->Robot_Task->rtde_control->poseTrans(startPos, {offset, 0, 0, 0, 0, 0});
        taskInterface->Robot_Task->gotoTP(currentPose, false, 0.05, 0.02);
      }
      else if (counter == 1)
      {
        std::vector<double> currentPose =
          taskInterface->Robot_Task->rtde_control->poseTrans(startPos, {0, 0, offset, 0, 0, 0});
        taskInterface->Robot_Task->gotoTP(currentPose, false, 0.05, 0.02);
      }
      else if (counter == 2)
      {
        std::vector<double> currentPose =
          taskInterface->Robot_Task->rtde_control->poseTrans(startPos, {-offset, 0, 0, 0, 0, 0});
        taskInterface->Robot_Task->gotoTP(currentPose, false, 0.05, 0.02);
      }
      else if (counter == 3)
      {
        std::vector<double> currentPose =
          taskInterface->Robot_Task->rtde_control->poseTrans(startPos, {0, 0, -offset, 0, 0, 0});
        taskInterface->Robot_Task->gotoTP(currentPose, false, 0.05, 0.02);
      }
    }

    // ROS_INFO_STREAM(counter + 1 << " / " << 4 << " executed");

    counter++;
  } while (counter < 5 && returnValue != 0);

  taskInterface->planningTime_.push_back(sumPlaningTime);
  return returnValue;
}

int Spiralsuche::executeWithLateralControl(double distanceTrajectory, TaskInterface* taskInterface)
{
  taskInterface->Robot_Task->rtde_control->zeroFtSensor();
  double initialvelocity     = 0.095;
  double acceleration = taskInterface->acceleration_;
  std::vector<double> screwHeadPoint = {-0.16541, -0.46529, 0.30346, 1.146, -1.291, -1.243};
  //std::vector<double> screwHeadPoint = {-0.16752, -0.46183, 0.33346, 1.146, -1.291, -1.243};
  taskInterface->Robot_Task->rtde_control->moveL(screwHeadPoint, initialvelocity, acceleration, false);
  std::cout << "Screw point reached" << std::endl;

  auto startPlanningTime = std::chrono::system_clock::now();
  distance_trajectory_   = distanceTrajectory;
  distance_steps_        = distanceTrajectory;
  if (plan(taskInterface) == 1)
    return 1;
  auto endPlanningTime = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsedSecondsPlan;
  elapsedSecondsPlan = endPlanningTime - startPlanningTime;
  taskInterface->planningTime_.push_back((double)elapsedSecondsPlan.count());

  taskInterface->Drill_Task->triggerProg(FASTEN);
  taskInterface->Drill_Task->triggerStart();
  auto startScrewingTime = std::chrono::system_clock::now();


  std::vector<double> toolSpeed = {0, 0, -0.005, 0, 0, 0};
  taskInterface->Robot_Task->rtde_control->moveUntilContact(toolSpeed);
  std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();
  for (double value : refTCPForce){
                std::cout << value << " ";
            }
  std::vector<double> contactPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();


  bool movedUp       = false;
  int blockForcemode = 0;
  for (int h = 0; h < path_.size(); h++)
  {
    // ROS_INFO_STREAM((double)h/path_.size() << " %");

    std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();

    std::vector<double> TCPForce = refTCPForce;
    bool targetNewWp             = false;
    int progress;
    do
    {
      double angle, velocity, acceleration;
      acceleration        = taskInterface->acceleration_;
      double deceleration = 10;
      double strengthMid  = taskInterface->strength_min_ +
                           (taskInterface->strength_max_ - taskInterface->strength_min_) / 2;

      if (movedUp)
      {
        angle        = 0;
        velocity     = taskInterface->velocity_;
        acceleration = taskInterface->acceleration_;
      }
      else
      {
        if (TCPForce.at(1) <= taskInterface->strength_max_ &&
            TCPForce.at(1) >= taskInterface->strength_min_)
        {
          angle        = 0;
          velocity     = taskInterface->velocity_;
          acceleration = taskInterface->acceleration_;
        }
        else
        {
          refTCPForce.at(1) = TCPForce.at(1);

          double strengthRange = 4;
          double maxAngle      = 45;
          if (TCPForce.at(1) > taskInterface->strength_max_)
          {
            double percentage = ((TCPForce.at(1) - taskInterface->strength_max_) / strengthRange);
            angle             = percentage * maxAngle;
            if (angle > maxAngle)
            {
              angle        = 89.999;
              velocity     = 0.004;
              acceleration = 0.002;
            }
          }
          else if (TCPForce.at(1) < taskInterface->strength_min_)
          {
            double percentage = ((taskInterface->strength_min_ - TCPForce.at(1)) / strengthRange);
            angle             = -percentage * maxAngle;
            if (angle < maxAngle)
            {
              angle        = -89.999;
              velocity     = 0.004;
              acceleration = 0.002;
            }

            blockForcemode = 0;
          }

          velocity = taskInterface->velocity_ / cos(abs(angle) * (PI / 180));
          if (velocity > 0.004)
            velocity = 0.004;
        }
      }
      // ROS_ERROR_STREAM("Angel: " << angle << "\nAcceleration: " << acceleration << "\nVelocity: "
      // << velocity << "\nDeceleration: " << deceleration << "\nForce: " << TCPForce.at(1));

      std::vector<double> currentPose;
      std::vector<double> newWp;
      bool isWrenchLimit;

      auto startPauseTimer       = std::chrono::system_clock::now();
      double elapsedSecondsPause = 0;
      do
      {
        std::vector<double> pathVecCurrentPose =
          taskInterface->Robot_Task->TransformBaseToToolFrame(
            {path_[h][0], path_[h][1], path_[h][2]});
        double pathVecCurrentPoseMag =
          sqrt(pow(pathVecCurrentPose.at(0), 2) + pow(pathVecCurrentPose.at(2), 2));
        double wpOffset = pathVecCurrentPoseMag * tan(angle * (PI / 180));

        currentPose              = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
        pathVecCurrentPose.at(1) = wpOffset;
        newWp = taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, pathVecCurrentPose);
        taskInterface->Robot_Task->rtde_control->moveL(newWp, velocity, acceleration, true);


        progress    = taskInterface->Robot_Task->rtde_control->getAsyncOperationProgress();
        targetNewWp = false;
        while (movedUp && (progress >= 0 && elapsedSecondsPause < 1))
        {
          auto endPauseTimer                                 = std::chrono::system_clock::now();
          std::chrono::duration<double> ChronoElapsedSeconds = endPauseTimer - startPauseTimer;
          elapsedSecondsPause                                = (double)ChronoElapsedSeconds.count();

          progress = taskInterface->Robot_Task->rtde_control->getAsyncOperationProgress();
          if (progress < 0 && h < path_.size() - 1)
          {
            h++;
            targetNewWp = true;
          }

          TCPForce      = taskInterface->Robot_Task->getToolFrameForce();
          isWrenchLimit = abs(sqrt(pow(TCPForce.at(0), 2) + pow(TCPForce.at(2), 2))) > 5;
          if (isWrenchLimit)
          {
            targetNewWp = false;
            break;
          }
        }
        if (movedUp)
        {
          taskInterface->Robot_Task->rtde_control->stopL(deceleration);
        }
      } while (targetNewWp);
      movedUp = false;

      bool adjustAngle = false;
      do
      {
        bool checkWhileForcemode = false;
        std::chrono::system_clock::time_point startCheckLateralTimer;
        do
        {
          TCPForce      = taskInterface->Robot_Task->getToolFrameForce();
          isWrenchLimit = abs(sqrt(pow(TCPForce.at(0), 2) + pow(TCPForce.at(2), 2))) > 5;
          // ROS_INFO_STREAM(isWrenchLimit);
          // ROS_INFO_STREAM(abs(sqrt(pow(TCPForce.at(0), 2) + pow(TCPForce.at(2), 2))));
          if (isWrenchLimit && !movedUp && blockForcemode == 0)
          {
            taskInterface->Robot_Task->rtde_control->stopL();
            currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
            taskInterface->Robot_Task->startForcemode(WRENCH_DOWN, 8);
            startCheckLateralTimer = std::chrono::system_clock::now();
            checkWhileForcemode    = true;
            blockForcemode++;
          }

          do
          {
            if (taskInterface->Drill_Task->isDrillReady())
            {
              taskInterface->Robot_Task->rtde_control->stopL(50);

              auto endScrewingTime = std::chrono::system_clock::now();
              std::chrono::duration<double> elapsedSecondsScrewing;
              elapsedSecondsScrewing = endScrewingTime - startScrewingTime;
              if ((double)elapsedSecondsScrewing.count() > 10)
              {
                taskInterface->Drill_Task->triggerProg(FASTEN);
                taskInterface->Drill_Task->triggerStart();

                startScrewingTime = std::chrono::system_clock::now();
                if (!checkWhileForcemode)
                {
                  taskInterface->Robot_Task->rtde_control->moveL(
                    newWp, velocity, acceleration, true);
                }
              }
              else
              {
                taskInterface->Robot_Task->rtde_control->stopL();
                taskInterface->Robot_Task->rtde_control->forceModeStop();
                // taskInterface->unscrew();

                ROS_INFO_STREAM("SUCCESS: srcew found");
                return 0;
              }
            }

            auto endCheckLateralTimer = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsedSecondsChecking =
              endCheckLateralTimer - startCheckLateralTimer;
            if (!taskInterface->Drill_Task->isDrillReady() && checkWhileForcemode &&
                (double)elapsedSecondsChecking.count() >= 1)
            {
              checkWhileForcemode = false;
              taskInterface->Robot_Task->rtde_control->forceModeStop();
              ROS_ERROR_STREAM("stopFM");
              taskInterface->Robot_Task->rtde_control->moveL(currentPose, 0.05, 0.02, false);
              taskInterface->Robot_Task->rtde_control->moveL(newWp, velocity, acceleration, true);
            }
          } while (checkWhileForcemode);

          if (isWrenchLimit && !movedUp)
          {
            taskInterface->Robot_Task->rtde_control->stopL();

            currentPose               = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
            std::vector<double> upPos = taskInterface->Robot_Task->rtde_control->poseTrans(
              currentPose, {0, 0.05, 0, 0, 0, 0});
            taskInterface->Robot_Task->rtde_control->moveL(upPos, 0.01, 0.015, true);
            movedUp     = true;
            adjustAngle = true;
          }
          if (!isWrenchLimit && movedUp)
          {
            taskInterface->Robot_Task->rtde_control->stopL();
            currentPose               = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
            std::vector<double> upPos = taskInterface->Robot_Task->rtde_control->poseTrans(
              currentPose, {0, 0.001, 0, 0, 0, 0});
            taskInterface->Robot_Task->rtde_control->moveL(upPos, 0.01, 0.015, false);
          }

        } while (isWrenchLimit);

        double currentDepth    = taskInterface->Robot_Task->calcDepth(contactPose);
        bool isMaxDepthReached = abs(currentDepth) > (taskInterface->abort_at_depth_ + 0.001);
        if (isMaxDepthReached)
        {
          taskInterface->Robot_Task->rtde_control->stopL();
          ROS_ERROR_STREAM("ERROR: Slipped of Screw!");
          return 2;
        }


        progress = taskInterface->Robot_Task->rtde_control->getAsyncOperationProgress();

        if (!movedUp)
        {
          if (angle > 0)
          {
            adjustAngle = TCPForce.at(1) < strengthMid;
            if (TCPForce.at(1) > refTCPForce.at(1))
            {
              adjustAngle       = true;
              refTCPForce.at(1) = TCPForce.at(1);
            }
          }
          else if (angle < 0)
          {
            adjustAngle = TCPForce.at(1) > strengthMid;
            if (TCPForce.at(1) < refTCPForce.at(1))
            {
              adjustAngle       = true;
              refTCPForce.at(1) = TCPForce.at(1);
            }
          }
          else
            adjustAngle = TCPForce.at(1) > taskInterface->strength_max_ ||
                          TCPForce.at(1) < taskInterface->strength_min_;
        }
      } while (progress >= 0 && !adjustAngle);
      taskInterface->Robot_Task->rtde_control->stopL(deceleration);

      if (progress < 0)
        targetNewWp = true;
    } while (!targetNewWp);
  }
  taskInterface->Robot_Task->rtde_control->stopL();

  ROS_ERROR_STREAM("ERROR: screw not found!");
  return 1;
}


int SukzessiveApproximationBrian::plan(TaskInterface* taskInterface)
{
  if (taskInterface->radius_searchfield_ < distance_steps_)
  {
    ROS_ERROR_STREAM(
      "ERROR: planning not possible radius_searchfield_ must be greater than distance_steps_");
    return 1;
  }


  int nextLocation                = 0;
  std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
  double xComponent, zComponent;
  double sumXComponent  = 0;
  double sumZComponent  = 0;
  double lastXComponent = 0;
  double lastZComponent = 0;

  std::vector<std::vector<double> > path;
  std::vector<double> xPoints, zPoints;
  srand(time(NULL));
  for (int i = 0; i < number_tries_; i++)
  {
    bool calcDirection = true;
    while (calcDirection)
    {
      nextLocation = rand() % 8 + 1;

      switch (nextLocation)
      {
        case 1:
          xComponent = distance_steps_;
          zComponent = 0;
          break;
        case 2:
          xComponent = distance_steps_ * sin(PI / 4);
          zComponent = distance_steps_ * cos(PI / 4);
          break;
        case 3:
          xComponent = 0;
          zComponent = distance_steps_;
          break;
        case 4:
          xComponent = -distance_steps_ * sin(PI / 4);
          zComponent = distance_steps_ * cos(PI / 4);
          break;
        case 5:
          xComponent = -distance_steps_;
          zComponent = 0;
          break;
        case 6:
          xComponent = -distance_steps_ * sin(PI / 4);
          zComponent = -distance_steps_ * cos(PI / 4);
          break;
        case 7:
          xComponent = 0;
          zComponent = -distance_steps_;
          break;
        case 8:
          xComponent = distance_steps_ * sin(PI / 4);
          zComponent = -distance_steps_ * cos(PI / 4);
          break;
      }

      sumXComponent             = sumXComponent + xComponent;
      sumZComponent             = sumZComponent + zComponent;
      double distanceFromCenter = sqrt(pow(sumXComponent, 2) + pow(sumZComponent, 2));

      if ((xComponent != -lastXComponent || zComponent != -lastZComponent) &&
          taskInterface->radius_searchfield_ >= distanceFromCenter)
        calcDirection = false;
      else
      {
        sumXComponent = sumXComponent - xComponent;
        sumZComponent = sumZComponent - zComponent;
      }
    }
    lastXComponent = xComponent;
    lastZComponent = zComponent;

    xPoints.push_back(sumXComponent);
    zPoints.push_back(sumZComponent);

    std::vector<double> transformationVector = {xComponent, 0, zComponent, 0, 0, 0};
    std::vector<double> waypoint =
      taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, transformationVector);
    currentPose = waypoint;
    path.push_back(waypoint);
  }
  path_ = path;

  xPoints.insert(xPoints.begin(), 0);
  zPoints.insert(zPoints.begin(), 0);
  std::vector<std::pair<std::string, std::vector<double> > > vals = {{"xValue", xPoints},
                                                                     {"zValue", zPoints}};
  write_csv("/home/nucuser/repos/wbk-effector-group/urcontroller2/plot/saBrian_search.csv", vals);

  return 0;
}

int SukzessiveApproximationBrian::execute(double distanceSteps,
                                          int numberTries,
                                          TaskInterface* taskInterface)
{
  taskInterface->Robot_Task->rtde_control->zeroFtSensor();

  distance_steps_        = distanceSteps;
  number_tries_          = numberTries;
  auto startPlanningTime = std::chrono::system_clock::now();
  if (plan(taskInterface) == 1)
    return 1;
  auto endPlanningTime = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsedSecondsPlan;
  elapsedSecondsPlan = endPlanningTime - startPlanningTime;
  taskInterface->planningTime_.push_back((double)elapsedSecondsPlan.count());

  taskInterface->Robot_Task->startForcemode(WRENCH_DOWN, taskInterface->strength_min_);
  for (int tryCount = 0; tryCount < path_.size(); tryCount++)
  {
    taskInterface->Drill_Task->triggerProg(FASTEN);
    taskInterface->Drill_Task->triggerStart();
    auto startScrewingTime = std::chrono::system_clock::now();
    taskInterface->Robot_Task->startForcemode(WRENCH_DOWN, taskInterface->strength_min_);

    std::vector<double> TCPforce;
    do
    {
      TCPforce = taskInterface->Robot_Task->rtde_receive->getActualTCPForce();
    } while (TCPforce[2] < taskInterface->strength_min_);

    auto start = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds;
    do
    {
      if (taskInterface->Drill_Task->isDrillReady())
      {
        auto endScrewingTime = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsedSecondsScrewing;
        elapsedSecondsScrewing = endScrewingTime - startScrewingTime;
        if ((double)elapsedSecondsScrewing.count() > 2)
        {
          auto startTimerRestart = std::chrono::system_clock::now();
          taskInterface->Drill_Task->triggerProg(FASTEN);
          taskInterface->Drill_Task->triggerStart();
          auto endTimerRestart = std::chrono::system_clock::now();
          start                = start + (endTimerRestart - startTimerRestart);
          startScrewingTime    = std::chrono::system_clock::now();
        }
        else
        {
          taskInterface->Robot_Task->rtde_control->forceModeStop();
          // taskInterface->unscrew();

          ROS_INFO_STREAM("SUCCESS: srcew found");
          return 0;
        }
      }
      auto end        = std::chrono::system_clock::now();
      elapsed_seconds = end - start;
    } while (elapsed_seconds.count() <= contact_time_);

    taskInterface->Robot_Task->rtde_control->forceModeStop();

    taskInterface->Robot_Task->gotoTP(
      path_.at(tryCount), false, taskInterface->velocity_, taskInterface->acceleration_);
    taskInterface->Robot_Task->gotoTP(
      path_.at(tryCount), false, taskInterface->velocity_, taskInterface->acceleration_);
  }

  ROS_ERROR_STREAM("ERROR: screw not found!");
  return 1;
}


int SukzessiveApproximationNave::plan(TaskInterface* taskInterface)
{
  if (taskInterface->radius_searchfield_ < distance_steps_)
  {
    ROS_ERROR_STREAM(
      "ERROR: planning not possible radius_searchfield_ must be greater than distance_steps_");
    return 1;
  }


  int nextLocation                = 0;
  std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
  double xComponent, zComponent;
  double sumXComponent  = 0;
  double sumZComponent  = 0;
  double lastXComponent = 0;
  double lastZComponent = 0;

  std::vector<std::vector<double> > path;
  std::vector<double> xPoints, zPoints;
  srand(time(NULL));
  for (int i = 0; i < number_tries_; i++)
  {
    bool calcDirection = true;
    while (calcDirection)
    {
      nextLocation = rand() % 8 + 1;

      switch (nextLocation)
      {
        case 1:
          xComponent = distance_steps_;
          zComponent = 0;
          break;
        case 2:
          xComponent = distance_steps_ * sin(PI / 4);
          zComponent = distance_steps_ * cos(PI / 4);
          break;
        case 3:
          xComponent = 0;
          zComponent = distance_steps_;
          break;
        case 4:
          xComponent = -distance_steps_ * sin(PI / 4);
          zComponent = distance_steps_ * cos(PI / 4);
          break;
        case 5:
          xComponent = -distance_steps_;
          zComponent = 0;
          break;
        case 6:
          xComponent = -distance_steps_ * sin(PI / 4);
          zComponent = -distance_steps_ * cos(PI / 4);
          break;
        case 7:
          xComponent = 0;
          zComponent = -distance_steps_;
          break;
        case 8:
          xComponent = distance_steps_ * sin(PI / 4);
          zComponent = -distance_steps_ * cos(PI / 4);
          break;
      }

      sumXComponent             = sumXComponent + xComponent;
      sumZComponent             = sumZComponent + zComponent;
      double distanceFromCenter = sqrt(pow(sumXComponent, 2) + pow(sumZComponent, 2));

      if ((xComponent != -lastXComponent || zComponent != -lastZComponent) &&
          taskInterface->radius_searchfield_ >= distanceFromCenter)
        calcDirection = false;
      else
      {
        sumXComponent = sumXComponent - xComponent;
        sumZComponent = sumZComponent - zComponent;
      }
    }
    lastXComponent = xComponent;
    lastZComponent = zComponent;

    xPoints.push_back(sumXComponent);
    zPoints.push_back(sumZComponent);

    std::vector<double> transformationVector = {xComponent, 0, zComponent, 0, 0, 0};
    std::vector<double> waypoint =
      taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, transformationVector);
    currentPose = waypoint;
    path.push_back(waypoint);
  }
  path_ = path;

  xPoints.insert(xPoints.begin(), 0);
  zPoints.insert(zPoints.begin(), 0);
  std::vector<std::pair<std::string, std::vector<double> > > vals = {{"xValue", xPoints},
                                                                     {"zValue", zPoints}};
  write_csv("/home/nucuser/repos/wbk-effector-group/urcontroller2/plot/saNave_search.csv", vals);

  return 0;
}

int SukzessiveApproximationNave::execute(double distanceSteps,
                                         int numberTries,
                                         TaskInterface* taskInterface)
{
  taskInterface->Robot_Task->rtde_control->zeroFtSensor();

  auto startPlanningTime = std::chrono::system_clock::now();
  number_tries_          = numberTries;
  distance_steps_        = distanceSteps;
  if (plan(taskInterface) == 1)
    return 1;
  auto endPlanningTime = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsedSecondsPlan;
  elapsedSecondsPlan = endPlanningTime - startPlanningTime;
  taskInterface->planningTime_.push_back((double)elapsedSecondsPlan.count());

  taskInterface->Drill_Task->triggerProg(FASTEN);
  taskInterface->Drill_Task->triggerStart();
  auto startScrewingTime = std::chrono::system_clock::now();


  std::vector<double> toolSpeed = {0, 0, -0.005, 0, 0, 0};
  taskInterface->Robot_Task->rtde_control->moveUntilContact(toolSpeed);
  std::vector<double> contactPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();

  for (int h = 0; h < path_.size(); h++)
  {
    // ROS_INFO_STREAM((double)h/path_.size() << " %");

    std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();

    std::vector<double> TCPForce = refTCPForce;
    bool targetNewWp             = false;
    do
    {
      double angle, velocity, acceleration;
      double deceleration = 5;
      acceleration        = taskInterface->acceleration_;
      if (TCPForce.at(1) <= taskInterface->strength_max_ &&
          TCPForce.at(1) >= taskInterface->strength_min_)
      {
        angle    = 0;
        velocity = taskInterface->velocity_;
      }
      else
      {
        refTCPForce.at(1) = TCPForce.at(1);

        double strengthRange = 4;
        double maxAngle      = 45;
        if (TCPForce.at(1) > taskInterface->strength_max_)
        {
          double percentage = ((TCPForce.at(1) - taskInterface->strength_max_) / strengthRange);
          angle             = percentage * maxAngle;
          if (angle > maxAngle)
          {
            angle        = 89.999;
            velocity     = 0.004;
            acceleration = 0.002;
          }
        }
        else if (TCPForce.at(1) < taskInterface->strength_min_)
        {
          double percentage = ((taskInterface->strength_min_ - TCPForce.at(1)) / strengthRange);
          angle             = -percentage * maxAngle;
          if (angle < maxAngle)
          {
            angle        = -89.999;
            velocity     = 0.004;
            acceleration = 0.002;
          }
        }

        velocity = taskInterface->velocity_ / cos(abs(angle) * (PI / 180));
        if (velocity > 0.004)
          velocity = 0.004;
      }

      std::vector<double> pathVecCurrentPose = taskInterface->Robot_Task->TransformBaseToToolFrame(
        {path_[h][0], path_[h][1], path_[h][2]});
      double pathVecCurrentPoseMag =
        sqrt(pow(pathVecCurrentPose.at(0), 2) + pow(pathVecCurrentPose.at(2), 2));
      double wpOffset = pathVecCurrentPoseMag * tan(angle * (PI / 180));

      std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
      pathVecCurrentPose.at(1)        = wpOffset;
      std::vector<double> newWp =
        taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, pathVecCurrentPose);
      taskInterface->Robot_Task->rtde_control->moveL(newWp, velocity, acceleration, true);

      bool adjustAngle = false;
      int progress;
      do
      {
        bool isWrenchSafetylimit = abs(sqrt(pow(TCPForce.at(0), 2) + pow(TCPForce.at(2), 2))) > 15;
        double currentDepth      = taskInterface->Robot_Task->calcDepth(contactPose);
        bool isMaxDepthReached   = abs(currentDepth) > (taskInterface->abort_at_depth_ + 0.001);
        if (taskInterface->Drill_Task->isDrillReady())
        {
          taskInterface->Robot_Task->rtde_control->stopL(50);

          auto endScrewingTime = std::chrono::system_clock::now();
          std::chrono::duration<double> elapsedSecondsScrewing;
          elapsedSecondsScrewing = endScrewingTime - startScrewingTime;
          if ((double)elapsedSecondsScrewing.count() > 2)
          {
            taskInterface->Drill_Task->triggerProg(FASTEN);
            taskInterface->Drill_Task->triggerStart();
            startScrewingTime = std::chrono::system_clock::now();
            break;
          }
          else
          {
            // taskInterface->unscrew();

            ROS_INFO_STREAM("SUCCESS: srcew found");
            return 0;
          }
        }
        else if (isMaxDepthReached || isWrenchSafetylimit)
        {
          taskInterface->Robot_Task->rtde_control->stopL();
          if (isMaxDepthReached)
          {
            ROS_ERROR_STREAM("ERROR: Slipped of Screw!");
            return 2;
          }
          else if (isWrenchSafetylimit)
          {
            ROS_ERROR_STREAM("ERROR: Wrench to high, aborting ...!");
            return 3;
          }
        }


        progress = taskInterface->Robot_Task->rtde_control->getAsyncOperationProgress();

        TCPForce = taskInterface->Robot_Task->getToolFrameForce();

        double strengthMid = taskInterface->strength_min_ +
                             (taskInterface->strength_max_ - taskInterface->strength_min_) / 2;
        if (angle > 0)
        {
          adjustAngle = TCPForce.at(1) < strengthMid;
          if (TCPForce.at(1) > refTCPForce.at(1))
          {
            adjustAngle       = true;
            refTCPForce.at(1) = TCPForce.at(1);
          }
        }
        else if (angle < 0)
        {
          adjustAngle = TCPForce.at(1) > strengthMid;
          if (TCPForce.at(1) < refTCPForce.at(1))
          {
            adjustAngle       = true;
            refTCPForce.at(1) = TCPForce.at(1);
          }
        }
        else
          adjustAngle = TCPForce.at(1) > taskInterface->strength_max_ ||
                        TCPForce.at(1) < taskInterface->strength_min_;
      } while (progress >= 0 && !adjustAngle);
      taskInterface->Robot_Task->rtde_control->stopL(deceleration);

      if (progress < 0)
        targetNewWp = true;
    } while (!targetNewWp);
  }
  taskInterface->Robot_Task->rtde_control->stopL();

  ROS_ERROR_STREAM("ERROR: screw not found!");
  return 1;
}


int LinearschwingungUmMittelpunkt::plan(TaskInterface* taskInterface)
{
  // calc rotationIncrement for axial symetric pattern
  double IdealRotationIncrement = 2 * asin((loopwidth_ / 2.0) / taskInterface->radius_searchfield_);
  int divisor                   = (int)((2.0 * PI) / IdealRotationIncrement) +
                (4 - (int)((2.0 * PI) / IdealRotationIncrement) % 4);
  if (divisor < 1)
  {
    ROS_ERROR_STREAM("loopwidth_ to high!!");
    return 1;
  }
  double rotationIncrement = (2.0 * PI) / divisor;

  // initiate vectors and path points
  double distanceEndboints     = loopwidth_;
  double offsetStuetzpunkt     = (distanceEndboints * sqrt(2)) / 2.0;
  double stuetzpunktLength     = taskInterface->radius_searchfield_ + offsetStuetzpunkt;
  double xDirectionStuetzpunkt = stuetzpunktLength;
  double zDirectionStuetzpunkt = 0;

  std::vector<double> xPoints, zPoints;

  bool approximate  = true;
  int trigger2      = 0;
  double currentPhi = 0;
  for (int i = 0; i < divisor / 2; i++)
  {
    xPoints.push_back(0);
    zPoints.push_back(0);

    int trigger = 1;
    for (int i = 0; i < 2; i++)
    {
      double xDirection = taskInterface->radius_searchfield_;
      double zDirection = 0;

      double xDirectionRotated = xDirection * cos(currentPhi) - zDirection * sin(currentPhi);
      double zDirectionRotated = xDirection * sin(currentPhi) + zDirection * cos(currentPhi);

      xPoints.push_back(xDirectionRotated);
      zPoints.push_back(zDirectionRotated);

      if (trigger == 1 && trigger2 == 0)
      {
        double xDirectionStuetzpunktRotated =
          xDirectionStuetzpunkt * cos(0.5 * rotationIncrement + currentPhi) -
          zDirectionStuetzpunkt * sin(0.5 * rotationIncrement + currentPhi);
        double zDirectionStuetzpunktRotated =
          xDirectionStuetzpunkt * sin(0.5 * rotationIncrement + currentPhi) +
          zDirectionStuetzpunkt * cos(0.5 * rotationIncrement + currentPhi);

        xPoints.push_back(xDirectionStuetzpunktRotated);
        zPoints.push_back(zDirectionStuetzpunktRotated);

        currentPhi = currentPhi + rotationIncrement;
      }
      if (trigger == 1 && trigger2 == 1)
      {
        double xDirectionStuetzpunktRotated =
          xDirectionStuetzpunkt * cos(-0.5 * rotationIncrement + currentPhi) -
          zDirectionStuetzpunkt * sin(-0.5 * rotationIncrement + currentPhi);
        double zDirectionStuetzpunktRotated =
          xDirectionStuetzpunkt * sin(-0.5 * rotationIncrement + currentPhi) +
          zDirectionStuetzpunkt * cos(-0.5 * rotationIncrement + currentPhi);

        xPoints.push_back(xDirectionStuetzpunktRotated);
        zPoints.push_back(zDirectionStuetzpunktRotated);

        currentPhi = currentPhi - rotationIncrement;
      }
      trigger = 0;
    }


    if (trigger2 == 1)
    {
      currentPhi = currentPhi + PI + 2 * rotationIncrement;
      trigger2   = 0;
    }
    else
    {
      currentPhi = currentPhi - PI;
      trigger2   = 1;
    }
  }
  xPoints.push_back(0);
  zPoints.push_back(0);

  // calculate path vectors
  std::vector<double> transformationVecInX, transformationVecInZ;
  for (int i = 0; i < xPoints.size() - 1; i++)
  {
    transformationVecInX.push_back(xPoints.at(i + 1) - xPoints.at(i));
    transformationVecInZ.push_back(zPoints.at(i + 1) - zPoints.at(i));
  }
  double maxBlendAtStuetzpunkt =
    sqrt(pow(transformationVecInX.at(1), 2) + pow(transformationVecInZ.at(1), 2));

  // calculate robotposes
  std::vector<std::vector<double> > path;
  std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
  int trigger3                    = 0;
  for (int i = 0; i < transformationVecInX.size(); i++)
  {
    std::vector<double> transformationVector = {
      transformationVecInX.at(i), 0, transformationVecInZ.at(i), 0, 0, 0};
    std::vector<double> waypoint =
      taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, transformationVector);
    currentPose = waypoint;

    // Note: 1;5;9;13
    double pathBlend, velocity, acceleration;
    pathBlend           = 0;
    bool iIsGreaterTwo  = i > 1;
    bool iIsStuetzpunkt = (i - 1) % 4 == 0;
    if ((iIsGreaterTwo && iIsStuetzpunkt) || trigger3 == 1 || i == 1)
    {
      velocity     = 10 * taskInterface->velocity_;
      acceleration = 10 * taskInterface->acceleration_;
      if (iIsGreaterTwo && iIsStuetzpunkt || i == 1)
      {
        pathBlend = 0.95 * maxBlendAtStuetzpunkt;
        trigger3  = 1;
      }
      else
      {
        trigger3 = 0;
      }
    }
    else
    {
      velocity     = taskInterface->velocity_;
      acceleration = taskInterface->acceleration_;
      trigger3     = 0;
    }
    path.push_back(waypoint);
  }
  path_ = path;

  xPoints.insert(xPoints.begin(), 0);
  zPoints.insert(zPoints.begin(), 0);
  std::vector<std::pair<std::string, std::vector<double> > > vals = {{"xValue", xPoints},
                                                                     {"zValue", zPoints}};
  write_csv("/home/nucuser/repos/wbk-effector-group/urcontroller2/plot/lo_search.csv", vals);

  return 0;
}

int LinearschwingungUmMittelpunkt::execute(double loopwidth, TaskInterface* taskInterface)
{
  taskInterface->Robot_Task->rtde_control->zeroFtSensor();

  auto startPlanningTime = std::chrono::system_clock::now();
  loopwidth_             = loopwidth;
  if (plan(taskInterface) == 1)
    return 1;
  auto endPlanningTime = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsedSecondsPlan;
  elapsedSecondsPlan = endPlanningTime - startPlanningTime;
  taskInterface->planningTime_.push_back((double)elapsedSecondsPlan.count());

  taskInterface->Drill_Task->triggerProg(FASTEN);
  taskInterface->Drill_Task->triggerStart();
  auto startScrewingTime = std::chrono::system_clock::now();


  std::vector<double> toolSpeed = {0, 0, -0.005, 0, 0, 0};
  taskInterface->Robot_Task->rtde_control->moveUntilContact(toolSpeed);
  std::vector<double> contactPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();

  for (int h = 0; h < path_.size(); h++)
  {
    // ROS_INFO_STREAM((double)h/path_.size() << " %");

    std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();

    std::vector<double> TCPForce = refTCPForce;
    bool targetNewWp             = false;
    do
    {
      double angle, velocity, acceleration;
      double deceleration = 5;
      acceleration        = taskInterface->acceleration_;
      if (TCPForce.at(1) <= taskInterface->strength_max_ &&
          TCPForce.at(1) >= taskInterface->strength_min_)
      {
        angle    = 0;
        velocity = taskInterface->velocity_;
      }
      else
      {
        refTCPForce.at(1) = TCPForce.at(1);

        double strengthRange = 4;
        double maxAngle      = 45;
        if (TCPForce.at(1) > taskInterface->strength_max_)
        {
          double percentage = ((TCPForce.at(1) - taskInterface->strength_max_) / strengthRange);
          angle             = percentage * maxAngle;
          if (angle > maxAngle)
          {
            angle        = 89.999;
            velocity     = 0.004;
            acceleration = 0.002;
          }
        }
        else if (TCPForce.at(1) < taskInterface->strength_min_)
        {
          double percentage = ((taskInterface->strength_min_ - TCPForce.at(1)) / strengthRange);
          angle             = -percentage * maxAngle;
          if (angle < maxAngle)
          {
            angle        = -89.999;
            velocity     = 0.004;
            acceleration = 0.002;
          }
        }

        velocity = taskInterface->velocity_ / cos(abs(angle) * (PI / 180));
        if (velocity > 0.004)
          velocity = 0.0035;
      }

      std::vector<double> pathVecCurrentPose = taskInterface->Robot_Task->TransformBaseToToolFrame(
        {path_[h][0], path_[h][1], path_[h][2]});
      double pathVecCurrentPoseMag =
        sqrt(pow(pathVecCurrentPose.at(0), 2) + pow(pathVecCurrentPose.at(2), 2));
      double wpOffset = pathVecCurrentPoseMag * tan(angle * (PI / 180));

      std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
      pathVecCurrentPose.at(1)        = wpOffset;
      std::vector<double> newWp =
        taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, pathVecCurrentPose);
      taskInterface->Robot_Task->rtde_control->moveL(newWp, velocity, acceleration, true);

      bool adjustAngle = false;
      int progress;
      do
      {
        bool isWrenchSafetylimit = abs(sqrt(pow(TCPForce.at(0), 2) + pow(TCPForce.at(2), 2))) > 15;
        double currentDepth      = taskInterface->Robot_Task->calcDepth(contactPose);
        bool isMaxDepthReached   = abs(currentDepth) > (taskInterface->abort_at_depth_ + 0.001);
        if (taskInterface->Drill_Task->isDrillReady())
        {
          taskInterface->Robot_Task->rtde_control->stopL(50);

          auto endScrewingTime = std::chrono::system_clock::now();
          std::chrono::duration<double> elapsedSecondsScrewing;
          elapsedSecondsScrewing = endScrewingTime - startScrewingTime;
          if ((double)elapsedSecondsScrewing.count() > 2)
          {
            taskInterface->Drill_Task->triggerProg(FASTEN);
            taskInterface->Drill_Task->triggerStart();
            startScrewingTime = std::chrono::system_clock::now();
            break;
          }
          else
          {
            // taskInterface->unscrew();

            ROS_INFO_STREAM("SUCCESS: srcew found");
            return 0;
          }
        }
        else if (isMaxDepthReached || isWrenchSafetylimit)
        {
          taskInterface->Robot_Task->rtde_control->stopL();
          if (isMaxDepthReached)
          {
            ROS_ERROR_STREAM("ERROR: Slipped of Screw!");
            return 2;
          }
          else if (isWrenchSafetylimit)
          {
            ROS_ERROR_STREAM("ERROR: Wrench to high, aborting ...!");
            return 3;
          }
        }


        progress = taskInterface->Robot_Task->rtde_control->getAsyncOperationProgress();

        TCPForce = taskInterface->Robot_Task->getToolFrameForce();

        double strengthMid = taskInterface->strength_min_ +
                             (taskInterface->strength_max_ - taskInterface->strength_min_) / 2;
        if (angle > 0)
        {
          adjustAngle = TCPForce.at(1) < strengthMid;
          if (TCPForce.at(1) > refTCPForce.at(1))
          {
            adjustAngle       = true;
            refTCPForce.at(1) = TCPForce.at(1);
          }
        }
        else if (angle < 0)
        {
          adjustAngle = TCPForce.at(1) > strengthMid;
          if (TCPForce.at(1) < refTCPForce.at(1))
          {
            adjustAngle       = true;
            refTCPForce.at(1) = TCPForce.at(1);
          }
        }
        else
          adjustAngle = TCPForce.at(1) > taskInterface->strength_max_ ||
                        TCPForce.at(1) < taskInterface->strength_min_;
      } while (progress >= 0 && !adjustAngle);
      taskInterface->Robot_Task->rtde_control->stopL(deceleration);
      // ROS_ERROR_STREAM("Force: " << TCPForce.at(1) << " / " << refTCPForce.at(1));

      if (progress < 0)
        targetNewWp = true;
    } while (!targetNewWp);
  }
  taskInterface->Robot_Task->rtde_control->stopL();

  ROS_ERROR_STREAM("ERROR: screw not found!");
  return 1;
}


int LissajousscheFiguren::plan(TaskInterface* taskInterface)
{
  if (taskInterface->radius_searchfield_ < distance_trajectory_)
  {
    ROS_ERROR_STREAM(
      "ERROR: planning not possible radius_searchfield_ must be greater than distance_trajectory_");
    return 1;
  }

  // calc ideal distance trajectory, for symetric pattern
  int divisor          = (int)((taskInterface->radius_searchfield_ / distance_trajectory_) + 1);
  distance_trajectory_ = taskInterface->radius_searchfield_ / divisor;

  // calculate points
  std::vector<double> xPoints, zPoints;
  double xValue, zValue;


  zPoints.push_back(0);

  int i = 0;
  while (true)
  {
    int argument;

    if (i % 2 == 0)
    {
      argument = 1;
    }
    else
    {
      argument = -1;
    }

    xValue = (-distance_trajectory_ / 2 - i * distance_trajectory_) * argument;
    zValue = (-taskInterface->radius_searchfield_ + i * distance_trajectory_) * argument;
    if (abs(zValue) < abs(taskInterface->radius_searchfield_ / 2) ||
        abs(xValue) > abs(taskInterface->radius_searchfield_ / 2))
      break;
    for (int j = 0; j < 2; j++)
    {
      zPoints.push_back(zValue);
      xPoints.push_back(xValue);
    }

    xValue = (distance_trajectory_ / 2 + i * distance_trajectory_) * argument;
    zValue = (taskInterface->radius_searchfield_ - i * distance_trajectory_) * argument;
    if (abs(zValue) < abs(taskInterface->radius_searchfield_ / 2) ||
        abs(xValue) > abs(taskInterface->radius_searchfield_ / 2))
      break;
    for (int j = 0; j < 2; j++)
    {
      zPoints.push_back(zValue);
      xPoints.push_back(xValue);
    }

    xValue = (-distance_trajectory_ / 2 - i * distance_trajectory_) * argument;
    zValue = (distance_trajectory_ / 2 + i * distance_trajectory_) * argument;
    if (abs(zValue) > abs(taskInterface->radius_searchfield_ / 2) ||
        abs(xValue) > abs(taskInterface->radius_searchfield_ / 2))
      break;
    for (int j = 0; j < 2; j++)
    {
      zPoints.push_back(zValue);
      xPoints.push_back(xValue);
    }

    xValue = (-taskInterface->radius_searchfield_ + i * distance_trajectory_) * argument;
    zValue = (-distance_trajectory_ / 2 - i * distance_trajectory_) * argument;
    if (abs(zValue) > abs(taskInterface->radius_searchfield_ / 2) ||
        abs(xValue) < abs(taskInterface->radius_searchfield_ / 2))
      break;
    for (int j = 0; j < 2; j++)
    {
      zPoints.push_back(zValue);
      xPoints.push_back(xValue);
    }

    xValue = (taskInterface->radius_searchfield_ - i * distance_trajectory_) * argument;
    zValue = (distance_trajectory_ / 2 + i * distance_trajectory_) * argument;
    if (abs(zValue) > abs(taskInterface->radius_searchfield_ / 2) ||
        abs(xValue) < abs(taskInterface->radius_searchfield_ / 2))
      break;
    for (int j = 0; j < 2; j++)
    {
      zPoints.push_back(zValue);
      xPoints.push_back(xValue);
    }

    i++;
  }

  zPoints.pop_back();

  // calculate path vectors
  std::vector<double> transformationVecInX, transformationVecInZ;
  for (int i = 0; i < xPoints.size() - 1; i++)
  {
    transformationVecInX.push_back(xPoints.at(i + 1) - xPoints.at(i));
    transformationVecInZ.push_back(zPoints.at(i + 1) - zPoints.at(i));
  }

  // calculate robotposes
  std::vector<std::vector<double> > path;
  std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
  int trigger3                    = 0;
  for (int i = 0; i < transformationVecInX.size(); i++)
  {
    std::vector<double> transformationVector = {
      transformationVecInX.at(i), 0, transformationVecInZ.at(i), 0, 0, 0};
    std::vector<double> waypoint =
      taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, transformationVector);
    currentPose = waypoint;
    path.push_back(waypoint);
  }
  path_ = path;

  xPoints.insert(xPoints.begin(), 0);
  zPoints.insert(zPoints.begin(), 0);
  std::vector<std::pair<std::string, std::vector<double> > > vals = {{"xValue", xPoints},
                                                                     {"zValue", zPoints}};
  write_csv("/home/nucuser/repos/wbk-effector-group/urcontroller2/plot/lf_search.csv", vals);

  return 0;
}

int LissajousscheFiguren::execute(double distanceTrajectory, TaskInterface* taskInterface)
{
  taskInterface->Robot_Task->rtde_control->zeroFtSensor();

  auto startPlanningTime = std::chrono::system_clock::now();
  distance_trajectory_   = distanceTrajectory;
  if (plan(taskInterface) == 1)
    return 1;
  auto endPlanningTime = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsedSecondsPlan;
  elapsedSecondsPlan = endPlanningTime - startPlanningTime;
  taskInterface->planningTime_.push_back((double)elapsedSecondsPlan.count());

  taskInterface->Drill_Task->triggerProg(FASTEN);
  taskInterface->Drill_Task->triggerStart();
  auto startScrewingTime = std::chrono::system_clock::now();


  std::vector<double> toolSpeed = {0, 0, -0.005, 0, 0, 0};
  taskInterface->Robot_Task->rtde_control->moveUntilContact(toolSpeed);
  std::vector<double> contactPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();

  for (int h = 0; h < path_.size(); h++)
  {
    // ROS_INFO_STREAM((double)h/path_.size() << " %");

    std::vector<double> refTCPForce = taskInterface->Robot_Task->getToolFrameForce();

    std::vector<double> TCPForce = refTCPForce;
    bool targetNewWp             = false;
    do
    {
      double angle, velocity, acceleration;
      double deceleration = 5;
      acceleration        = taskInterface->acceleration_;
      if (TCPForce.at(1) <= taskInterface->strength_max_ &&
          TCPForce.at(1) >= taskInterface->strength_min_)
      {
        angle    = 0;
        velocity = taskInterface->velocity_;
      }
      else
      {
        refTCPForce.at(1) = TCPForce.at(1);

        double strengthRange = 4;
        double maxAngle      = 45;
        if (TCPForce.at(1) > taskInterface->strength_max_)
        {
          double percentage = ((TCPForce.at(1) - taskInterface->strength_max_) / strengthRange);
          angle             = percentage * maxAngle;
          if (angle > maxAngle)
          {
            angle        = 89.999;
            velocity     = 0.004;
            acceleration = 0.002;
          }
        }
        else if (TCPForce.at(1) < taskInterface->strength_min_)
        {
          double percentage = ((taskInterface->strength_min_ - TCPForce.at(1)) / strengthRange);
          angle             = -percentage * maxAngle;
          if (angle < maxAngle)
          {
            angle        = -89.999;
            velocity     = 0.004;
            acceleration = 0.002;
          }
        }

        velocity = taskInterface->velocity_ / cos(abs(angle) * (PI / 180));
        if (velocity > 0.004)
          velocity = 0.004;
      }

      std::vector<double> pathVecCurrentPose = taskInterface->Robot_Task->TransformBaseToToolFrame(
        {path_[h][0], path_[h][1], path_[h][2]});
      double pathVecCurrentPoseMag =
        sqrt(pow(pathVecCurrentPose.at(0), 2) + pow(pathVecCurrentPose.at(2), 2));
      double wpOffset = pathVecCurrentPoseMag * tan(angle * (PI / 180));

      std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getActualTCPPose();
      pathVecCurrentPose.at(1)        = wpOffset;
      std::vector<double> newWp =
        taskInterface->Robot_Task->rtde_control->poseTrans(currentPose, pathVecCurrentPose);
      taskInterface->Robot_Task->rtde_control->moveL(newWp, velocity, acceleration, true);

      bool adjustAngle = false;
      int progress;
      do
      {
        bool isWrenchSafetylimit = abs(sqrt(pow(TCPForce.at(0), 2) + pow(TCPForce.at(2), 2))) > 15;
        double currentDepth      = taskInterface->Robot_Task->calcDepth(contactPose);
        bool isMaxDepthReached   = abs(currentDepth) > (taskInterface->abort_at_depth_ + 0.001);
        if (taskInterface->Drill_Task->isDrillReady())
        {
          taskInterface->Robot_Task->rtde_control->stopL(50);

          auto endScrewingTime = std::chrono::system_clock::now();
          std::chrono::duration<double> elapsedSecondsScrewing;
          elapsedSecondsScrewing = endScrewingTime - startScrewingTime;
          if ((double)elapsedSecondsScrewing.count() > 2)
          {
            taskInterface->Drill_Task->triggerProg(FASTEN);
            taskInterface->Drill_Task->triggerStart();
            startScrewingTime = std::chrono::system_clock::now();
            break;
          }
          else
          {
            // taskInterface->unscrew();

            ROS_INFO_STREAM("SUCCESS: srcew found");
            return 0;
          }
        }
        else if (isMaxDepthReached || isWrenchSafetylimit)
        {
          taskInterface->Robot_Task->rtde_control->stopL();
          if (isMaxDepthReached)
          {
            ROS_ERROR_STREAM("ERROR: Slipped of Screw!");
            return 2;
          }
          else if (isWrenchSafetylimit)
          {
            ROS_ERROR_STREAM("ERROR: Wrench to high, aborting ...!");
            return 3;
          }
        }


        progress = taskInterface->Robot_Task->rtde_control->getAsyncOperationProgress();

        TCPForce = taskInterface->Robot_Task->getToolFrameForce();
        // ROS_ERROR_STREAM("Force: " << TCPForce.at(1) << " / " << refTCPForce.at(1));

        double strengthMid = taskInterface->strength_min_ +
                             (taskInterface->strength_max_ - taskInterface->strength_min_) / 2;
        if (angle > 0)
        {
          adjustAngle = TCPForce.at(1) < strengthMid;
          if (TCPForce.at(1) > refTCPForce.at(1))
          {
            adjustAngle       = true;
            refTCPForce.at(1) = TCPForce.at(1);
          }
        }
        else if (angle < 0)
        {
          adjustAngle = TCPForce.at(1) > strengthMid;
          if (TCPForce.at(1) < refTCPForce.at(1))
          {
            adjustAngle       = true;
            refTCPForce.at(1) = TCPForce.at(1);
          }
        }
        else
          adjustAngle = TCPForce.at(1) > taskInterface->strength_max_ ||
                        TCPForce.at(1) < taskInterface->strength_min_;
      } while (progress >= 0 && !adjustAngle);
      taskInterface->Robot_Task->rtde_control->stopL(deceleration);

      if (progress < 0)
        targetNewWp = true;
    } while (!targetNewWp);
  }
  taskInterface->Robot_Task->rtde_control->stopL();

  ROS_ERROR_STREAM("ERROR: screw not found!");
  return 1;
}


void TaskInterface::unscrew()
{
  Robot_Task->startForcemode(WRENCH_UP, 0.5);
  Drill_Task->triggerProg(RESET);
  Drill_Task->triggerProg(LOSEN);
 
  while (!Drill_Task->isDrillReady())
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // blocking
  
  Robot_Task->rtde_control->forceModeStop();
}


bool TaskInterface::checkLateralForce(std::vector<double> moveDirection)
{
  ROS_INFO_STREAM("checkLateralForce()");

  // rotate moveDirection 90 degrees in x,z plane of tool frame, and calculate unit vector
  double xUnitCompRotated =
    (-moveDirection.at(2) / sqrt(pow(-moveDirection.at(2), 2) + pow(moveDirection.at(0), 2)));
  double zUnitCompRotated =
    (moveDirection.at(0) / sqrt(pow(-moveDirection.at(2), 2) + pow(moveDirection.at(0), 2)));

  // scale to 15 cm
  double xCompScaled = xUnitCompRotated * 0.15;
  double zCompScaled = zUnitCompRotated * 0.15;

  std::vector<double> currentPos = Robot_Task->rtde_receive->getActualTCPPose();

  Robot_Task->rtde_control->forceModeSetDamping(1);
  std::vector<double> taskFrameLeft =
    Robot_Task->rtde_control->poseTrans(currentPos, {xCompScaled, 0, zCompScaled, 0, 0, 0});
  std::vector<double> taskFrameRight =
    Robot_Task->rtde_control->poseTrans(currentPos, {-xCompScaled, 0, -zCompScaled, 0, 0, 0});
  std::vector<int> selectionVector = {0, 1, 0, 0, 0, 0};
  std::vector<double> wrenchLeft   = {0, 5, 0, 0, 0, 0};
  std::vector<double> wrenchRight  = {0, 5, 0, 0, 0, 0};
  int forceType                    = 1;
  std::vector<double> limits       = {0.1, 0.005, 0.1, 0.17, 0.17, 0.17};

  bool checkLeft  = false;
  bool checkRight = false;

  auto startCheckingTimer = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsedSeconds;
  do
  {
    Robot_Task->rtde_control->forceMode(
      taskFrameLeft, selectionVector, wrenchLeft, forceType, limits);
    std::vector<double> TCPForce = Robot_Task->getToolFrameForce();
    double reactionForce = xUnitCompRotated * TCPForce.at(0) + zUnitCompRotated * TCPForce.at(2);

    ROS_ERROR_STREAM("LEFT: " << reactionForce);
    if (reactionForce < -2)
    {
      // currentPosLeft = Robot_Task->rtde_receive->getActualTCPPose();
      checkLeft = true;
      break;
    }

    auto endCheckingTimer = std::chrono::system_clock::now();
    elapsedSeconds        = endCheckingTimer - startCheckingTimer;
  } while ((double)elapsedSeconds.count() < 0.1);
  Robot_Task->rtde_control->forceModeStop();

  Robot_Task->rtde_control->moveL(currentPos, 0.05, 0.02, false);

  startCheckingTimer = std::chrono::system_clock::now();
  do
  {
    Robot_Task->rtde_control->forceMode(
      taskFrameRight, selectionVector, wrenchRight, forceType, limits);
    std::vector<double> TCPForce = Robot_Task->getToolFrameForce();
    double reactionForce = xUnitCompRotated * TCPForce.at(0) + zUnitCompRotated * TCPForce.at(2);

    ROS_ERROR_STREAM("RIGHT: " << reactionForce);
    if (reactionForce > 2)
    {
      checkRight = true;
      break;
    }


    auto endCheckingTimer = std::chrono::system_clock::now();
    elapsedSeconds        = endCheckingTimer - startCheckingTimer;
  } while ((double)elapsedSeconds.count() < 0.1);
  Robot_Task->rtde_control->forceModeStop();

  Robot_Task->rtde_control->moveL(currentPos, 0.05, 0.02, false);

  ROS_INFO_STREAM(checkLeft << checkRight);
  return checkLeft && checkRight;
}
