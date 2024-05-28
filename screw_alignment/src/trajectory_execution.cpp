#include "screw_alignment/spiral_interpolation.h" 
#include "screw_alignment/trajectory_execution.h"

#include <chrono>
#include <thread>
#include <std_msgs/Bool.h>


class TrajectoryExecutor {
public:
    TrajectoryExecutor(ros::NodeHandle& nh) : nh_(nh){
        p_client_ = nh_.serviceClient<agiprobot_msgs::tcp>("/pushTCPPose");
    }

    // Define a function to fetch a current TCP Pose using the ROS Service
    std::vector<double> fetchTCPPose() {
        // Initialize the ROS node
        ros::NodeHandle nh;

        // Create a service client
        ros::ServiceClient fetch_client = nh.serviceClient<agiprobot_msgs::tcp>("/fetchTCPPose");

        // Create a service request

        agiprobot_msgs::tcp tcp_r;

        // Call the service for the current pose
        if (fetch_client.call(tcp_r)) {
            // Check if the service call was successful
            std::cout << "/fetchTCPPose Service" << std::endl;
            std::vector<double> fetched_pose = tcp_r.response.pose;
            std::cout << "Current Pose is: " << std::endl;
            for (double value : fetched_pose){
                std::cout << value << " ";
            }
            std::cout << std::endl;
            ROS_INFO("Current Pose successfully fetched.");
            return fetched_pose; // Return fetched pose 
        } else {
            ROS_ERROR("Failed to call service to fetch Current Pose.");
        }
    }

    std::vector<double> fetchToolframeForce() {
        // Initialize the ROS node
        ros::NodeHandle nh;

        // Create a service client
        ros::ServiceClient fetch_force_client = nh.serviceClient<agiprobot_msgs::toolframeForce>("/fetchToolframeForce");

        // Create a service request

        agiprobot_msgs::toolframeForce toolForce;

        // Call the service for the current pose
        if (fetch_force_client.call(toolForce)) {
            // Check if the service call was successful
            std::cout << "/fetchToolframeForce Service" << std::endl;
            std::vector<double> fetched_toolframeForce = toolForce.response.forceVector;
            std::cout << "Current Toolframe Force is: " << std::endl;
            for (double value : fetched_toolframeForce){
                std::cout << value << " ";
            }
            std::cout << std::endl;
            ROS_INFO("Current force vector successfully fetched.");
            return fetched_toolframeForce; // Return fetched pose 
        } else {
            ROS_ERROR("Failed to call service to fetch Current force vector.");
        }
    }


    // Define a function to push a vector of TCP poses using the ROS Service
    bool pushTCPPose(const std::vector<std::vector<double>>& points) {
        // Initialize the ROS node
        ros::NodeHandle nh;

        // Create a service client
        ros::ServiceClient push_client = nh.serviceClient<agiprobot_msgs::tcp>("/pushTCPPose");

        // Loop through the poses and push each one
        int point_counter = 1; 
        for (const std::vector<double>& point : points) {
            if (point.size() != 6)
            {
                ROS_WARN("Skipping invalid pose - expected a 6D pose vector.");
                continue;
            }
            // Create a request message for each pose
            agiprobot_msgs::tcp::Request req;

            // Giving pose value to service
            req.pose = point;
            // Printing each point with count
            std::cout << "Point " << point_counter << std::endl;
            for (double value : point) {
                std::cout << value << " ";
            }
            std::cout << std::endl;
            // Create a response object
            agiprobot_msgs::tcp::Response res;
            // Call the service for the current pose
            if (push_client.call(req, res))
            {
                if (res.success)
                {
                    ROS_INFO("Successfully pushed TCP pose.");
                }
                else
                {
                    ROS_ERROR("Failed to push TCP pose.");
                    return false;
                }
            }
            else
            {
                ROS_ERROR("Failed to call service: /pushTCPPose");
                return false; // Exit the method on service call failure
            }
            point_counter ++;
        }
        return true;  // Return true if all poses were successfully pushed
    }


    bool drillON = false;
    bool isDrillReady()
        {
            return !drillON;
        }
   
        
    private:
    ros::NodeHandle nh_;
    ros::ServiceClient p_client_;
    };

    void digitalInputCallback(const std_msgs::Bool::ConstPtr& msg) {
        // Print the received data (True or False)
        if (msg->data) {
            ROS_INFO("Received True on /digital_input04 - Drill is operating.");
        } else {
            ROS_INFO("Received False on /digital_input04 - Drill is not operating.");
        }
    }
    int main(int argc, char** argv) {

        ros::init(argc, argv, "trajectory_execution_node");

        ros::NodeHandle nh;
        

        // Create an instance of the TrajectoryExecutor class
        TrajectoryExecutor posePusher(nh);

        // Fetching Current TCP Pose
        std::vector<double> current_pose = posePusher.fetchTCPPose();
        std::vector<double> current_force = posePusher.fetchToolframeForce();


        // Create an instance of the SpiralInterpolation class
        SpiralInterpolation spiral;

        // Parameters for spiral interpolation
        //double angleIncrement = 0.001;
        //double trajectoryDistance = 0.0005;
        //double radiusSearch = 0.002;

        // Compute the interpolated poses using the SpiralInterpolation class
        //std::vector<std::vector<double>> interpolatedPoses = spiral.interpolateSpiral(angleIncrement, trajectoryDistance, radiusSearch, current_pose);

        //posePusher.pushTCPPose(interpolatedPoses);

        // Create a publisher to publish to /digital_output04
        ros::Publisher pub = nh.advertise<std_msgs::Bool>("/digital_output04", 10);
        std_msgs::Bool pub_msg;
        int msg_count = 0;
        ros::Rate rate(10);  // 10 Hz spin rate
        while (ros::ok() && msg_count < 10) {
            posePusher.drillON = !posePusher.drillON;
            std::cout << "drillON " << posePusher.drillON << std::endl;

            pub_msg.data = posePusher.drillON;

            pub.publish(pub_msg);

            // Print the message and spin
            std::cout << "Pubmsg " << pub_msg << std::endl;
            ros::spinOnce();
            rate.sleep();
            msg_count++;
        }
       
        ros:: shutdown();   
        return 0;
    }
