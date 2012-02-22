#include <string>
#include <fstream>
#include <stdlib.h>
#include <ros/ros.h>
#include <math.h>
//#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <boost/lexical_cast.hpp>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <trajectory_msgs/JointTrajectory.h>


typedef struct{
  char color_set;
  int id;
  double alpha;
  std::string ns;
  std_msgs::ColorRGBA color;
  ros::Duration duration;
} MarkerInfo;

class VisualizeArm
{
  public:

    /* \brief constructor
     * @param "right arm" or "left arm"
    */     
    VisualizeArm(std::string arm_name);

    /* \brief destructor */
    ~VisualizeArm();

    /* \brief set reference frame of visualizations */
    void setReferenceFrame(std::string &frame);

    /* \brief initialize the KDL chain for the robot arm */
    bool initKDLChain();

    /* \brief compute FK for the pr2 arm meshes using the KDL chain */
    bool computeFKforVisualizationWithKDL(const std::vector<double> &jnt_pos, std::vector<geometry_msgs::PoseStamped> &poses);

    /* \brief compute FK for a joint configuration using the KDL chain */
    bool computeFKwithKDL(const std::vector<double> angles, int frame_num, geometry_msgs::Pose &pose);

    /* \brief compute FK for the pr2 arm meshes using the kinematic service*/
    bool computeFKforVisualization(const std::vector<double> &jnt_pos, std::vector<geometry_msgs::PoseStamped> &poses);

    /* \brief visualize the pr2 arm in the configuration */
    void visualizeArmConfiguration(double color_num, const std::vector<double> &jnt_pos);

    /* \brief visualize the arm of the pr2 in specified color (color: 1-360) */
    void visualizeArmMeshes(double color_num, std::vector<geometry_msgs::PoseStamped> &poses);

    /* \brief visualize a pose (sphere, arrow, string of text) */
    void visualizePose(const std::vector<double> &pose, std::string text);

    /* \brief visualize a pose (sphere, arrow, string of text) */
    void visualizePose(const geometry_msgs::Pose &pose, std::string text);

    /* \brief visualize a list of poses (sphere, arrow, pose index number) */
    void visualizePoses(const std::vector<std::vector<double> > &poses);
   
    /* \brief visualize the trajectory stored in a CSV file */
    void visualizeTrajectoryFile(std::string filename, int throttle);

    /* \brief visualize the environment described by an env file */
    void visualizeEnvironment(std::string filename);

    /* \brief visualize cuboids */
    void visualizeObstacles(const std::vector<std::vector<double> > &obstacles);
  
    /* \brief visualize a path made up of 4cm spheres */
    void visualize3DPath(std::vector<std::vector<double> > &dpath);

    /* \brief parse a CSV file - note: no comma at end of last line! */
    bool parseCSVFile(std::string filename, int num_cols, std::vector<std::vector<double> > &data);

    /* \brief parse a CSV file that contains poses - should rewrite it to
     * use parseCSVFile() */
    bool parsePoseFile(std::string filename, std::vector<std::vector<double> > &poses);

    /* \brief parse an environment file - currently not very robust at all */
    bool parseEnvironmentFile(std::string filename);

    /* \brief display a throttled set of arm configurations in a trajectory
     * by default throttle = 5 */
    void visualizeArmConfigurations(const std::vector<std::vector<double> > &traj, int throttle);

    /* \brief display a throttled set of arm configurations in a trajectory msg */
    void visualizeJointTrajectoryMsg(trajectory_msgs::JointTrajectory traj_msg, int throttle);
    
    /* \brief print out the information that was parsed from the env file */
    void printEnvironmentInfo(FILE *fid);

    /* \brief display a gripper given the arm's joint angles */
    void visualizeGripperConfiguration(double color_num, const std::vector<double> &jnt_pos);

    /* \brief display the gripper meshes (called by visualizeGripperConfiguration) */
    void visualizeGripperMeshes(double color_num, std::vector<geometry_msgs::PoseStamped> &poses);

    /* \brief display a list of states (xyz coordinates) (intended for use with sbpl) */
    void visualizeBasicStates(const std::vector<std::vector<double> > &states, const std::vector<double> &color, std::string name, double size);
    
    /* \brief display a list of states (xyz coordinates with rpy arrows) (intended for use with sbpl) */
    void visualizeDetailedStates(const std::vector<std::vector<double> > &states, const std::vector<std::vector<double> >&color, std::string name, double size);

    /* \brief display a sphere */
    void visualizeSphere(std::vector<double> pose, int color, std::string text, double radius);
    
    /* \brief display a sphere */
    void visualizeSphere(geometry_msgs::PoseStamped pose, double radius, MarkerInfo m);

    /* \brief display a list of spheres of the same radius and color */
    void visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string text, double radius);

    /* \brief display a cylinder */
    void visualizeCylinder(geometry_msgs::PoseStamped pose, double length, double radius, MarkerInfo m);
    
    void visualizeShape(geometry_msgs::PoseStamped pose, geometry_msgs::Vector3 scale, MarkerInfo m, std::string shape);

    /* DOESN'T WORK */
    void clearAllVisualizations();

    /* \brief delete a visualization marker from rviz */
    void deleteVisualization(std::string ns, int id);

    void visualizeSimpleArmConfiguration(const std::vector<double> &jnt_pos, MarkerInfo &mi, bool gripper);

    void visualizeSimpleArm(const std::vector<geometry_msgs::PoseStamped> &poses, MarkerInfo &mi, bool gripper);

    void initColorSets();

    void setColorMsg(std_msgs::ColorRGBA &c, double r, double g, double b, double a);

    void getSimpleGripperPose(geometry_msgs::Pose wpose, geometry_msgs::Pose &gpose);

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    ros::Publisher marker_publisher_;
    ros::Publisher marker_array_publisher_;

    visualization_msgs::Marker marker_;
    visualization_msgs::MarkerArray marker_array_;

    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;
    std::vector<std::string> pr2_arm_meshes_;
    std::vector<std::string> pr2_gripper_meshes_;
    std::vector<geometry_msgs::Vector3> pr2_arm_meshes_scale_;

    KDL::JntArray jnt_pos_in_;
    KDL::JntArray jnt_pos_out_;
    KDL::Frame p_out_;
    KDL::Chain chain_;
    KDL::Chain gripper_l_chain_;
    KDL::Chain gripper_r_chain_;
    KDL::ChainFkSolverPos_recursive *fk_solver_;
    KDL::ChainFkSolverPos_recursive *gripper_l_fk_solver_;
    KDL::Tree kdl_tree_;

    int num_joints_;
    int num_color_sets_;
    std::string arm_name_;
    std::string arm_config_file_;
    std::string reference_frame_;
    std::string side_;
    std::string side_full_;
    std::string fk_service_name_;
    std::string ik_service_name_;
    std::string chain_root_name_;
    std::string chain_tip_name_;

    std::vector<double> start_config_;
    std::vector<double> goal_pose_;
    std::vector<std::vector<double> > cubes_;
    std::vector<std::vector<std_msgs::ColorRGBA> > color_sets_;
    double position_tolerance_;
    double orientation_tolerance_;
    bool goal_is_6dof_;
};


