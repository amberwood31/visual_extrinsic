#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <sstream>

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


using namespace std;

const float eps = 1e-4;
template<typename T>
cv::Matx<T, 3, 3> rodrigue2rot(const cv::Matx<T, 3, 1>& paramIn)
{
    cv::Matx<T, 3, 3>  I = cv::Matx<T, 3, 3>::eye();

    T x = paramIn(0, 0);
    T y = paramIn(1, 0);
    T z = paramIn(2, 0);

    T d2 = x*x+y*y+z*z;
    T d = sqrt(d2);

    cv::Matx<T, 3, 3> W;
    W << 0, -z, y,
        z, 0, -x,
        -y,  x, 0;

    if(d<eps)
        return (I + W + 0.5f*W*W);
    else
        return (I + W*sin(d)*(1/d) + W*W*(1.0f-cos(d))*(1/d2));
    
}

template<typename T>
cv::Matx<T, 4, 4> rodrigue2hom(const cv::Matx<T, 6, 1>& rodrigueRep){
    cv::Matx<T, 3, 1> rodrigueR(rodrigueRep(0, 0), rodrigueRep(1,0), rodrigueRep(2, 0));
    cv::Matx<T, 3, 3> R = rodrigue2rot(rodrigueR);

    cv::Matx<T, 4, 4> homM(
        R(0, 0), R(0, 1), R(0, 2), rodrigueRep(3, 0),
        R(1, 0), R(1, 1), R(1, 2), rodrigueRep(4, 0),
        R(2, 0), R(2, 1), R(2, 2), rodrigueRep(5, 0),
        T(0), T(0), T(0), T(1));

    return homM;
}

cv::Matx44f invMat(const cv::Matx44f& M)
{
    cv::Matx33f R = M.get_minor<3, 3>(0, 0);
    R = R.t();
    cv::Vec3f t(M(0, 3), M(1, 3), M(2, 3));
    t = -R * t;
    cv::Matx44f out(
        R(0, 0), R(0, 1), R(0, 2), t(0),
        R(1, 0), R(1, 1), R(1, 2), t(1),
        R(2, 0), R(2, 1), R(2, 2), t(2),
        0.0, 0.0, 0.0, 1.0);

    return out;
}

void RotateZToCam(int c, const cv::Matx<float, 3,3>& temp_rot){
    cv::Matx<float, 3, 1> input(0.0, 0.0, 1.0);

    cv::Matx<float, 3, 1> temp;
    // std::cout << "check translation: " << cam_pose(0,3) << ", " << cam_pose(1,3) << ", " << cam_pose(2,3) <<std::endl;
    temp = temp_rot * input;
    // std::cout << "cam" << c;
    // std::cout << " optical axis in world frame: " << std::endl;
    // std::cout << temp << std::endl;
}

tf2::Matrix3x3 CVMat2TF(cv::Matx33f& cv_mat){
    tf2::Matrix3x3 temp(cv_mat(0,0), cv_mat(0,1), cv_mat(0,2),
                        cv_mat(1,0), cv_mat(1,1), cv_mat(1,2),
                        cv_mat(2,0), cv_mat(2,1), cv_mat(2,2));
    return temp;
}


void LoadExtrinsics(const string path2calibrations, std::vector<cv::Matx44f>& extrinsics, int& nrCams)
{
    
    string mcs_settings = path2calibrations;
    std::cout << "load path: " << mcs_settings << std::endl;
    cv::FileStorage mcs_calib_data(mcs_settings, cv::FileStorage::READ);
    nrCams = (int)mcs_calib_data["CameraSystem.nrCams"];
    std::string cam_type = std::string(mcs_calib_data["CameraSystem.CamType"]);
    std::cout << "cam type: " << cam_type << std::endl;
    std::vector<cv::Matx44f> M_c_s(nrCams);
    std::cout << "num_cams: " << nrCams << std::endl;

    extrinsics.resize(nrCams);

    for (int c = 0; c < nrCams; ++c)
    {
        // all M_c
        cv::Matx61f tmp;
        for (int p = 1; p < 7; ++p)
        {
            string param = "CameraSystem.cam" + std::to_string(c + 1) + "_" + std::to_string(p);
            tmp(p - 1) = mcs_calib_data[param];
        }
        
        

        M_c_s[c] = rodrigue2hom<float>(tmp);//@audit need to confirm with LC what's the representation for the rotation parameters inside the calibration file
            

        std::cout << "load transform: " << std::endl;

        extrinsics[c] = M_c_s[c];
        


    }
    std::cout << "finished loading Extrinsics " << std::endl;
    
}

void LoadExtrinsicsHuizhou(const string path2calibrations, std::vector<cv::Matx44f>& extrinsics, int& nrCams)
{
    
    string mcs_settings = path2calibrations;
    std::cout << "load path: " << mcs_settings << std::endl;
    cv::FileStorage mcs_calib_data(mcs_settings, cv::FileStorage::READ);
    nrCams = (int)mcs_calib_data["CameraSystem.nrCams"];
    std::string cam_type = std::string(mcs_calib_data["CameraSystem.CamType"]);
    std::cout << "cam type: " << cam_type << std::endl;
    std::cout << "num_cams: " << nrCams << std::endl;

    extrinsics.resize(nrCams);

    for (int c = 0; c < nrCams; ++c)
    {
        // all M_c
        cv::Matx61f tmp;
        for (int p = 0; p < 6; ++p)
        {
            string param = "CameraSystem.cam" + std::to_string(c + 1) + "_" + std::to_string(p+1);
            tmp(p) = mcs_calib_data[param];
        }
        std::cout << "load vector: " << tmp << std::endl;
        
        cv::Matx33f rot = rodrigue2rot<float>(tmp.get_minor<3,1>(0,0));
  
        cv::Matx33f rot_inv = rot.inv();
        cv::Matx31f tvec = tmp.get_minor<3,1>(3,0);
        cv::Matx31f trans = - rot_inv * tvec;
            
    
        cv::Matx44f temp = cv::Matx44f::eye();
        temp(0,0) = rot_inv(0,0);
        temp(0,1) = rot_inv(0,1);
        temp(0,2) = rot_inv(0,2);
        temp(1,0) = rot_inv(1,0);
        temp(1,1) = rot_inv(1,1);
        temp(1,2) = rot_inv(1,2);
        temp(2,0) = rot_inv(2,0);
        temp(2,1) = rot_inv(2,1);
        temp(2,2) = rot_inv(2,2);

        temp(0,3) = trans(0);
        temp(1,3) = trans(1);
        temp(2,3) = trans(2);
        std::cout << "load transform: " << std::endl;
        std::cout << temp << std::endl;

        extrinsics[c] = temp;
        
        RotateZToCam(c, rot);


    }

    // transform cam0 to be identity matrix
    cv::Matx44f inv_cam0 = invMat(extrinsics[0]);
    for (int c = 0; c < nrCams; ++c){
        extrinsics[c] = inv_cam0 * extrinsics[c];
    }

    std::cout << "finished loading Extrinsics " << std::endl;
    
}

void PrintTFMatrix(const tf2::Matrix3x3& input){
    std::cout << "tf transform: " << input[0].getX() << "," << input[0].getY() << "," << input[0].getZ() <<std::endl;
    std::cout << "              " << input[1].getX() << "," << input[1].getY() << "," << input[1].getZ() <<std::endl;
    std::cout << "              " << input[2].getX() << "," << input[2].getY() << "," << input[2].getZ() <<std::endl;


}
void Sent2TF(int cam, const tf2::Matrix3x3& rot, const float& t0, const float& t1, const float& t2){
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "world";
    static_transformStamped.child_frame_id = "cam" + std::to_string(cam);
    static_transformStamped.transform.translation.x = t0;
    static_transformStamped.transform.translation.y = t1;
    static_transformStamped.transform.translation.z = t2;

    double roll, pitch, yaw;
    rot.getRPY(roll, pitch, yaw);
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(static_transformStamped);
    std::cout << "Sent to TF " << std::endl;
    PrintTFMatrix(rot);
    std::cout << "translation: " << t0 << ", " << t1 << "," << t2 << std::endl;
    // std::cout <<"check deter minant: " << m.determinant() << std::endl;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "visualize_extrinsics");
    if (argc != 2)
	{
		std::cerr << std::endl << "Usage: ./visualize_extrinsics extrinsics_file" << std::endl;
		return 1;
	}

	std::string path2extrintics = std::string(argv[1]);
    std::vector<cv::Matx44f> extrinsics;
    int nrCams;
    LoadExtrinsics(path2extrintics, extrinsics, nrCams);
    // std::cout << "test t0:" << extrinsics[0](3) << std::endl;
  
    for (int c = 0; c < nrCams ; c++){
        cv::Matx33f temp_m = extrinsics[c].get_minor<3,3>(0,0);
        tf2::Matrix3x3 temp = CVMat2TF(temp_m);

        Sent2TF(c, temp, extrinsics[c](0,3), extrinsics[c](1,3), extrinsics[c](2,3));

    }
    
    
    ROS_INFO("Spinning until killed publishing cam to world");
    ros::spin();
  
    return 0;
}
