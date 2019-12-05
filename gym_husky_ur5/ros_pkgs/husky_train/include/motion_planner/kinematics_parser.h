#ifndef KINEMATICS_PARSER_H
#define KINEMATICS_PARSER_H

#include "Eigen/Dense"

const double Pi = 3.14159265358979;

class Parser
{
public:
    Parser();
    Eigen::MatrixXd Foward(Eigen::VectorXd q);
    Eigen::MatrixXd Jacobn(Eigen::VectorXd q);
    Eigen::MatrixXd Jacob0(Eigen::VectorXd q);
    Eigen::VectorXd Inverse(Eigen::MatrixXd t, Eigen::VectorXd qR);
private:
    Eigen::MatrixXd dh;
    Eigen::VectorXd JointAngle2DhAngle(Eigen::VectorXd q);
    Eigen::VectorXd DhAngle2JointAngle(Eigen::VectorXd dhQ);
    Eigen::Matrix4d Translation(Eigen::Vector3d p);
    Eigen::Matrix4d Rotation(char axis, double r);
    Eigen::VectorXd DeltaT(Eigen::MatrixXd t1, Eigen::MatrixXd t2);
    Eigen::MatrixXd DhFoward(Eigen::MatrixXd dh, Eigen::VectorXd dhQ);
    Eigen::MatrixXd DhJacobn(Eigen::MatrixXd dh, Eigen::VectorXd dhQ);
    Eigen::MatrixXd DhJacob0(Eigen::MatrixXd dh, Eigen::VectorXd dhQ);
    Eigen::VectorXd DhInverse(Eigen::MatrixXd dh, Eigen::MatrixXd t, Eigen::VectorXd dhQR);
};

#endif // KINEMATICS_PARSER_H
