
#include "motion_planner/kinematics_parser.h"

Parser::Parser()
{
    // DH Parameters of Jaco, p1
    // double d1 = 0.2755;	// Base to elbow
    // double d2 = 0.4100;	// Arm length
    // double d3 = 0.2073;	// Front arm length
    // double d4 = 0.0741;	// First wrist length
    // double d5 = 0.0741;	// Second wrist length
    // double d6 = 0.1600;	// Wrist to center of the hand 20181215 from 0.16 to 0.21
    // double e2 = 0.0098;	// Joint 3-4 lateral offset

    // double aa = 0.5235987755;
    // double ca = (cos(aa));
    // double sa = (sin(aa));
    // double c2a = (cos(2 * aa));
    // double s2a = (sin(2 * aa));
    // double d4b = (d3 + sa / s2a*d4);
    // double d5b = (sa / s2a*d4 + sa / s2a*d5);
    // double d6b = (sa / s2a*d5 + d6);

    // DH is DH model of the manipulator, p4. Rows are DOF, cols are alpha/a/theta/d.
    // The angle uses radian, the distance uses mm.
    // Eigen::MatrixXd dh(6, 4);
    // dh << Pi / 2,  0, 0,   d1,
    //           Pi, d2, 0,   0,
    //       Pi / 2,  0, 0, -e2,
    //       2 * aa,  0, 0, -d4b,
    //       2 * aa,  0, 0, -d5b,
    //           Pi,  0, 0, -d6b;

    // UR5 DH
    // https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_kinematics/src/ur_kin.cpp
    const double d1 =  0.089159;
    const double a2 = -0.42500;
    const double a3 = -0.39225;
    const double d4 =  0.10915;
    const double d5 =  0.09465;
    const double d6 =  0.0823;
  
    // DH for UR5:
    // a = [0.00000, -0.42500, -0.39225,  0.00000,  0.00000,  0.0000]
    // d = [0.089159,  0.00000,  0.00000,  0.10915,  0.09465,  0.0823]
    // alpha = [ 1.570796327, 0, 0, 1.570796327, -1.570796327, 0 ]
    // q_home_offset = [0, -1.570796327, 0, -1.570796327, 0, 0]
    // joint_direction = [-1, -1, 1, 1, 1, 1]
    // mass = [3.7000, 8.3930, 2.2750, 1.2190, 1.2190, 0.1879]
    // center_of_mass = [ [0, -0.02561, 0.00193], [0.2125, 0, 0.11336], [0.11993, 0.0, 0.0265], [0, -0.0018, 0.01634], [0, 0.0018,0.01634], [0, 0, -0.001159] ]
  
    // DH is DH model of the manipulator. Rows are DOF, cols are alpha/a/theta/d.
    // The angle uses radian, the distance uses mm.
    // https://www.universal-robots.com/how-tos-and-faqs/faq/ur-faq/parameters-for-calculations-of-kinematics-and-dynamics-45257/
    Eigen::MatrixXd dh(6, 4);
    dh <<  Pi/2,  0, 0,  d1,
              0, a2, 0,   0,
              0, a3, 0,   0,
           Pi/2,  0, 0,  d4,
          -Pi/2,  0, 0,  d5,
              0,  0, 0,  d6;

    this->dh = dh;
}

Eigen::MatrixXd Parser::Foward(Eigen::VectorXd q)
{
    Eigen::VectorXd dhQ = JointAngle2DhAngle(q);
    return DhFoward(dh, dhQ);
}

Eigen::MatrixXd Parser::Jacobn(Eigen::VectorXd q)
{
    Eigen::VectorXd dhQ = JointAngle2DhAngle(q);
    return DhJacobn(dh, dhQ);
}

Eigen::MatrixXd Parser::Jacob0(Eigen::VectorXd q)
{
    Eigen::VectorXd dhQ = JointAngle2DhAngle(q);
    return DhJacob0(dh, dhQ);
}

Eigen::VectorXd Parser::Inverse(Eigen::MatrixXd t, Eigen::VectorXd qR)
{
    Eigen::VectorXd dhQR = JointAngle2DhAngle(qR);
    return DhInverse(dh, t, dhQR);
}

// Transformation from DH algorithm to Jaco physical angles, p4
Eigen::VectorXd Parser::JointAngle2DhAngle(Eigen::VectorXd q)
{
    Eigen::VectorXd dhQ(6);
    // For Jaco2
    // dhQ << -q(0), q(1) - 0.5*Pi, q(2) + 0.5*Pi, q(3), q(4) - Pi, q(5) + (Pi - 80.0 / 180 * Pi);
    // For UR5
    dhQ << q(0), q(1), q(2), q(3), q(4), q(5);

    return dhQ;
}

// Transformation from Jaco physical angles to DH algorithm, p4
Eigen::VectorXd Parser::DhAngle2JointAngle(Eigen::VectorXd dhQ)
{
    Eigen::VectorXd q(6);
    // For Jaco2
    // q << -dhQ(0), (dhQ(1) + 0.5*Pi), dhQ(2) - 0.5*Pi, dhQ(3), dhQ(4) + Pi, dhQ(5) - (Pi - 80.0 / 180 * Pi);
    // For UR5
    q << dhQ(0), dhQ(1), dhQ(2), dhQ(3), dhQ(4), dhQ(5);

    return q;
}

Eigen::Matrix4d Parser::Translation(Eigen::Vector3d p)
{
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity(4, 4);

    t(0, 3) = p(0);
    t(1, 3) = p(1);
    t(2, 3) = p(2);

    return t;
}

Eigen::Matrix4d Parser::Rotation(char axis, double r)
{
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity(4, 4);

    double ct = cos(r);
    double st = sin(r);

    switch (axis)
    {
    case 'x':
        t << 1, 0, 0, 0, 0, ct, -st, 0, 0, st, ct, 0, 0, 0, 0, 1;
        break;
    case 'y':
        t << ct, 0, st, 0, 0, 1, 0, 0, -st, 0, ct, 0, 0, 0, 0, 1;
        break;
    case 'z':
        t << ct, -st, 0, 0, st, ct, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
        break;
    }

    return t;
}

Eigen::VectorXd Parser::DeltaT(Eigen::MatrixXd t1, Eigen::MatrixXd t2)
{
    Eigen::VectorXd diff(6);
    Eigen::VectorXd cross = Eigen::VectorXd::Zero(3);

    for (int i = 0; i < 3; i++)
    {
        Eigen::Vector3d x1, x2;
        x1 << t1(0, i), t1(1, i), t1(2, i);
        x2 << t2(0, i), t2(1, i), t2(2, i);

        cross += x1.cross(x2);
    }

    for (int i = 0; i < 3; i++)
    {
        diff(i) = t2(i, 3) - t1(i, 3);
        diff(i + 3) = 0.5*cross(i);
    }

    return diff;
}

Eigen::MatrixXd Parser::DhFoward(Eigen::MatrixXd dh, Eigen::VectorXd dhQ)
{
    Eigen::Matrix4d t60 = Eigen::Matrix4d::Identity(4, 4);

    for (int i = 0; i < dh.rows(); i++)
    {
        Eigen::Vector3d p;
        p << dh(i, 1), 0, dh(i, 3);
        dh(i, 2) += dhQ(i);

        t60 *= Rotation('z', dh(i, 2)) * Translation(p) * Rotation('x', dh(i, 0));
    }

    return t60;
}

Eigen::MatrixXd Parser::DhJacobn(Eigen::MatrixXd dh, Eigen::VectorXd dhQ)
{
    Eigen::MatrixXd jn = Eigen::MatrixXd::Identity(6, 6);

    Eigen::Matrix4d t = Eigen::Matrix4d::Identity(4, 4);
    for (int i = dh.rows() - 1; i >= 0; i--)
    {
        Eigen::Vector3d pp;
        pp << dh(i, 1), 0, dh(i, 3);
        dh(i, 2) += dhQ(i);
        t = Rotation('z', dh(i, 2)) * Translation(pp) * Rotation('x', dh(i, 0)) * t;

        Eigen::Vector3d a, o, n, p;
        a << t(0, 0), t(1, 0), t(2, 0);
        o << t(0, 1), t(1, 1), t(2, 1);
        n << t(0, 2), t(1, 2), t(2, 2);
        p << t(0, 3), t(1, 3), t(2, 3);

        jn(0, i) = -a(0) * p(1) + a(1) * p(0);
        jn(1, i) = -o(0) * p(1) + o(1) * p(0);
        jn(2, i) = -n(0) * p(1) + n(1) * p(0);
        jn(3, i) = a(2);
        jn(4, i) = o(2);
        jn(5, i) = n(2);
    }

    return jn;
}

Eigen::MatrixXd Parser::DhJacob0(Eigen::MatrixXd dh, Eigen::VectorXd dhQ)
{
    Eigen::MatrixXd j0 = DhJacobn(dh, dhQ);

    Eigen::MatrixXd t0 = DhFoward(dh, dhQ);
    Eigen::MatrixXd t(6, 6);
    t << t0(0, 0), t0(0, 1), t0(0, 2), 0, 0, 0, t0(1, 0), t0(1, 1), t0(1, 2), 0, 0, 0, t0(2, 0), t0(2, 1), t0(2, 2), 0, 0, 0, 0, 0, 0, t0(0, 0), t0(0, 1), t0(0, 2), 0, 0, 0, t0(1, 0), t0(1, 1), t0(1, 2), 0, 0, 0, t0(2, 0), t0(2, 1), t0(2, 2);
    j0 = t * j0;

    return j0;
}

Eigen::VectorXd Parser::DhInverse(Eigen::MatrixXd dh, Eigen::MatrixXd t, Eigen::VectorXd dhQR)
{
    Eigen::VectorXd qR = DhAngle2JointAngle(dhQR);
    Eigen::VectorXd q(6);
    Eigen::VectorXd dhQ = dhQR;
    Eigen::VectorXd deltaQ;

    double norm = 1;
    double min = 1e-12; // Stop condition
    int count = 0;
    int maxCount = 1000; // Max number of iterations

    while (norm > min)
    {
        Eigen::VectorXd e = DeltaT(DhFoward(dh, dhQ), t);
        Eigen::MatrixXd j0 = DhJacob0(dh, dhQ);
        // Only for latest version of Eigen
        // Eigen::VectorXd dDhQ = (j0.pinv()) *e;
        Eigen::VectorXd dDhQ = j0.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(e);

        dhQ += dDhQ;
        for (int i = 0; i < dhQ.rows(); i++) dhQ(i) -= ((int)(dhQ(i) / (2 * Pi))) * 2 * Pi;
        norm = dDhQ.norm();

        q = DhAngle2JointAngle(dhQ);
        deltaQ = q - qR;

        for (int i = 0; i < q.rows(); i++)
        {
            deltaQ(i) -= ((int)(deltaQ(i) / (2 * Pi))) * 2 * Pi;
            if (deltaQ(i) < -Pi) deltaQ(i) += 2 * Pi; else if (deltaQ(i) > Pi) deltaQ(i) -= 2 * Pi;
            if (deltaQ(i) < -Pi / 20 || deltaQ(i) > Pi / 20) norm = 1;
        }

        count++;
        if (count > maxCount) { q << 10 * Pi, 0, 0, 0, 0, 0; return q; }
    }

    return qR + deltaQ;
}