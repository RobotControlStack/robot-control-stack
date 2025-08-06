xArm7, RobotMetaConfig{
        // q_home (7‐vector):
        (VectorXd(7) << 0, 0, 0, 0, 0, 0, 0).finished(),
        // dof:
        7,
        // joint_limits (2×7):
        (Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::ColMajor>(2, 7) <<
            // low 7‐tuple
            -2 * M_PI, -2.094395, -2 * M_PI, -3.92699, -2 * M_PI, -M_PI, -2 * M_PI,
            // high 7‐tuple
             2 * M_PI,  2.059488,  2 * M_PI,  0.191986, 2 * M_PI, 1.692969, 2 * M_PI).finished()