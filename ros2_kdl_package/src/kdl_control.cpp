#include "kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() /*friction compensation?*/;
}

Eigen :: VectorXd KDLController :: idCntr ( KDL :: Frame & _desPos ,
                                            KDL :: Twist & _desVel ,
                                            KDL :: Twist & _desAcc ,
                                            double _Kpp , double _Kpo ,
                                            double _Kdp , double _Kdo)
                                            {



// X_TILDE
     Eigen::Matrix<double,6,1> x_tilde;

     Eigen::Vector3d p_d(_desPos.p.data);
     Eigen::Vector3d p_e(robot_->getEEFrame().p.data); 
     Eigen::Matrix<double,3,1> e_p = computeLinearError(p_d,p_e);

     Eigen::Matrix<double,3,3,Eigen::RowMajor> R_d(_desPos.M.data);
     Eigen::Matrix<double,3,3,Eigen::RowMajor> R_e(robot_->getEEFrame().M.data);
     R_d = matrixOrthonormalization(R_d);
     R_e = matrixOrthonormalization(R_e);
     Eigen::Matrix<double,3,1> e_o = computeOrientationError(R_d,R_e);
  
     x_tilde << e_p, e_o;

// X_TILDE_DOT    
      Eigen::Matrix<double,6,1> x_dot_tilde;

     Eigen::Vector3d p_dot_d(_desVel.vel.data);
     Eigen::Vector3d p_dot_e(robot_->getEEVelocity().vel.data);
     Eigen::Matrix<double,3,1> e_dot_p = computeLinearError(p_dot_d,p_dot_e);

     Eigen::Vector3d w_d(_desVel.rot.data);
     Eigen::Vector3d w_e(robot_->getEEVelocity().rot.data);
     Eigen::Matrix<double,3,1> e_dot_o = computeOrientationVelocityError(w_d, w_e, R_d, R_e);

     x_dot_tilde << e_dot_p, e_dot_o; 


// ACC DESIDERATA
     Eigen::Matrix<double,6,1> x_dotdot_d;
     Eigen::Matrix<double,3,1> p_dotdot_d(_desAcc.vel.data);
     Eigen::Matrix<double,3,1> r_dotdot_d(_desAcc.rot.data);
     x_dotdot_d << p_dotdot_d, r_dotdot_d;

// MATRICI DEI GUADAGNI
     Eigen::Matrix<double,6,6> Kp, Kd;
    Kp=Eigen::MatrixXd::Zero(6,6);
    Kd=Eigen::MatrixXd::Zero(6,6);
     Kp.block(0,0,3,3) = _Kpp*Eigen::Matrix3d::Identity();
     Kp.block(3,3,3,3) = _Kpo*Eigen::Matrix3d::Identity();
     Kd.block(0,0,3,3) = _Kdp*Eigen::Matrix3d::Identity();
     Kd.block(3,3,3,3) = _Kdo*Eigen::Matrix3d::Identity();

//  CALCOLO DELLA Y
    Eigen::Matrix<double,7,1> y;
    Eigen::Matrix<double,6,7> J = robot_->getEEJacobian().data;
    Eigen::Matrix<double,7,7> B = robot_->getJsim();
    Eigen::Matrix<double,7,6> Jpinv = pseudoinverse(J);


    y << Jpinv*(x_dotdot_d - robot_->getEEJacDotqDot() + Kd*x_dot_tilde + Kp*x_tilde);

     return B*y+ robot_->getCoriolis() ;

}


