#include "kdl_planner.h"


KDLPlanner::KDLPlanner(){}

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius)
{
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
}
KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}
void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

void KDLPlanner::trapezoidal_vel(double t,double tc, double &s,double &sdot, double &sdotdot )
{
    // Calcolo il valore costante per l'accelerazione massima (s_dotdot_c) 
    // basato sulla durata totale della traiettoria (trajDuration_)
    double sdotdot_c = 4.5 / (std::pow(trajDuration_, 2));
    // Se il tempo t è inferiore al tempo di accelerazione (tc), 
    if (t <= tc) {
        // Calcolo la posizione (s), velocità (s_d) e accelerazione (s_dd) durante la fase di accelerazione
        s = 0.5 * sdotdot_c * std::pow(t, 2);   // Posizione in funzione del tempo
        sdot = sdotdot_c * t;                    // Velocità in funzione del tempo
        sdotdot = sdotdot_c;                       // Accelerazione costante

    // Se t è compreso tra il tempo di accelerazione e decelerazione, si trova nella fase di velocità costante
    } else if (t <= trajDuration_ - tc) {
        // Durante questa fase, la velocità è costante e l'accelerazione è zero
        s = sdotdot_c * tc * (t - tc / 2);    // Posizione in funzione del tempo, considerando la velocità costante
        sdot = sdotdot_c * tc;                 // Velocità costante
        sdotdot = 0;                            // Accelerazione nulla

    // Se t è maggiore del tempo finale meno tc, si trova nella fase di decelerazione
    } else {
        // Calcolo la posizione (s), velocità (s_d) e accelerazione (s_dd) durante la fase di decelerazione
        s = 1 - 0.5 * sdotdot_c * std::pow(trajDuration_ - t, 2); // Posizione alla fine del movimento
        sdot = sdotdot_c * (trajDuration_ - t);  // Velocità decrescente con il tempo
        sdotdot = -sdotdot_c;                      // Accelerazione negativa durante la decelerazione
    }
}


void KDLPlanner::cubic_polinomial(double t, double &s, double &s_d, double &s_dd) {
  
   // coefficienti derivati da vincoli di posizione e velocità agli estremi del movimento
    double a_0 = 0 ;		//iniziale assunta nulla
    double a_1 = 0; 		//velocita iniziale assunta nulla
    double a_2 = 3 / (std::pow(trajDuration_, 2)); 			  // Coefficiente per il termine t^2 posizione finale =1
    double a_3 = -2 / (std::pow(trajDuration_, 3));  		         // Coefficiente per il termine t^3 velocita  finale =0

    // Calcolo ascissa curvilinea s in funzione del tempo t
    s = a_3 * std::pow(t, 3) + a_2 * std::pow(t, 2) + a_1 * t + a_0 ;
    s_d = 3 * a_3 * std::pow(t, 2) + 2 * a_2 * t + a_1 ;
    s_dd = 6 * a_3 * t + 2 * a_2;
}



trajectory_point KDLPlanner::compute_trajectory(double time, unsigned int k)
{
  double s , s_d , s_dd ;
  trajectory_point traj;
  if ( k==1 || k==3 ) cubic_polinomial (time,s,s_d,s_dd);
  else trapezoidal_vel(time, accDuration_, s, s_d, s_dd);
  
  if ( k == 1 || k == 2 ) { //circular
  	  traj.pos[0]= trajInit_[0];
	  traj.pos[1]= trajInit_[1] -  trajRadius_ * cos(2 * 3.14 * s) + trajRadius_;
	  traj.pos[2]= trajInit_[2] -  trajRadius_ * sin(2 * 3.14 * s);

	  traj.vel[0]= 0;
	  traj.vel[1]=  trajRadius_*2*3.14*s_d*sin(2*3.14*s);
	  traj.vel[2]= -trajRadius_*2*3.14*s_d*cos(2*3.14*s);

	  traj.acc[0]= 0;
	  traj.acc[1]= trajRadius_  * (2 * 3.14) * s_dd * sin(2 * 3.14 * s) + trajRadius_  * (2 * 3.14) * (2 * 3.14) * std::pow(s_d, 2) * cos(2 * 3.14 * s);
	  traj.acc[2]= trajRadius_  * (2 * 3.14) * (2 * 3.14) * std::pow(s_d, 2) * sin(2 * 3.14 * s) - trajRadius_  * (2 * 3.14) * s_dd * cos(2 * 3.14 * s);
  	 
      }
  else if ( k == 3 || k == 4 ) { //linear
	  traj.pos = trajInit_ + s *( trajEnd_ - trajInit_ );
	  traj.vel = s_d *( trajEnd_ - trajInit_ );
	  traj.acc = s_dd *( trajEnd_ - trajInit_ );
	  		
	}
  return traj;
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}









