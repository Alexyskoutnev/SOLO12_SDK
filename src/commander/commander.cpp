
#include "commander/commander.hpp"

namespace commander
{

Commander::Commander(const std::string ref_traj_fname, const char mb_hostname[], const double kp,
                     const double kd)
    : kp(kp), kd(kd)
{
	// mb(mb_hostname);
	
	ref_traj.reserve(t_dim_expected);
	readmatrix(ref_traj_fprefix + ref_traj_fname, ref_traj);

	t_dim = ref_traj.size(); /** determine t_dim */
	traj.reserve(t_dim);
}
Commander::~Commander(){};

} // namespace commander