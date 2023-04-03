#include "commander.hpp"

using commander::Commander;

int
main(int argc, char *argv[])
{
	//* give the process a high priority
	// nice(-20);
	// rt_timer::set_process_priority();

	Commander com;

	
	// matrix_rw::Reader<traj_dim> readmatrix;
	// VarRowMat_T<traj_dim> ref_traj;
	// ref_traj.reserve(t_dim_expected);
	// readmatrix(fprefix + fname, ref_traj); /** read from file */

	return 0;
}