
#include "commander/commander.hpp"

namespace commander
{

Commander::Commander(const std::string ref_traj_fname, const char mb_hostname[], const double kp,
                     const double kd)
    : kp(kp), kd(kd)
{
	// mb(mb_hostname);
	// rt_timer::set_process_priority();

	ref_traj.reserve(t_dim_expected);
	readmatrix(ref_traj_fprefix + ref_traj_fname, ref_traj);

	t_dim = ref_traj.size(); /** determine t_dim */
	traj.reserve(t_dim);
}
Commander::~Commander(){};

class Commander::MbInitSender
{
  public:
	void
	send()
	{
		// mb.SendInit();
	}
};

void
Commander::initialize_mb()
{
	for (size_t i = 0; i < driver_count; ++i) {
		// mb.motor_drivers[i].motor1->SetCurrentReference(0);
		// mb.motor_drivers[i].motor2->SetCurrentReference(0);
		// mb.motor_drivers[i].motor1->Enable();
		// mb.motor_drivers[i].motor2->Enable();
		////* Set the gains for the PD controller running on the cards.
		// mb.motor_drivers[i].motor1->set_kp(kp);
		// mb.motor_drivers[i].motor2->set_kp(kp);
		// mb.motor_drivers[i].motor1->set_kd(kd);
		// mb.motor_drivers[i].motor2->set_kd(kd);
		////* Set the maximum current controlled by the card.
		// mb.motor_drivers[i].motor1->set_current_sat(current_sat);
		// mb.motor_drivers[i].motor2->set_current_sat(current_sat);
		// mb.motor_drivers[i].EnablePositionRolloverError();
		// mb.motor_drivers[i].SetTimeout(timeout);
		// mb.motor_drivers[i].Enable();
	}

	/** create a timer thread to call the action periodically */
	MbInitSender mb_send_init;
	rt_timer::Timer<MbInitSender> init_timer(.1, mb_send_init, &MbInitSender::send);
	rt_timer::TimerThread<MbInitSender> init_thread(init_timer);

	/** start the timer thread for timed duration */
	init_thread.run_for(std::chrono::seconds(masterboard_timeout));

	// auto last = std::chrono::system_clock::now();
	// auto try_wait = 0.1;

	// while (!mb.IsTimeout() && !mb.IsAckMsgReceived()) {
	//	if (((std::chrono::duration<double>)(std::chrono::system_clock::now() - last))
	//	        .count() > try_wait) {
	//		last = std::chrono::system_clock::now();
	//		mb.SendInit();
	//	}
	// }

	// if (mb.IsTimeout()) {
	//	printf("Timeout while waiting for ack.\n");
	// }
}

} // namespace commander