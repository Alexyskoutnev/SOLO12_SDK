/* Copyright 2023, Cinar, A. L.
 * SPDX-License-Identifier: MIT
 */

#ifndef DUMMY_INTERFACE_HPP_CINARAL_230405_1534
#define DUMMY_INTERFACE_HPP_CINARAL_230405_1534

#include "config.hpp"

/** dummy interface to debug on WIN32 without MasterBoardInterface */
class MasterBoardInterface
{
  public:
	MasterBoardInterface(const std::string){};
	void Init(){};
	bool
	IsTimeout()
	{
		return false;
	};
	bool
	IsAckMsgReceived()
	{
		return false;
	};
	void SendInit(){};
	void SendCommand(){};
	void ParseSensorData(){};
	void PrintIMU(){};
	void PrintADC(){};
	void PrintMotors(){};
	void PrintMotorDrivers(){};
	void PrintStats(){};
	class DummyDriver
	{
	  public:
		class DummyMotor
		{
		  public:
			void SetCurrentReference(double){};
			void SetPositionReference(double){};
			void SetVelocityReference(double){};
			void Enable(){};
			void set_kp(double){};
			void set_kd(double){};
			void set_current_sat(double){};
			void set_enable_index_offset_compensation(bool){};
			bool
			get_enable_index_offset_compensation()
			{
				return false;
			};
			void SetPositionOffset(double){};
			bool
			HasIndexBeenDetected()
			{
				return false;
			};
			bool
			IsEnabled()
			{
				return false;
			};
			double
			GetPosition()
			{
				return 0;
			};
			double
			GetVelocity()
			{
				return 0;
			};
		};
		DummyMotor *motor1;
		DummyMotor *motor2;

		void EnablePositionRolloverError(){};
		void SetTimeout(double){};
		void Enable(){};
	};
	DummyDriver::DummyMotor motors[commander::motor_count];
	DummyDriver motor_drivers[commander::driver_count];
};
#endif