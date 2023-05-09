/*
 * SOLO12_SDK commander
 *
 * MIT License
 *
 * Copyright (c) 2023 Cinar, A. L.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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