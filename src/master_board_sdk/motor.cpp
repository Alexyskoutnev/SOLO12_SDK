#include <stdio.h>
#include "master_board_sdk/motor.h"
#include <ncurses.h>

Motor::Motor()
{
  position = 0;
  position_offset = 0;
  velocity = 0;
  current = 0;
  is_enabled = false;
  is_ready = false;
  index_toggle_bit = false;
  has_index_been_detected = false;

  enable = false;
  enable_position_rollover_error = false;
  enable_index_toggle_bit = false;
  enable_index_offset_compensation = false;
}

void Motor::SetDriver(MotorDriver *driver)
{
  this->driver = driver;
}

void Motor::Print()
{
  printw("%7d | ", is_enabled);
  printw("%5d | ", is_ready);
  printw("%4d | ", index_toggle_bit);
  printw("%9d | ", has_index_been_detected);
  printw("%13e | ", position);
  printw("%13e | ", velocity);
  printw("%13e | ", current);
  printw("\n");
}

void Motor::Enable()
{
  enable=true;
}

void Motor::Disable()
{
  enable=false;
}

void Motor::SetPositionOffset(double offset)
{
  position_offset = offset;
}

void Motor::SetPositionReference(double ref)
{
  position_ref = ref;
}

void Motor::SetVelocityReference(double ref)
{
  velocity_ref = ref;
}

void Motor::SetCurrentReference(double ref)
{
  current_ref = ref;
}

void Motor::SetKp(double val)
{
  kp = val;
}

void Motor::SetKd(double val)
{
  kd = val;
}

void Motor::SetSaturationCurrent(double val)
{
  current_sat = val;
}

bool Motor::IsReady()
{
  return is_ready;
}

bool Motor::IsEnabled()
{
  return is_enabled;
}

bool Motor::HasIndexBeenDetected()
{
  return has_index_been_detected;
}

bool Motor::GetIndexToggleBit()
{
  return index_toggle_bit;
}

double Motor::GetPosition()
{
  return position;
}

double Motor::GetPositionOffset()
{
  return position_offset;
}

double Motor::GetVelocity()
{
  return velocity;
}

double Motor::GetCurrent()
{
  return current;
}
