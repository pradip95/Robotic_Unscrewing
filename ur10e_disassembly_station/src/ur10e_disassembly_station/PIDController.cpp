#include "ur10e_disassembly_station/PIDController.h"

PIDController::PIDController(double kp, double ki, double kd, double setpoint, double tolerance)
  : m_kp(kp)
  , m_ki(ki)
  , m_kd(kd)
  , m_setpoint(setpoint)
  , m_tolerance(tolerance)
  , m_integral(0)
  , m_prevError(0)
{
}
//-------------------------------------------------------------------------------------
// setters
//-------------------------------------------------------------------------------------
void PIDController::setSetpoint(double setpoint)
{
  m_setpoint = setpoint;
}

void PIDController::setKp(double kp)
{
  m_kp = kp;
}

void PIDController::setKi(double ki)
{
  m_ki = ki;
}

void PIDController::setKd(double kd)
{
  m_kd = kd;
}

//-------------------------------------------------------------------------------------
// updating PID controller
//-------------------------------------------------------------------------------------
double PIDController::update(double input, double dt, double output_limit)
{
  //-------------------------------------------------------------------------------------
  // check if the input already meets the tolerance
  //-------------------------------------------------------------------------------------
  double error = m_setpoint - input; 
  if (fabs(error) < m_tolerance)
  {
    return 0.0;
  }
  //-------------------------------------------------------------------------------------
  // update values
  //-------------------------------------------------------------------------------------
  double dError = (error - m_prevError) / dt; 
  m_integral += error * dt;                   
  m_prevError = error;                       
  double output = m_kp * error + m_ki * m_integral + m_kd * dError; // PID

  //-------------------------------------------------------------------------------------
  // bound the ouput inside output_limit 
  //-------------------------------------------------------------------------------------
  if (output > output_limit)
  {
    output = output_limit;
  }

  if (output < -output_limit)
  {
    output = -output_limit;
  }
  //-------------------------------------------------------------------------------------
  // finally return the output of this step
  //-------------------------------------------------------------------------------------
  return output;
}