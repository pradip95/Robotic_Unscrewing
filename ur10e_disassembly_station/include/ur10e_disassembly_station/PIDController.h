/**
 * @file 
 * @brief A typical PID controller
 * @author yuhanjin89@gmail.com
 * 
 */
#include <cmath>
class PIDController 
{
public:
  
  PIDController(double kp, double ki, double kd, double setpoint, double tolerance);

  /**
   * @brief set setpoint of the PID controller
   * 
   * @param setpoint can be any real variable
   */
  void setSetpoint(double setpoint);

  /**
   * @brief update the PID controller, need to be called once at every system loop
   * 
   * @param input system input read from outside
   * @param dt delta T
   * @param ouput_limit boundary for output
   * @return double ouput value
   */
  double update(double input, double dt, double ouput_limit);

  /**
   * @brief set the Kp value
   * 
   * @param kp Kp
   */
  void setKp(double kp);

  /**
   * @brief set the Ki value
   * 
   * @param ki Kp
   */
  void setKi(double ki);

  /**
   * @brief set the Kd value
   * 
   * @param kd Kd
   */
  void setKd(double kd);

private:
  /**
   * @brief proportional gain
   * 
   */
  double m_kp;          

  /**
   * @brief integral gain
   * 
   */
  double m_ki;         

  /**
   * @brief derivative gain
   * 
   */
  double m_kd;         

  /**
   * @brief target setpoint i.e. control goal for the system
   * 
   */
  double m_setpoint;    

  /**
   * @brief tolerance for target
   * 
   */
  double m_tolerance;

  /**
   * @brief integral accumulater 
   * 
   */
  double m_integral;    

  /**
   * @brief previous error
   * 
   */
  double m_prevError;   
};
