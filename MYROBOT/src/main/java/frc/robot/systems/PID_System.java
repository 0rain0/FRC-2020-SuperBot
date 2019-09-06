package frc.robot.systems;

import edu.wpi.first.wpilibj.DriverStation;

//// Still Testing ////
public class PID_System {
  private boolean Enable_PID = true;
  private boolean Enable_AntiWindUp = false;
  private boolean Enable_AutoStop = false;

  private double AutoStop_SteadyRange = 0;
  private double AutoStop_SteadyTime = 0;

  public void Enable_PID(boolean TrueFalse){
    Enable_PID = TrueFalse;
  }

  public void Enable_AntiWindUp(boolean TrueFalse){
    Enable_AntiWindUp = TrueFalse;
  }

  public void Enable_AutoStop(boolean TrueFalse,double SteadyRange,double SteadyTime){
    if(TrueFalse = true){
      AutoStop_SteadyRange = SteadyRange;
      AutoStop_SteadyTime = SteadyTime;
      if(AutoStop_SteadyRange == 0 || AutoStop_SteadyTime == 0){
        DriverStation.reportWarning("PID_System: AutoStop Parameter Cannot Be 0", true);
        Enable_AutoStop = false;
      }else{
        Enable_AutoStop = true;
      }
    }else{
      Enable_AutoStop = false;
    }
  }

  private double Error = 0;
  private double Previous_Error = 0;
  private double SetPoint = 0;
  private double Intergral = 0;
  private double Derivative = 0;

  /*
  public void Init(){
    Enable_PID = true;
    Enable_Anti_WindUp = false;
    Enable_Auto_Quit = false;
  }
  */

  public void Init_Parameter(){
    Error = 0;
    Previous_Error = 0;
    SetPoint = 0;
    Intergral = 0;
    Derivative = 0;
  }

  
  public double PID_Classic(double Value, double Kp, double Ki, double Kd){
    if(Enable_PID){
      Error = SetPoint - Value;
      Intergral = Intergral + Error;
      Derivative = Error - Previous_Error;
      Previous_Error = Error;
      return Kp * Error + Ki * Intergral + Kd*Derivative;
    }else{
      return 0;
    }
  }

  
  public boolean PID_Finished(){
    if(Enable_AutoStop){
      boolean Steady = (Error > -AutoStop_SteadyRange && Error < AutoStop_SteadyRange);
      //double Steady_Countdown = ()
      return false;
    }else{
      return false;
    }
  }


}
