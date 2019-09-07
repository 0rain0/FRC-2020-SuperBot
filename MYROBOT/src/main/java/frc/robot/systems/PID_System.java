package frc.robot.systems;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

//// Still Testing ////
public class PID_System {
  private Timer AutoStop_Timer;
  private Timer System_Timer;

  PID_System(){
    AutoStop_Timer = new Timer();
    AutoStop_Timer.reset();
    AutoStop_Timer.start();

    System_Timer = new Timer();
    System_Timer.reset();
    System_Timer.start();
    SystemTime_Previous = System_Timer.get();
  }

  private boolean Enable_PID = true;
  private boolean Enable_AntiWindUp = false;
  private boolean Enable_AutoStop = false;
  private boolean Enable_TimeOut = false;

  private byte AutoStop_Status = 0;
  private double AutoStop_SteadyRange = 0;// ErrorTolerance
  private double AutoStop_SteadyTime = 0;
  private double AutoStop_PrviousTime = 0;

  private double AntiWindUp_ClampingLimit = 0;

  private double SystemTime_Previous = 0;
  private double SystemTime_Timeout = 0;

  /*
  public void Init(){
    Enable_PID = true;
    Enable_Anti_WindUp = false;
    Enable_Auto_Quit = false;
  }
  
  public void Init_Parameter(){
    Error = 0;
    Previous_Error = 0;
    SetPoint = 0;
    Intergral = 0;
    Derivative = 0;
  }
  */

  public void Enable_PID(boolean TrueFalse){
    Enable_PID = TrueFalse;
  }

  public void Enable_AntiWindUp(boolean TrueFalse,double ClampingLimit){
    if(TrueFalse = true){
      AntiWindUp_ClampingLimit = ClampingLimit;
      if(AntiWindUp_ClampingLimit <= 0){
        DriverStation.reportError("PID_System:AntiWindup Parameter Cannot Be 0 or Smaller", true);
        Enable_AntiWindUp = false;
      }else{
        Enable_AntiWindUp = true;
      }
    }else{
      Enable_AntiWindUp = TrueFalse;
    }
  }

  public void Enable_AutoStop(boolean TrueFalse,double SteadyRange,double SteadyTime){
    if(TrueFalse = true){
      AutoStop_SteadyRange = SteadyRange;
      AutoStop_SteadyTime = SteadyTime;
      if(AutoStop_SteadyRange <= 0 || AutoStop_SteadyTime <= 0){
        DriverStation.reportWarning("PID_System:AutoStop Parameter Cannot Be 0 or Smaller", true);
        Enable_AutoStop = false;
      }else{
        Enable_AutoStop = true;
      }
    }else{
      Enable_AutoStop = false;
    }
  }

  public void Enable_TimeOut(boolean TrueFalse, double TimeOut){
    if(TrueFalse = true){
      SystemTime_Timeout = TimeOut;
      if(SystemTime_Timeout <= 0){
        DriverStation.reportWarning("PID_System:Timeout Parameter Cannot Be 0 or Smaller", true);
        Enable_TimeOut = false;
      }else{
        Enable_TimeOut = true;
      }
    }else{
      Enable_TimeOut = false;
    }
  }

  private double Error = 0;
  private double Previous_Error = 0;
  private double SetPoint = 0;
  private double Intergral = 0;
  private double Derivative = 0;
  
  public double PID(double Value, double Kp, double Ki, double Kd){
    if(Enable_PID){
      Error = SetPoint - Value;
      Intergral = Intergral + Error;
      Derivative = Error - Previous_Error;
      Previous_Error = Error;
      double Pid = Kp * Error + Ki * Intergral + Kd * Derivative;
      if(Enable_AntiWindUp){
        if((Error > 0 && Pid > 0) || (Error < 0 && Pid < 0)){//Same Sign Detect
          if(Math.abs(Pid) > AntiWindUp_ClampingLimit){//Saturation Detect
            Intergral = 0;
            Pid = Kp * Error + Kd * Derivative;
          }
        }
      }
      return Pid;
    }else{
      return 0;
    }
  }

  public boolean PID_Finished(){
    if(Enable_AutoStop){
      double Now_Time = AutoStop_Timer.get();
      boolean Now_Steady = Math.abs(Error) < AutoStop_SteadyRange;
      boolean Now_Steady_Finished = Now_Time - AutoStop_PrviousTime > AutoStop_SteadyTime;
      if(!Now_Steady){
        AutoStop_Status = 0;
      }else if(AutoStop_Status == 0 && Now_Steady){
        AutoStop_Status = 1;
        AutoStop_PrviousTime = Now_Time;
      }else if(AutoStop_Status == 1 && Now_Steady_Finished){
        AutoStop_Status = 0;
        return true;
      }
    }
    if(Enable_TimeOut){
      if((System_Timer.get() - SystemTime_Previous) > SystemTime_Timeout){
        return true;
      }
    }
    return false;
  }
}
