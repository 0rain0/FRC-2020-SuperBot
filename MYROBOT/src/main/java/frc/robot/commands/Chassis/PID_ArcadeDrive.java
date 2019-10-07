package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Utility;
import frc.robot.systems.PID_System;

//// Still Testing ////
public class PID_ArcadeDrive extends Command {
  public PID_System PID1 = new PID_System();
  public Timer PID1Enable_Timer = new Timer();

  double PID1Enable_PriviousTime = 0;
  boolean PID1Enable = true;

  double Gryo_HeadingAngle = 0;

  public PID_ArcadeDrive() {
    requires(Robot.m_Chassis);
    PID1.Init();
    PID1.Enable_PID(true);
    PID1.Enable_AntiWindUp(true, 1);
    PID1.Enable_AutoStop(false, 0, 0);
    PID1.Enable_TimeOut(false, 0);

    PID1Enable_Timer.reset();
    PID1Enable_Timer.start();
  }

  @Override
  protected void initialize() {
    Gryo_HeadingAngle = Robot.m_Chassis.Get_Angle();
  }
  
  @Override
  protected void execute() {
    double Joystick_Y = Robot.m_Oi.GetAxis(RobotMap.Joystick_LY);
    double Joystick_X = Robot.m_Oi.GetAxis(RobotMap.Joystick_RX);
    boolean Joystick_Y_InDeadZone = false;
    boolean Joystick_X_InDeadZone = false;

    if(RobotMap.Joystick_Y_Invert){
      Joystick_Y = Joystick_Y * -1;
    }
    if(RobotMap.Joystick_X_Invert){
      Joystick_X = Joystick_X * -1;
    }

    //https://www.desmos.com/calculator/epgkans3c0
    if(Joystick_Y >= RobotMap.Joystick_DeadZone && Joystick_Y <= 1){
      Joystick_Y = RobotMap.Joystick_Y_OutPutRate * Math.pow(Math.abs(Joystick_Y),RobotMap.Joystick_Y_Exponential);
    }else if(Joystick_Y <= -RobotMap.Joystick_DeadZone && Joystick_Y >= -1){
      Joystick_Y = RobotMap.Joystick_Y_OutPutRate * -Math.pow(Math.abs(Joystick_Y),RobotMap.Joystick_Y_Exponential);
    }else{
      Joystick_Y = 0;
      Joystick_Y_InDeadZone = true;
    }
    if(Joystick_X >= RobotMap.Joystick_DeadZone && Joystick_X <= 1){
      Joystick_X = RobotMap.Joystick_X_OutPutRate * Math.pow(Math.abs(Joystick_X),RobotMap.Joystick_X_Exponential);
    }else if(Joystick_X <= -RobotMap.Joystick_DeadZone && Joystick_X >= -1){
      Joystick_X = RobotMap.Joystick_X_OutPutRate * -Math.pow(Math.abs(Joystick_X),RobotMap.Joystick_X_Exponential);
    }else{
      Joystick_X = 0;
      Joystick_X_InDeadZone = true;
    }

    double Rspd = 0;
    double Lspd = 0;
    if(Robot.m_Oi.GetButton(RobotMap.Button_Right)){
      Gryo_HeadingAngle = Robot.m_Chassis.Get_Angle();
    }

    if(Joystick_X_InDeadZone == true){
      if(PID1Enable){
        double Gryo = (((((Robot.m_Chassis.Get_Angle() + (360 - Gryo_HeadingAngle)) % 360) + 180) % 360) - 180);
        double Pid = PID1.PID(Gryo, RobotMap.Chassis_Kp, RobotMap.Chassis_Ki, RobotMap.Chassis_Kd);
        Rspd = Joystick_Y - Pid;
        Lspd = Joystick_Y + Pid;
      }else{
        if(PID1Enable_Timer.get() >= PID1Enable_PriviousTime + RobotMap.PIDEnable_Delay){
          PID1Enable = true;
          PID1.Init_Parameter();
          Gryo_HeadingAngle = Robot.m_Chassis.Get_Angle();
        }
        Rspd = Joystick_Y;
        Lspd = Joystick_Y;
      }
    }else{
      PID1Enable = false;
      PID1Enable_PriviousTime = PID1Enable_Timer.get();
      Rspd = Joystick_Y - Joystick_X;
      Lspd = Joystick_Y + Joystick_X;
    }

    Rspd = Utility.Constrain(Rspd, 1, -1);
    Lspd = Utility.Constrain(Lspd, 1, -1);

    Robot.m_Chassis.SetSpeed(Lspd,Rspd);
  }
  
  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    Robot.m_Chassis.SetSpeed(0,0);
  }

  @Override
  protected void interrupted() {
    this.end();
  }
}
