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

    //System.out.println("Y:"+ Joystick_Y +"  X:" + Joystick_X);

    double Rspd = 0;
    double Lspd = 0;
    if(Robot.m_Oi.GetButton(RobotMap.Button_Right)){
      Gryo_HeadingAngle = Robot.m_Chassis.Get_Angle();
    }

    if(Joystick_X_InDeadZone == true){
      if(PID1Enable){
        //System.out.println("1");
        double Gryo = (((((Robot.m_Chassis.Get_Angle() + (360 - Gryo_HeadingAngle)) % 360) + 180) % 360) - 180);
        double Pid = PID1.PID(Gryo, RobotMap.Chassis_Kp, RobotMap.Chassis_Ki, RobotMap.Chassis_Kd);
        //System.out.println(Gryo);
        Rspd = Joystick_Y - Pid;
        Lspd = Joystick_Y + Pid;
        //System.out.println(Pid);
        
      }else{
        //System.out.println("0");
        if(PID1Enable_Timer.get() >= PID1Enable_PriviousTime + RobotMap.PIDEnable_Delay){
          PID1Enable = true;
          PID1.Init_Parameter();
          Gryo_HeadingAngle = Robot.m_Chassis.Get_Angle();
        }
        Rspd = Joystick_Y;
        Lspd = Joystick_Y;
      }
    }else{
      //System.out.println("2");
      PID1Enable = false;
      //PID1.Init_Parameter();
      //Gryo_HeadingAngle = Robot.m_Chassis.Get_Angle();
      //PID1Enable_PriviousTime = PID1Enable_Timer.get();
      Rspd = Joystick_Y - Joystick_X;
      Lspd = Joystick_Y + Joystick_X;
    }

    Rspd = Utility.Constrain(Rspd, 1, -1);
    Lspd = Utility.Constrain(Lspd, 1, -1);

    System.out.println("Rspd:"+Rspd+"  Lspd"+Lspd);
    Robot.m_Chassis.SetSpeed(Lspd,Rspd);
    /*
    double Rspd = 0.0;
    double Lspd = 0.0;

    
    if(Math.abs(Joystick_Y) < RobotMap.Joystick_DeadZone){
      Joystick_Y = 0;
    }
    if(Math.abs(Joystick_X) < RobotMap.Joystick_DeadZone){
      Robot.m_oi.SetRumble(0);
      Robot.m_Chassis.EnablePID();
      Joystick_X = 0;
      double gryo = Robot.m_Chassis.ReadNowAngle(Robot.m_Chassis.ReadAngle());
      double pid = Robot.m_Chassis.PID(gryo, RobotMap.Chassis_Kp, RobotMap.Chassis_Ki,RobotMap.Chassis_Kd);
      Rspd = Joystick_Y + pid;
      Lspd = Joystick_Y - pid;
    }else{
      Robot.m_oi.SetRumble(Math.abs(Joystick_X));
      Rspd = Joystick_Y + Joystick_X;
      Lspd = Joystick_Y - Joystick_X;
      Robot.m_Chassis.DisablePID();
      Robot.m_Chassis.SetInitPIDVariable();
    }
    Rspd = Useful.Constrain(Rspd,1,-1);
    Lspd = Useful.Constrain(Lspd,1,-1);
    //System.out.println(Lspd + "  " + Rspd);
    if(Robot.m_oi.GetAxis(RobotMap.Axis_LT) > 0.75){
      Robot.m_Chassis.SetSpeed(-Lspd*0.5,Rspd*0.5);
    }else{
      Robot.m_Chassis.SetSpeed(-Lspd,Rspd);
    }
  
  */
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
