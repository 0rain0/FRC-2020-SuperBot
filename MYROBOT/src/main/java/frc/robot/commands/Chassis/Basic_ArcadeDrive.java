package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Utility;

public class Basic_ArcadeDrive extends Command {
  public Basic_ArcadeDrive() {
    requires(Robot.m_Chassis);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    double Joystick_Y = Robot.m_Oi.GetAxis(RobotMap.Joystick_LY);
    double Joystick_X = Robot.m_Oi.GetAxis(RobotMap.Joystick_RX);

    if(RobotMap.Joystick_Y_Invert){
      Joystick_Y = Joystick_Y * -1;
    }
    if(RobotMap.Joystick_X_Invert){
      Joystick_X = Joystick_X * -1;
    }

    System.out.println(Joystick_X);

    //https://www.desmos.com/calculator/epgkans3c0
    if(Joystick_Y > 0 && Joystick_Y < 1){
      Joystick_Y = RobotMap.Joystick_Y_OutPutRate * Math.pow(Math.abs(Joystick_Y),RobotMap.Joystick_Y_Exponential);
    }else if(Joystick_Y < 0 && Joystick_Y > -1){
      Joystick_Y = RobotMap.Joystick_Y_OutPutRate * -Math.pow(Math.abs(Joystick_Y),RobotMap.Joystick_Y_Exponential);
    }
    if(Joystick_X > 0 && Joystick_X < 1){
      Joystick_X = RobotMap.Joystick_X_OutPutRate * Math.pow(Math.abs(Joystick_X),RobotMap.Joystick_X_Exponential);
    }else if(Joystick_X < 0 && Joystick_X > -1){
      Joystick_X = RobotMap.Joystick_X_OutPutRate * -Math.pow(Math.abs(Joystick_X),RobotMap.Joystick_X_Exponential);
    }

    double Rspd = Joystick_Y - Joystick_X;
    double Lspd = Joystick_Y + Joystick_X;

    Rspd = Utility.Constrain(Rspd,1,-1);
    Lspd = Utility.Constrain(Lspd,1,-1);
    
    Robot.m_Chassis.SetSpeed(Lspd,Rspd);
  }

  @Override
  protected boolean isFinished(){
    return false;
  }

  @Override
  protected void end(){
    Robot.m_Chassis.SetSpeed(0,0);
  }

  @Override
  protected void interrupted(){
    this.end();
  }
}
