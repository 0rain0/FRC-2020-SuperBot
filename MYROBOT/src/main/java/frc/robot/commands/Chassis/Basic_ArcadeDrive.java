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
    double Joystick_LY = Robot.m_Oi.GetAxis(RobotMap.Joystick_LY);
    double Joystick_RX = Robot.m_Oi.GetAxis(RobotMap.Joystick_RX);

    if(RobotMap.Joystick_LY_Invert){
      Joystick_LY = Joystick_LY * -1;
    }
    if(RobotMap.Joystick_RX_Invert){
      Joystick_RX = Joystick_RX * -1;
    }

    //https://www.desmos.com/calculator/epgkans3c0
    if(Joystick_LY > 0 && Joystick_LY < 1){
      Joystick_LY = RobotMap.Joystick_LY_OutPutRate * Math.pow(Math.abs(Joystick_LY),RobotMap.Joystick_LY_Exponential);
    }else if(Joystick_LY < 0 && Joystick_LY > -1){
      Joystick_LY = RobotMap.Joystick_LY_OutPutRate * -Math.pow(Math.abs(Joystick_LY),RobotMap.Joystick_LY_Exponential);
    }
    if(Joystick_RX > 0 && Joystick_RX < 1){
      Joystick_RX = RobotMap.Joystick_RX_OutPutRate * Math.pow(Math.abs(Joystick_RX),RobotMap.Joystick_RX_Exponential);
    }else if(Joystick_RX < 0 && Joystick_RX > -1){
      Joystick_RX = RobotMap.Joystick_RX_OutPutRate * -Math.pow(Math.abs(Joystick_RX),RobotMap.Joystick_RX_Exponential);
    }

    double Rspd = Joystick_LY - Joystick_RX;
    double Lspd = Joystick_LY + Joystick_RX;

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
