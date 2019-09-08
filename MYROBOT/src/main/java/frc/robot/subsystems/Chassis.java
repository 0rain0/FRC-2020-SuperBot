package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.Chassis.Basic_ArcadeDrive;
import frc.robot.commands.Chassis.PID_ArcadeDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.Timer;

public class Chassis extends Subsystem {
  private WPI_TalonSRX Motor_RF = new WPI_TalonSRX(RobotMap.Motor_RA);
  private WPI_TalonSRX Motor_RB = new WPI_TalonSRX(RobotMap.Motor_RB);
  private WPI_TalonSRX Motor_LF = new WPI_TalonSRX(RobotMap.Motor_LA);
  private WPI_TalonSRX Motor_LB = new WPI_TalonSRX(RobotMap.Motor_LB);
  private ADXRS450_Gyro Gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);


  public Chassis(){
    
    Motor_RF.setInverted(RobotMap.Motor_RA_Invert);
    Motor_RB.setInverted(RobotMap.Motor_RB_Invert);
    Motor_LF.setInverted(RobotMap.Motor_LA_Invert);
    Motor_LB.setInverted(RobotMap.Motor_LB_Invert);
    Motor_RB.follow(Motor_RF);
    Motor_LB.follow(Motor_LF);

  }

  public void SetSpeed(double Lspd,double Rspd){
    Motor_RF.set(Rspd*RobotMap.ChassisPowerPercentage);
    //Motor_RB.set(Rspd*RobotMap.ChassisPowerPercentage);
    Motor_LF.set(Lspd*RobotMap.ChassisPowerPercentage);
    //Motor_LB.set(Rspd*RobotMap.ChassisPowerPercentage);
  }

  public void Init_Gryo(){
    Gyro.reset();
    Gyro.calibrate();
  }

  public double Get_Angle(){
    return Gyro.getAngle() % 360.0;
  }
  
  /*
  public double ReadNowAngle(double Value){
    return ((((Value + (360 - InitAngle)) % 360) + 180) % 360) - 180;
  }

  public void DisablePID(){
    PID_Previous_Time = PID_Timer.get();
    Enable_PID = false;
  }

  public void EnablePID(){
    if(Enable_PID == false && PID_Timer.get() > PID_Previous_Time + RobotMap.PID_Enable_Delay){
      Enable_PID = true;
      SetInitAngle(ReadAngle());
    }
  }

  public double PID(double Value,double Kp,double Ki,double Kd){
    if(Enable_PID){
      Error = SetPoint - Value;
      Intergral = Intergral + Error;
      Derivative = Error - Pre_Error;
      Pre_Error = Error;
      return Kp*Error + Ki*Intergral * Kd*Derivative;
    }else{
      return 0;
    }
  }

  public void SetInitPIDVariable(){
    Error = 0;
    Pre_Error = 0;
    Intergral = 0;
    Derivative = 0;
  }
  */
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new PID_ArcadeDrive());
  }
}
