package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
//import frc.robot.commands.Chassis.Basic_ArcadeDrive;
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
  }

  public void SetSpeed(double Lspd,double Rspd){
    Motor_RF.set(Rspd*RobotMap.ChassisPowerPercentage);
    Motor_RB.set(Rspd*RobotMap.ChassisPowerPercentage);
    Motor_LF.set(Lspd*RobotMap.ChassisPowerPercentage);
    Motor_LB.set(Lspd*RobotMap.ChassisPowerPercentage);
  }

  public void SetSeparateSpeed(double RF,double LF,double LB,double RB){
    Motor_RF.set(RF*RobotMap.ChassisPowerPercentage);
    Motor_LF.set(LF*RobotMap.ChassisPowerPercentage);
    Motor_LB.set(LB*RobotMap.ChassisPowerPercentage);
    Motor_RB.set(RB*RobotMap.ChassisPowerPercentage);
  }

  public void Init_Gryo(){
    Gyro.reset();
    Gyro.calibrate();
  }

  public double Get_Angle(){
    return Gyro.getAngle() % 360.0;
  }
  
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new PID_ArcadeDrive());
  }
}
