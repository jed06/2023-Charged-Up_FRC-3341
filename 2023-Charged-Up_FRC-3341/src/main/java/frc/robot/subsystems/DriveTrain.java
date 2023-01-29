// Based upon 2021's Competition Season DriveTrain code

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Robot;

import com.kauailabs.navx.frc.AHRS;
//comment


public class DriveTrain extends SubsystemBase 
{
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;
  private final VictorSPX _leftDriveVictor;
  private final VictorSPX _rightDriveVictor;
  private final AHRS navX;
  public boolean logOverride = false;
  public DoubleLogEntry anglelog;

  
  public DriveTrain() 
  {
    DataLogManager.start();
    DataLog log = DataLogManager.getLog();
    anglelog = new DoubleLogEntry(log, "/my/double");

    
    leftDriveTalon = new WPI_TalonSRX(Constants.OperatorConstants.LeftDriveTalonPort);
    rightDriveTalon = new WPI_TalonSRX(Constants.OperatorConstants.RightDriveTalonPort);
    _leftDriveVictor = new VictorSPX(Constants.OperatorConstants.LeftDriveVictorPort);
    _rightDriveVictor = new VictorSPX(Constants.OperatorConstants.RightDriveVictorPort);
    navX = new AHRS(SPI.Port.kMXP);

    _leftDriveVictor.follow(leftDriveTalon);
    _rightDriveVictor.follow(rightDriveTalon);
  
    leftDriveTalon.setNeutralMode(NeutralMode.Coast);
    rightDriveTalon.setNeutralMode(NeutralMode.Coast);

    leftDriveTalon.setInverted(false);
    rightDriveTalon.setInverted(true);
    _leftDriveVictor.setInverted(InvertType.FollowMaster);
    _rightDriveVictor.setInverted(InvertType.FollowMaster);

    leftDriveTalon.setSensorPhase(true);
    rightDriveTalon.setSensorPhase(true);

    leftDriveTalon.configFactoryDefault();
    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightDriveTalon.configFactoryDefault();
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    
    // Week 4 Motion Magic
   leftDriveTalon.config_kP(0, 2.5, 10);
   leftDriveTalon.config_kI(0, 0, 10);
   leftDriveTalon.config_kD(0, 0, 10);
   leftDriveTalon.configMotionAcceleration(328, 10);
   leftDriveTalon.configMotionCruiseVelocity(400, 10);
   // If Left Velocity is 200
   // then Left Accel 120

   rightDriveTalon.config_kP(0, 2.5, 10);
   rightDriveTalon.config_kI(0, 0, 10);
   rightDriveTalon.config_kD(0, 0, 10);
   rightDriveTalon.configMotionAcceleration(2000, 10);
   rightDriveTalon.configMotionCruiseVelocity(386, 10);

  }

  public void magicDrive(double displacement){
    leftDriveTalon.set(ControlMode.MotionMagic, Constants.OperatorConstants.tickstoMeters*displacement);
    rightDriveTalon.set(ControlMode.MotionMagic, Constants.OperatorConstants.tickstoMeters*displacement);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    leftDriveTalon.set(leftSpeed);
    rightDriveTalon.set(rightSpeed);
  }

  
  public void resetEncoders() {
    leftDriveTalon.setSelectedSensorPosition(0,0,10);
    rightDriveTalon.setSelectedSensorPosition(0,0,10);
  }

  public double getTicks() {
    return (leftDriveTalon.getSelectedSensorPosition(0) + rightDriveTalon.getSelectedSensorPosition(0)) / 2.0;
  }

  public double getDisplacement(){
    return (getTicks()/Constants.OperatorConstants.tickstoMeters);
  }

  public double getYAngle(){
    return navX.getPitch();
  }
  @Override
  public void periodic() {
    tankDrive(RobotContainer.getJoy1().getY()*-0.2, RobotContainer.getJoy2().getY()*-0.2);
    if (RobotContainer.getJoy1().getRawButtonReleased(12)){
      logOverride = !logOverride;
    }
    if (logOverride){
      anglelog.append(getYAngle());
    }
  }

  @Override
  public void simulationPeriodic() 
  {
    // This method will be called once per scheduler run during simulation
  }
}