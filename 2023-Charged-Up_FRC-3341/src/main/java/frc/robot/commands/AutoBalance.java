// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  DriveTrain dt;
  //double maxPower = 0.5; //if the robot was vertical(theoretically) the talons would go at this power 
  double maxSpeed;
  double balanceDistance;
  double angleThreshhold;
  double initialYaw;
  double waitTime;
  boolean nearTop = false;
  boolean isbalanced =false;

  double previousAngle = 0;
  double currentAngle = 0;
  double deltaAngle = 0;

  PIDController pid;
  PIDController yawPID;

  Timer timer;
  Timer executeTimer;

  public AutoBalance( DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt = dt;
    maxSpeed = 0.6;
    balanceDistance = 1.1;
    waitTime = 0.5;
    pid = new PIDController(0.2, 0.3, 0.2);
    yawPID = new PIDController(0.02, 0, 0);
    angleThreshhold = Constants.OperatorConstants.angleThreshhold;
    timer = new Timer();
    executeTimer = new Timer();
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetEncoders();
    pid.setSetpoint(balanceDistance);
    yawPID.setSetpoint(dt.getAngle());
    timer.start();
    executeTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(dt.getDisplacement() > 1) maxSpeed = 0.35;
    else if(dt.getDisplacement() > .5) maxSpeed = 0.4;

    currentAngle = dt.getYAngle();
    deltaAngle = (currentAngle - previousAngle) / executeTimer.get();

    double pidSpeed = pid.calculate(dt.getDisplacement());
    double bootlegPID = 2 * (balanceDistance - dt.getDisplacement());
    double speed = Math.min(Math.max(Math.abs(bootlegPID), Math.abs(pidSpeed)), maxSpeed) * Math.abs(pidSpeed)/pidSpeed;
    double turningSpeed = yawPID.calculate(dt.getAngle());
    double minSpeed = 0.3 * Math.abs(dt.getYAngle())/dt.getYAngle();
    
    SmartDashboard.putNumber("PID Speed: ", pidSpeed);
    SmartDashboard.putNumber("Docking Speed: ", speed);
    SmartDashboard.putNumber("DockDistance: ", balanceDistance);
    SmartDashboard.putNumber("Timer: ", timer.get());
    SmartDashboard.putNumber("Delta Angle: ", deltaAngle);
    SmartDashboard.putBoolean("Balanced: ", isbalanced);

    if(!isbalanced){
      if(angleThreshhold > Math.abs(dt.getYAngle())){
        isbalanced = true;
        timer.reset();
        dt.brake();
      }else if(0.01 >= Math.abs(balanceDistance - dt.getDisplacement()) && angleThreshhold < Math.abs(dt.getYAngle())){
        if(timer.get() > waitTime){
          balanceDistance += 0.01 * Math.abs(dt.getYAngle())/dt.getYAngle();
          pid.setSetpoint(balanceDistance);
          timer.reset();
          waitTime += 0.05;
        }else{
          dt.tankDrive(minSpeed, minSpeed);
        }
      }else {
        dt.tankDrive(speed + turningSpeed, speed - turningSpeed);
      }
    }else{
      if(dt.getDisplacement() > 1 && dt.getDisplacement() < 2) dt.tankDrive(0, 0);
      if(angleThreshhold < Math.abs(dt.getYAngle())){
        dt.coast();
        isbalanced = false;
      }
    }

    previousAngle = currentAngle;
    executeTimer.reset();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0, 0);
    dt.coast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return .03 >= Math.abs(Constants.OperatorConstants.balanceDistance - dt.getDisplacement());
    //return false;
    return angleThreshhold >= Math.abs(dt.getYAngle()) && timer.get() > 2;
  }
}
