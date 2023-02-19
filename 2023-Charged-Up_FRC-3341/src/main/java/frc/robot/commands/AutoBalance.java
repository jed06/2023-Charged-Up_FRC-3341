// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  DriveTrain dt;
  //double maxPower = 0.5; //if the robot was vertical(theoretically) the talons would go at this power 
  double baseSpeed;
  double angleThreshhold;
  double previousAngle;
  double currentAngle;

  PIDController pid;

  public AutoBalance( DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt = dt;
    baseSpeed = 0.33;
    pid = new PIDController(0.7, 0.6, 0);
    angleThreshhold = Constants.OperatorConstants.angleThreshhold;
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetEncoders();
    pid.setSetpoint(Constants.OperatorConstants.balanceDistance);
    currentAngle = dt.getYAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    previousAngle = currentAngle;
    currentAngle = dt.getYAngle();
    /* 
    if((previousAngle > 0 && currentAngle < 0) || (previousAngle < 0 && currentAngle > 0)) baseSpeed -= 0.005;

    double speed;
    if(dt.getYAngle() > angleThreshhold) speed = baseSpeed;
    else if(dt.getYAngle() < -1 * angleThreshhold) speed = -baseSpeed;
    else speed = 0;
*/
    double speed;
    speed = pid.calculate(dt.getDisplacement());
    SmartDashboard.putNumber("Docking Speed: ", speed);
    dt.tankDrive(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return .03 >= Math.abs(Constants.OperatorConstants.balanceDistance - dt.getDisplacement());
    return false;
    //return 3 >= Math.abs(dt.getYAngle());
  }
}
