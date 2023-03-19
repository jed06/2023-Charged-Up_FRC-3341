// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutoDrive extends CommandBase {
  /** Creates a new AutoDrive. */
  DriveTrain dt;
  PIDController pid;
  double speed;
  double distance;

  public AutoDrive(DriveTrain dt, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt = dt;
    pid = new PIDController(0.4, 0, 0);
    speed = 0.55;
    this.distance = distance;
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetEncoders();
    pid.setSetpoint(distance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //speed = pid.calculate(dt.getDisplacement());
    dt.tankDrive(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return 0.1 >= Math.abs(distance - dt.getDisplacement()) || Constants.OperatorConstants.angleThreshhold + 3 <= Math.abs(dt.getYAngle());
  }
}
