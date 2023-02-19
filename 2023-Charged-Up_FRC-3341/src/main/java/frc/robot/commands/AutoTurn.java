// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoTurn extends CommandBase {
  /** Creates a new AutoTurn. */
  DriveTrain dt;
  double angle;
  PIDController pidController = new PIDController(0.0075, 0.01, 0);
  public AutoTurn(DriveTrain dt, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt = dt;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetNavX();

    pidController.setSetpoint(dt.getAngle() + angle);
    pidController.setTolerance(1);
    //dt.setPowerLimits(0.7);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(dt.getAngle());
    dt.tankDrive(-0.7*speed, 0.7*speed); //right motor is 1.32 times slower than left
    //dt.tankDrive(0.1*speed, 0.1*speed * 1.32);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0, 0);
    dt.setPowerLimits(1.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
