// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;


public class MagicDrive extends CommandBase {
  /** Creates a new MagicDrive. */
public double displacement;
private final DriveTrain dt;
  public MagicDrive(DriveTrain dt, double displacement) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt= dt;
    this.displacement= displacement;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dt.magicDrive(displacement);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(dt.getDisplacement()) >=  Math.abs(displacement);
  }
}
