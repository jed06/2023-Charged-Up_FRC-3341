// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;


public class TankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  private final DriveTrain _driveTrain; // declares drive train
  private final double _leftPower; // declares left joystick
  private final double _rightPower; //declares rightjoystick

  public TankDrive(DriveTrain dt, double leftp, double rightp) {
    // Use addRequirements() here to declare subsystem dependencies.
    _driveTrain = dt; //initializes drive train
    _leftPower = leftp; // initializes left joystick
    _rightPower = rightp; // initializes right joystick

    addRequirements(_driveTrain); // tank drive has to use drive train
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _driveTrain.tankDrive(-1*_leftPower, -1*_rightPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}