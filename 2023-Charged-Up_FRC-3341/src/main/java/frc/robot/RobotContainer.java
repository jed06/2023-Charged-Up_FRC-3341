// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot shoul
 * d be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  public static Joystick joystick2;
  public static Joystick joystick1;
  public static XboxController xbox;
  private final TankDrive tankDrive;
  private final MagicDrive magicDrive;
  private static DriveTrain dt;
  private final AutoTurn turn;
  private final AutoDrive forward;
  private final AutoBalance balance;
  private final Docking dock;
  
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    joystick1 = new Joystick(0);
    joystick2 = new Joystick(1);
    xbox = new XboxController(3);
    configureButtonBindings();
    // Configure the button bindings
    
    dt = new DriveTrain();
    dt.resetEncoders();
    dt.coast();

    tankDrive = new TankDrive(dt, joystick2.getY(), joystick1.getY());
   // dt.setDefaultCommand(tankDrive);
    magicDrive = new MagicDrive(dt, 1.0);
    turn = new AutoTurn(dt, 90);
    forward = new AutoDrive(dt, 5.69);
    balance = new AutoBalance(dt);
    dock = new Docking(dt);
  }
  public static Joystick getJoy1() {
    return joystick1;
  }
  public static Joystick getJoy2() {
    return joystick2;
  }

  public static XboxController getXBox() {
    return xbox;
  }
 

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
   public static DriveTrain getDriveTrain(){
    return dt;
  }
  public Command getAutonomousCommand(){
    return dock;
  }

}