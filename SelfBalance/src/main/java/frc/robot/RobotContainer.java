// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.BalanceOnCharger;
import frc.robot.commands.DriveForwardUntilPitch;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PigeonIMUSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static Joystick driveStick = new Joystick(0);

  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();

  public static final PigeonIMUSubsystem pigeonIMUSubsystem = new PigeonIMUSubsystem();

  public static final SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    driveSubsystem.setDefaultCommand(new DriveManuallyCommand());
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    new JoystickButton(driveStick, 11)
      .whileTrue(new DriveForwardUntilPitch(0.6, 15, true))
      .whileFalse(new InstantCommand(driveSubsystem::stopRobot, driveSubsystem))
      ;
    
    new JoystickButton(driveStick, 12)
      .whileTrue(new BalanceOnCharger(0))
      .whileFalse(new InstantCommand(driveSubsystem::stopRobot))
      ;
  }

}
