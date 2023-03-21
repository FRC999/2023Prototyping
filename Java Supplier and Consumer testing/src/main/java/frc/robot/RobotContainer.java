// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Command1Ending;
import frc.robot.commands.Command2Ending;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TestPrint1;
import frc.robot.commands.TestSequence;
import frc.robot.commands.TestSequenceInterruption;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Subsystem1;
import frc.robot.subsystems.Subsystem2;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public static Joystick driveStick;
  double a = 1;

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public static final Subsystem1 s1 = new Subsystem1();
  public static final Subsystem2 s2 = new Subsystem2();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    driveStick = new Joystick(0);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    new JoystickButton(driveStick, 9)
      .onTrue(new TestSequence(()->a));
      //.onTrue(new PrintCommand("TEST"));


    new JoystickButton(driveStick, 10)
      .onTrue(new TestPrint1(()->a));

    new JoystickButton(driveStick, 8)
    .onTrue(new TestPrint1(()->Double.NaN));

    //new JoystickButton(driveStick, 11)
    //  .onTrue(new TestPrint1(5));

    //new JoystickButton(driveStick, 8)
    //  .onTrue(new TestPrint1(a));

    //new JoystickButton(driveStick, 7)
    //  .onTrue(new TestSequenceInterruption());

    //new JoystickButton(driveStick, 8)
    //  .onTrue(new Command1Ending());

    //new JoystickButton(driveStick, 9)
    //  .onTrue(new Command2Ending());

    new JoystickButton(driveStick, 12)
      .onTrue(new InstantCommand(()->increaseA()));
  }

  public void increaseA() {
    a = a+1;
    System.out.println("New A:"+a);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
