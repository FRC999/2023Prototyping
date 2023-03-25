// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestSequence extends SequentialCommandGroup {

  DoubleSupplier ds;
  /** Creates a new TestSequence. */
  public TestSequence(DoubleSupplier d) {
    ds = d;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands (
    new ConditionalCommand(new PrintCommand("Less " + ds.getAsDouble()), 
        new PrintCommand("More " + ds.getAsDouble()),
         () -> (ds.getAsDouble()<3)),
    new ConditionalCommand(new PrintCommand("Less " + ds.getAsDouble()), 
        new InstantCommand(this::cancel),
          () -> ds.getAsDouble()<3),
    new ConditionalCommand(new PrintCommand("Less " + ds.getAsDouble()), 
        new PrintCommand("More " + ds.getAsDouble()),
           () -> ds.getAsDouble()<3)
    );
  }

}