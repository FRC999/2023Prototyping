// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestPrint1 extends CommandBase {
  DoubleSupplier p1;
  double p1s;
  /** Creates a new TestPrint1. */
  public TestPrint1(double p) {
    // Use addRequirements() here to declare subsystem dependencies.
    p1s = p;
  }
  public TestPrint1(DoubleSupplier param1) {
    // Use addRequirements() here to declare subsystem dependencies.
    p1 = param1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (p1 != null) {
      p1s=p1.getAsDouble();
    }
    System.out.println("P1:"+p1s);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
