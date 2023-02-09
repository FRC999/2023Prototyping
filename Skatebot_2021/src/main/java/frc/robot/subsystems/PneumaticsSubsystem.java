// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PneumaticsConstants;

public class PneumaticsSubsystem extends SubsystemBase {

    Compressor compressor ;
    DoubleSolenoid doubleSolenoid ;
    
  /** Creates a new Pneumatics. */
  public PneumaticsSubsystem() {

    compressor = new Compressor(PneumaticsConstants.PNEUMATICS_MODULE, PneumaticsModuleType.CTREPCM);
    doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.FORWARD_CHANNEL, PneumaticsConstants.REVERSE_CHANNEL);

    System.out.println("**** Activating compressor");
    activateCompressor();
 }

  private void activateCompressor() {
    compressor.enableDigital();
  }

  private void deactivateCompressor() {
    compressor.disable();
  }

  public void extendCylinder() {
    doubleSolenoid.set(Value.kForward);
  }

  public void retractCylinder() {
    doubleSolenoid.set(Value.kReverse);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
