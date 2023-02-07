// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  private WPI_TalonFX[] rightDriveTalonFX = new WPI_TalonFX[DriveConstants.rightMotorPortID.length];
  private WPI_TalonFX[] leftDriveTalonFX = new WPI_TalonFX[DriveConstants.leftMotorPortID.length];
  private DifferentialDrive drive;

  public DriveSubsystem() {
    System.out.println("Primary Motor - Left " + DriveConstants.leftMotorPortID[0] + " Primary Motor - Right "
        + DriveConstants.rightMotorPortID[0] + " Number of motors per side " + DriveConstants.rightMotorPortID.length);

    // Setting up motors on the left side
    for (int motor = 0; motor < DriveConstants.leftMotorPortID.length; motor++) {
      leftDriveTalonFX[motor] = new WPI_TalonFX(DriveConstants.leftMotorPortID[motor]);
      leftDriveTalonFX[motor].configFactoryDefault(); // reset the motor controller to defaults
      if (motor == 0) { // setup master
        leftDriveTalonFX[motor].set(ControlMode.PercentOutput, 0); // set the motor to Percent Output with Default of 0
        leftDriveTalonFX[motor].setInverted(DriveConstants.MotorInvert[1]);
      } else { // setup followers
        leftDriveTalonFX[motor].follow(leftDriveTalonFX[0]);
        leftDriveTalonFX[motor].setInverted(InvertType.FollowMaster); // set green lights when going forward
        System.out.println("Left Follower " + motor);
      }

      leftDriveTalonFX[motor].setSafetyEnabled(false);  // Disable motor safety - used for Motion Magic and trajectories
    }

    // Setting up motors on the right side
    for (int motor = 0; motor < DriveConstants.rightMotorPortID.length; motor++) {
      rightDriveTalonFX[motor] = new WPI_TalonFX(DriveConstants.rightMotorPortID[motor]);
      rightDriveTalonFX[motor].configFactoryDefault(); // reset the controller to defaults

      if (motor == 0) { // setup master
        rightDriveTalonFX[motor].set(ControlMode.PercentOutput, 0); // set the motor to Percent Output with Default of 0
        rightDriveTalonFX[motor].setInverted( DriveConstants.MotorInvert[0]);
      } else { // setup followers
        rightDriveTalonFX[motor].follow(rightDriveTalonFX[0]);
        rightDriveTalonFX[motor].setInverted(InvertType.FollowMaster); // set green lights when going forward
        System.out.println("Right Follower " + motor);
      }

      rightDriveTalonFX[motor].setSafetyEnabled(false);
    }

    // Engage brake mode
    driveTrainBrakeMode();

    configureEncoders();

    zeroDriveEncoders();

    drive = new DifferentialDrive(leftDriveTalonFX[0], rightDriveTalonFX[0]);
    drive.setSafetyEnabled(false); // safety must be disabled siince we plan to use Motion Magic

  }

  public void driveTrainBrakeMode() {
    for (int motor = 0; motor < DriveConstants.rightMotorPortID.length; motor++) {
      rightDriveTalonFX[motor].setNeutralMode(NeutralMode.Brake);
    }
    for (int motor = 0; motor < DriveConstants.leftMotorPortID.length; motor++) {
      leftDriveTalonFX[motor].setNeutralMode(NeutralMode.Brake);
    }
  }

  public void driveTrainCoastMode() {
    for (int motor = 0; motor < DriveConstants.rightMotorPortID.length; motor++) {
      rightDriveTalonFX[motor].setNeutralMode(NeutralMode.Coast);
    }
    for (int motor = 0; motor < DriveConstants.leftMotorPortID.length; motor++) {
      leftDriveTalonFX[motor].setNeutralMode(NeutralMode.Coast);
    }
  }
  public void manualDrive(double move, double turn) {
    
    // If joysticks will prove to be too sensitive near the center, turn on the deadband driving
    
    // drive.arcadeDrive(deadbandMove(move), deadbandTurn(turn));
    // System.out.println("D X "+move + " Y " + turn);
    //drive.arcadeDrive(move, turn);
    drive.arcadeDrive(move, turn);
  }

  public void stopRobot() {
    leftDriveTalonFX[0].set(TalonFXControlMode.PercentOutput, 0);
    rightDriveTalonFX[0].set(TalonFXControlMode.PercentOutput, 0);
  }

  /** Get the number of tics moved by the left encoder */
  public double getLeftEncoder() {
    return leftDriveTalonFX[0].getSelectedSensorPosition();
  }

  /** Get the number of tics moved by the left encoder */
  public double getRightEncoder() {
    return rightDriveTalonFX[0].getSelectedSensorPosition();
  }

  // Make sure to adjust Units-of-measure if needed
  // The RAW output is "number of ticks per 100ms", so you may need to convert it
  // into m/s
  public double getLeftEncoderSpeed() {
    return leftDriveTalonFX[0].getSelectedSensorVelocity();
  }

  public double getRightEncoderSpeed() {
    return rightDriveTalonFX[0].getSelectedSensorVelocity();
  }

  public void zeroDriveEncoders() {  // zero encoders on master mmotor controllers of the drivetrain

    rightDriveTalonFX[0].setSelectedSensorPosition(0);
    leftDriveTalonFX[0].setSelectedSensorPosition(0);
  }

  // Configure encoders on primary motors
  public void configureEncoders(){
    leftDriveTalonFX[0].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    rightDriveTalonFX[0].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
