// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.AbstractDocument.LeafElement;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {

  WPI_TalonFX leftmotor = new WPI_TalonFX(9);
  WPI_TalonFX rightmotor = new WPI_TalonFX(10);

  public DifferentialDrive drive;
  TalonFXConfiguration leftConfig;
  TalonFXConfiguration rightConfig;

  final double clicksPerFoot = 120615/10;
  public final int tickPerInch = (int)(clicksPerFoot / 12); // (int) (2048/(4*Math.PI));
  public final int tolerance = 1*tickPerInch;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    configureEncoders();
    configureSimpleMagic();
    brakeMode();
    zeroEncoders();
    drive = new DifferentialDrive(leftmotor, rightmotor);
    drive.setSafetyEnabled(false);
    leftConfig = new TalonFXConfiguration();
	  rightConfig = new TalonFXConfiguration();
  }

  public void brakeMode() {
    leftmotor.setNeutralMode(NeutralMode.Brake);
    rightmotor.setNeutralMode(NeutralMode.Brake);
  }

  public void arcadeDriving(double move, double turn){
    drive.arcadeDrive(move, turn);
  }

   /** Get the number of tics moved by the left encoder */
   public int getLeftEncoder() {
    return (int) leftmotor.getSelectedSensorPosition();
  }

  /** Get the number of tics moved by the left encoder */
  public int getRightEncoder() {
    return (int) rightmotor.getSelectedSensorPosition();
  }

  public double getLeftError() {
    return leftmotor.getClosedLoopError();// Returns the PID error for Pan motion control;
  }

  public double getRightError() {
    return rightmotor.getClosedLoopError();
  }

  public double getLeftTarget() {
    return leftmotor.getClosedLoopTarget();// Returns the PID error for Pan motion control;
  }

  public double getRightTarget() {
    return rightmotor.getClosedLoopTarget();
  }

  public void zeroEncoders(){
    leftmotor.setSelectedSensorPosition(0);
    rightmotor.setSelectedSensorPosition(0);
    System.out.println("encoders zeroed");
  }

  public void configureEncoders(){
    leftmotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    rightmotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
  }

  public void manualDrive(double move, double turn) {
    drive.arcadeDrive(move,turn);
  }

  public void configureSimpleMagic() {
      leftmotor.setSafetyEnabled(false);
      rightmotor.setSafetyEnabled(false);

      /* Configure motor neutral deadband */
      rightmotor.configNeutralDeadband(0.001, 30);
      leftmotor.configNeutralDeadband(0.001, 30);

      leftmotor.setSensorPhase(true);
      rightmotor.setSensorPhase(true);

      leftmotor.setInverted(false);
      rightmotor.setInverted(true);

      /* Set status frame periods to ensure we don't have stale data */
      
      rightmotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10,30);
      leftmotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10,30);
      rightmotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10,30);
      leftmotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10,30);

      leftmotor.configPeakOutputForward(+1.0, 30);
      leftmotor.configPeakOutputReverse(-1.0, 30);
      leftmotor.configNominalOutputForward(0, 30);
      leftmotor.configNominalOutputReverse(0, 30);

      rightmotor.configPeakOutputForward(+1.0, 30);
      rightmotor.configPeakOutputReverse(-1.0, 30);
      rightmotor.configNominalOutputForward(0, 30);
      rightmotor.configNominalOutputReverse(0, 30);      
      
      /* FPID Gains for each side of drivetrain */
      leftmotor.selectProfileSlot(0, 0);
      leftmotor.config_kP(0, 0.75, 30);
      leftmotor.config_kI(0, 0.005, 30);
      leftmotor.config_kD(0, 0.01,  30);
      leftmotor.config_kF(0, 0, 30);

      leftmotor.config_IntegralZone(0, 500,  30);
      leftmotor.configClosedLoopPeakOutput(0, 0.5, 30);
      leftmotor.configAllowableClosedloopError(0, 5, 30);
      

      rightmotor.selectProfileSlot(0, 0);
      rightmotor.config_kP(0, 0.75, 30);
      rightmotor.config_kI(0, 0.005, 30);
      rightmotor.config_kD(0, 0.01, 30);
      rightmotor.config_kF(0, 0, 30);

      rightmotor.config_IntegralZone(0, 5000, 30);
      rightmotor.configClosedLoopPeakOutput(0, 0.5, 30);
      rightmotor.configAllowableClosedloopError(0, 5, 30);

      rightmotor.configClosedLoopPeriod(0, 1,
        30);
      leftmotor.configClosedLoopPeriod(0, 1,
        30);

    /* Motion Magic Configurations */

    /**
     * Need to replace numbers with real measured values for acceleration and cruise
     * vel.
     */
    leftmotor.configMotionAcceleration(6750,30);
    leftmotor.configMotionCruiseVelocity(6750,30);
    leftmotor.configMotionSCurveStrength(3);

    rightmotor.configMotionAcceleration(6750,30);
    rightmotor.configMotionCruiseVelocity(6750,30);
    rightmotor.configMotionSCurveStrength(3);

    System.out.println("configure simple magic");

  } // End configureDriveTrainControllersForSimpleMagic

    
    //endingPosition is inches
    public void drivePIDLinear(int endingPosition) {
      leftmotor.set(TalonFXControlMode.MotionMagic,endingPosition*tickPerInch);
      rightmotor.set(TalonFXControlMode.MotionMagic,endingPosition*tickPerInch);
      System.out.println(endingPosition*tickPerInch);
    }

    public void driveForward(double power) {
      leftmotor.set(TalonFXControlMode.PercentOutput, power);
      rightmotor.set(TalonFXControlMode.PercentOutput, power);
    }

    public void stopRobot() {
      leftmotor.set(TalonFXControlMode.PercentOutput, 0);
      rightmotor.set(TalonFXControlMode.PercentOutput, 0);
    }

  public void setTarget() {
    //rightmotor.set(TalonFXControlMode.Position, 0, DemandType.AuxPID, 45);

    rightmotor.selectProfileSlot(0, 0);
    rightmotor.set(TalonFXControlMode.MotionMagic, 45);
    System.out.println("target set or smth");
  }

  /**
   * Reversing configuration set for pitch balance, so can drive normally again
   */
  public void ConfigureMotorsForDriving() {

    // May be not needed due to the negiative coefficient
    //leftmotor.setInverted(false);
    //rightmotor.setInverted(true);

    // Reversing settings set on the right motor for balance

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    talonConfig.slot0.kP = 0.75;
    talonConfig.slot0.kI = 0.005;
    talonConfig.slot0.kD = 0.01;
    talonConfig.slot0.kF = 0.0;

    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    talonConfig.primaryPID.selectedFeedbackCoefficient = 1;

    rightmotor.configAllSettings(talonConfig,30);

    rightmotor.configClosedLoopPeakOutput(0, 0.5);

    System.out.println("Configured motors for Driving");

  }

  public void ConfigureMotorsForBalancePitch(){

    /**
     * Need to reverse the inversion on the motors, since the pitch will DECREASE when driving forward
     * So the PID will need to drive "backwards" to get to the lower target angle value
     */
    // May be not needed due to the negative PID coefficient
    //leftmotor.setInverted(true);
    //rightmotor.setInverted(false);

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.slot0.kP = 0.4;
    talonConfig.slot0.kD = 0.0;
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
    //talonConfig.primaryPID.selectedFeedbackCoefficient = -360.0/8192; // Since the coefficient is negative, we do not need to invert the motors
    //talonConfig.primaryPID.selectedFeedbackCoefficient = -1;


    //rightmotor.configFactoryDefault();
    rightmotor.configAllSettings(talonConfig,30);

    rightmotor.configClosedLoopPeakOutput(0, 0.5);

    leftmotor.follow(rightmotor); // This MUST be unset at the end of the command!!! We want the robot to go straight when balancing

    System.out.println("Configured motors for Balance");

  }



  public void balanceRobotToPitch(double pitch) {
    ConfigureMotorsForBalancePitch();
    rightmotor.set(ControlMode.Position, 0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
