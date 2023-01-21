// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  WPI_TalonFX leftmotor = new WPI_TalonFX(9);
  WPI_TalonFX rightmotor = new WPI_TalonFX(10);
  final double clicksPerFoot = 120615/10;
  public final int tickPerInch = (int)(clicksPerFoot / 12); // (int) (2048/(4*Math.PI));
  public final int tolerance = 1*tickPerInch;

  public DifferentialDrive drive;
  
  TalonFXConfiguration leftConfig;
  TalonFXConfiguration rightConfig;
  public final static int kPigeonUnitsPerRotation = 8192;
  public final static double kTurnTravelUnitsPerRotation = 3600;
  public final static double kNeutralDeadband = 0.001;
  public final double turnTolerance = 1; 


  //private PigeonIMU localbird ;

  //testPosition constants
  
  // ***** TRAJECORY DRIVING VARIABLES ******
  private double ticksPerRotation = 2048; // Falcons
  private double wheelDiameter = 4; // inches
  private double gearBoxRation = 6.1; // gearbox - from encoders to the wheels
  private DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    configureEncoders();
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

  public void zeroEncoders(){
    leftmotor.setSelectedSensorPosition(0);
    rightmotor.setSelectedSensorPosition(0);
    System.out.println("encoders zeroed");
  }

  public void configureEncoders(){
    leftmotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    rightmotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
  }

  public void manualDrive(double turn, double move) {
    drive.arcadeDrive(turn,move);
  }

  
  //12/18/22 testing Subsystem

  public void ConfigureMotorTurning(){
    leftmotor.setSafetyEnabled(false);
    rightmotor.setSafetyEnabled(false);
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.slot0.kP = 0.4;
    talonConfig.slot0.kD = 0.0;
    talonConfig.remoteFilter0.remoteSensorDeviceID = 4;
    talonConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.GadgeteerPigeon_Yaw;
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;

    rightmotor.configFactoryDefault();
    rightmotor.configAllSettings(talonConfig);
    rightmotor.setNeutralMode(NeutralMode.Brake);
    rightmotor.configClosedLoopPeakOutput(0, 0.3);
  }

  public void birdTurnRight(double position) {
    rightmotor.set(ControlMode.Position, position);
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
    
    rightmotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10,
        30);
    leftmotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10,
        30);
    rightmotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10,
        30);
    leftmotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10,
        30);
    //rightmotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20,
    //    30);
    //leftmotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20,
    //    30);

    /**
    * Max out the peak output (for all modes). However you can limit the output of
    * a given PID object with configClosedLoopPeakOutput().
    */
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
    leftmotor.configMotionAcceleration(6750,
      30);
    leftmotor.configMotionCruiseVelocity(6750,
      30);
   leftmotor.configMotionSCurveStrength(3);

    rightmotor.configMotionAcceleration(6750,
      30);
    rightmotor.configMotionCruiseVelocity(6750,
      30);
    rightmotor.configMotionSCurveStrength(3);

    System.out.println("configure simple magic");

  } // End configureDriveTrainControllersForSimpleMagic

  public void drivePIDLinear(int endingPosition) {
    leftmotor.set(TalonFXControlMode.MotionMagic,endingPosition*tickPerInch);
    rightmotor.set(TalonFXControlMode.MotionMagic,endingPosition*tickPerInch);
    System.out.println(endingPosition*tickPerInch);
  }

  public void stopRobot() {
    leftmotor.set(TalonFXControlMode.PercentOutput, 0);
    rightmotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  //  **** TRAJECTORY DRIVING METHODS *****

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() { // needs to be meters per second
    return new DifferentialDriveWheelSpeeds(TranslateVelocityIntoMetersPerSecond(leftmotor.getSelectedSensorVelocity()),
                                            TranslateVelocityIntoMetersPerSecond(rightmotor.getSelectedSensorVelocity())
    );
  }

  public double TranslateVelocityIntoMetersPerSecond(double velocityRawUnits) {
    // Raw units - ticks per 100ms
    return Units.inchesToMeters((velocityRawUnits * 10) / tickPerInch) ;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    zeroEncoders();
    m_odometry.resetPosition(  // distances need to be in meters
        m_gyro.getRotation2d(), TranslateDistanceIntoMeters(leftmotor.getSelectedSensorPosition()),
        TranslateDistanceIntoMeters(rightmotor.getSelectedSensorPosition()), pose);
  }

  public double TranslateDistanceIntoMeters(double distanceRawUnits) {
    return Units.inchesToMeters(distanceRawUnits / tickPerInch) ;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
