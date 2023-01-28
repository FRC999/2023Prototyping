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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {

  WPI_TalonFX leftmotor = new WPI_TalonFX(9);
  WPI_TalonFX rightmotor = new WPI_TalonFX(10);
  //final double clicksPerFoot = 120615/10;
  final double clicksPerFoot = 1.021*(120615/10);
  public final int tickPerInch = (int)(clicksPerFoot / 12); // (int) (2048/(4*Math.PI));
  public final int tolerance = 1*tickPerInch;

  public DifferentialDrive drive;
  
  public final static int kPigeonUnitsPerRotation = 8192;
  public final static double kTurnTravelUnitsPerRotation = 3600;
  public final static double kNeutralDeadband = 0.001;
  public final double turnTolerance = 1; 

  private final Field2d m_field = new Field2d();
  
  //private PigeonIMU localbird ;

  //testPosition constants
  
  // ***** TRAJECORY DRIVING VARIABLES ******
  private double ticksPerRotation = 2048; // Falcons
  private double wheelDiameter = 4; // inches
  private double gearBoxRation = 6.1; // gearbox - from encoders to the wheels
  private DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    resetToFactoryDefaults();
    configureEncoders();
    configureSimpleMagic();
    brakeMode();
    zeroEncoders();
    drive = new DifferentialDrive(leftmotor, rightmotor);
    drive.setSafetyEnabled(false);

    // Set initial odometry for trajectories - where robot is right now from the encoder and IMU point of view
    m_odometry =
        new DifferentialDriveOdometry(
          RobotContainer.imuSubsystem.getRotation2d(),
          TranslateDistanceIntoMeters(leftmotor.getSelectedSensorPosition()),
          TranslateDistanceIntoMeters(-rightmotor.getSelectedSensorPosition())
        );

    SmartDashboard.putData("Field", m_field);
  }

  public void brakeMode() {
    leftmotor.setNeutralMode(NeutralMode.Brake);
    rightmotor.setNeutralMode(NeutralMode.Brake);
  }

  public void arcadeDriving(double move, double turn){
    drive.arcadeDrive(move, turn);
  }
  
  // Reset motors to factory defaults
  private void resetToFactoryDefaults() {
    leftmotor.configFactoryDefault();
    rightmotor.configFactoryDefault();
  }

   /** Get the number of tics moved by the left encoder */
   public int getLeftEncoder() {
    return (int) leftmotor.getSelectedSensorPosition();
  }

  /** Get the number of tics moved by the left encoder */
  public int getRightEncoderVelocity() {
    return (int) -rightmotor.getSelectedSensorVelocity();
  }

  public int getLeftEncoderVelocity() {
    return (int) leftmotor.getSelectedSensorVelocity();
  }

  /** Get the number of tics moved by the left encoder */
  public int getRightEncoder() {
    return (int) -rightmotor.getSelectedSensorPosition();
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
  
  
  public void setBotPose(double xCoord, double yCoord, double rotation){
    this.resetOdometry(new Pose2d(xCoord, yCoord, new Rotation2d(rotation)));
  }



  
  /*
    All Hardware PID setup is removed
    Only inversion and encoder iinversion settings are left here
   */

  public void configureSimpleMagic() {
    leftmotor.setSafetyEnabled(false);
    rightmotor.setSafetyEnabled(false);

    /* Configure motor neutral deadband */
    rightmotor.configNeutralDeadband(0.001, 30);
    leftmotor.configNeutralDeadband(0.001, 30);

    leftmotor.setSensorPhase(false);
    rightmotor.setSensorPhase(false);

    leftmotor.setInverted(false);
    rightmotor.setInverted(false);



    System.out.println("configure simple magic - just inversion and deadband");

  } // End configureDriveTrainControllersForSimpleMagic

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
    return new DifferentialDriveWheelSpeeds(
        TranslateVelocityIntoMetersPerSecond(leftmotor.getSelectedSensorVelocity()),
        TranslateVelocityIntoMetersPerSecond(-rightmotor.getSelectedSensorVelocity())
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

    zeroEncoders();   // Reset Encoders
    RobotContainer.imuSubsystem.zeroHeading();  // Reset Yaw

    m_odometry.resetPosition(  // distances need to be in meters
        RobotContainer.imuSubsystem.getRotation2d(),
        TranslateDistanceIntoMeters(leftmotor.getSelectedSensorPosition()),
        TranslateDistanceIntoMeters(-rightmotor.getSelectedSensorPosition()),
        pose);
  }

  public double TranslateDistanceIntoMeters(double distanceRawUnits) {
    return Units.inchesToMeters(distanceRawUnits / tickPerInch) ;
  }

    /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }


  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {

    System.out.println("TV L:"+leftVolts+" R:"+rightVolts);

    leftmotor.setVoltage(leftVolts);
    rightmotor.setVoltage(-rightVolts);
    drive.feed();
  }

    // Should be used in periodic when the trajectory navigation is running
    public void updateOdometry() {
      m_odometry.update(
        RobotContainer.imuSubsystem.getRotation2d(),
        TranslateDistanceIntoMeters(getLeftEncoder()),
        TranslateDistanceIntoMeters(-getRightEncoder())
      );
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_field.setRobotPose(m_odometry.getPoseMeters());

  }
}
