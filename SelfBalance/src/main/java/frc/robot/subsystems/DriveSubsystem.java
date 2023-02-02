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
  
  //taken from constants

    
  //WPI_TalonFX motorController = new WPI_TalonFX(10);
/*
  motorController = new WPI_TalonSRX(DEVICE_ID_TURRET);

  motorController.configFactoryDefault();
  //var rightConfig = new TalonSRXConfiguration();
  rightConfig.openloopRamp = 0.2;
  
  // Potentiometer is primary PID to get soft limit support
  rightConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.Analog;
  rightConfig.forwardSoftLimitThreshold = -156;
  rightConfig.forwardSoftLimitEnable = true;
  rightConfig.reverseSoftLimitThreshold = -400;
  rightConfig.reverseSoftLimitEnable = true;
  rightConfig.slot0.kP = 64d;
  rightConfig.slot0.kD = 0d;
  
  // We don't use Talon's sensor coefficient feature to convert native units to degrees mainly because it lowers
  // precision since the value has to result in an integer. For example, if we use a coefficient of 0.0439 to convert
  // pigeon units to degrees we only get 360 units per revolution vs. the native 8192.

  motorController.configAllSettings(rightConfig);
  motorController.selectProfileSlot(0, 0);
  motorController.selectProfileSlot(1, 1);
  motorController.configVoltageCompSaturation(12);
  motorController.enableVoltageCompensation(true);
  motorController.overrideLimitSwitchesEnable(false);
  motorController.setNeutralMode(NeutralMode.Brake);
  motorController.setSensorPhase(false);
  motorController.setInverted(true);
  motorController.setSafetyEnabled(true);
  
*/


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

    
    //endingPosition is inches
    public void drivePIDLinear(int endingPosition) {
      leftmotor.set(TalonFXControlMode.MotionMagic,endingPosition*tickPerInch);
      rightmotor.set(TalonFXControlMode.MotionMagic,endingPosition*tickPerInch);
      System.out.println(endingPosition*tickPerInch);
    }



    public void stopRobot() {
      leftmotor.set(TalonFXControlMode.PercentOutput, 0);
      rightmotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void configurePigeon(){
      
      //configure imu as the remote sensor for the right talon
      rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.GadgeteerPigeon_Yaw;
      rightConfig.remoteFilter0.remoteSensorDeviceID = 4; // the id for the pigeon is 4 :)

      //feedback coefficient and setting the pigeon as the feedback device
      rightConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();
      rightConfig.auxiliaryPID.selectedFeedbackCoefficient = kTurnTravelUnitsPerRotation / kPigeonUnitsPerRotation;

      //setting status frame periods
      rightmotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 20, 30);
		  rightmotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, 30);
		  rightmotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 20, 30);
      //localbird = RobotContainer.pigeonIMUSubsystem.getBird();
		  //localbird.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR , 5, 30); //this b not working
      
      (RobotContainer.pigeonIMUSubsystem.getBird()).setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR , 5, 30); 

      rightConfig.neutralDeadband = kNeutralDeadband;
      leftConfig.neutralDeadband = kNeutralDeadband;
      
      //max and min power supplied
      leftConfig.peakOutputForward = +0.3;
		  leftConfig.peakOutputReverse = -0.3;
		  rightConfig.peakOutputForward = +0.3;
		  rightConfig.peakOutputReverse = -0.3;

      /* FPID Gains for turn servo */
		  rightConfig.slot1.kP = 2.0;
		  rightConfig.slot1.kI = 0.0;
		  rightConfig.slot1.kD = 4.0;
		  rightConfig.slot1.kF = 0.0;
		  rightConfig.slot1.integralZone = 200;
		  rightConfig.slot1.closedLoopPeakOutput = 1.00;
		  rightConfig.slot1.allowableClosedloopError = 0;

      //closed loop time 
      rightConfig.slot0.closedLoopPeriod = 1;
		  rightConfig.slot1.closedLoopPeriod = 1;

      //configure settings
		  rightmotor.configAllSettings(rightConfig);
		  leftmotor.configAllSettings(leftConfig);

      System.out.println("pigeon configured");


      /*leftConfig.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 30);
      rightConfig.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 30);
      */
    }

    public void drivePIDTurn(double targetAngle) {
      leftmotor.set(TalonFXControlMode.MotionMagic, targetAngle);
      //rightmotor.set(TalonFXControlMode.MotionMagic, targetAngle);   
      System.out.println("pigeon turned");
    }

    public void configurePigeonCondensed(){
      (RobotContainer.pigeonIMUSubsystem.getBird()).configFactoryDefault();
      
      /* Set Neutral Mode */
      leftmotor.setNeutralMode(NeutralMode.Brake);
      //_rightMaster.setNeutralMode(NeutralMode.Brake);

      //leftmotor.setInverted(_leftInvert);
      //_rightMaster.setInverted(_rightInvert);

      /*
      * Talon FX does not need sensor phase set for its integrated sensor
      * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
      * and the user calls getSelectedSensor* to get the sensor's position/velocity.
      * 
      * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
      */
          // _rightMaster.setSensorPhase(true);
          // _leftMaster.setSensorPhase(true);
      
      /** Feedback Sensor Configuration */
      
      /* Configure the Pigeon IMU as a Remote Sensor for the right Talon */
      leftConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.GadgeteerPigeon_Yaw;
      leftConfig.remoteFilter0.remoteSensorDeviceID = (RobotContainer.pigeonIMUSubsystem.getBird()).getDeviceID();
      
      /* Configure the Remote Sensor to be the Selected Sensor of the right Talon */
      leftConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();
      
      /* Scale the Selected Sensor using a coefficient (Values explained in Constants.java */
      leftConfig.auxiliaryPID.selectedFeedbackCoefficient = kTurnTravelUnitsPerRotation / kPigeonUnitsPerRotation;
      
      /* Set status frame periods */
      leftmotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 20, 30);
      leftmotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, 30);
      leftmotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 20, 30);
      (RobotContainer.pigeonIMUSubsystem.getBird()).setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR , 5, 30);
      
      /* Configure neutral deadband */
      leftConfig.neutralDeadband = 0.001;
      //_leftConfig.neutralDeadband = Constants.kNeutralDeadband;		

      /* max out the peak output (for all modes).  However you can
      * limit the output of a given PID object with configClosedLoopPeakOutput().
      */
      leftConfig.peakOutputForward = +1.0;
      leftConfig.peakOutputReverse = -1.0;
      rightConfig.peakOutputForward = +1.0;
      rightConfig.peakOutputReverse = -1.0;

      /* FPID Gains for turn servo */
      /*
      rightConfig.slot1.kP = Constants.kGains_Turning.kP;
      rightConfig.slot1.kI = Constants.kGains_Turning.kI;
      rightConfig.slot1.kD = Constants.kGains_Turning.kD;
      rightConfig.slot1.kF = Constants.kGains_Turning.kF;
      rightConfig.slot1.integralZone = Constants.kGains_Turning.kIzone;
      rightConfig.slot1.closedLoopPeakOutput = Constants.kGains_Turning.kPeakOutput;
      rightConfig.slot1.allowableClosedloopError = 0;
      */
      
      /* 1ms per loop.  PID loop can be slowed down if need be.
      * For example,
      * - if sensor updates are too slow
      * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
      * - sensor movement is very slow causing the derivative error to be near zero.
      */
          int closedLoopTimeMs = 1;
          leftConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
          leftConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
      
      leftmotor.configAllSettings(leftConfig);

    
      /* Initialize */
      /*
      firstCall = true;
      state = false;
      printCount = 0;
      zeroYaw();
      */
    }
  public void ConfigureTurning(){

    leftmotor.setSafetyEnabled(false);
    rightmotor.setSafetyEnabled(false);

    rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.GadgeteerPigeon_Yaw;
    rightConfig.remoteFilter0.remoteSensorDeviceID = (RobotContainer.pigeonIMUSubsystem.getBird()).getDeviceID();

    System.out.println((RobotContainer.pigeonIMUSubsystem.getBird()).getDeviceID());
    
    rightConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();

    rightConfig.primaryPID.selectedFeedbackCoefficient = 360.0/8192.0;
    
    rightmotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 20, 30);
		rightmotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, 30);
		rightmotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 20, 30);
		(RobotContainer.pigeonIMUSubsystem.getBird()).setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR , 5, 30);

    rightConfig.neutralDeadband = 0.001;
		leftConfig.neutralDeadband = 0.001;

    leftConfig.peakOutputForward = +0.3;
		leftConfig.peakOutputReverse = -0.3;
		rightConfig.peakOutputForward = +0.3;
		rightConfig.peakOutputReverse = -0.3;

    rightConfig.slot0.kP = 0.75;
		rightConfig.slot0.kI = 0;
		rightConfig.slot0.kD = 0;
		rightConfig.slot0.kF = 0.0;
		rightConfig.slot0.integralZone = 200;
		rightConfig.slot0.closedLoopPeakOutput = 0.3;
		rightConfig.slot0.allowableClosedloopError = 0;

    int closedLoopTimeMs = 1;
    rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
    rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;

    rightmotor.configAllSettings(rightConfig);
  
    
    System.out.println("turning configured or smth");

    //the following things are from the teleopPeriodic in the ctre example code
    //rightmotor.selectProfileSlot(1, 1);
  }

  public void setTarget() {
    //rightmotor.set(TalonFXControlMode.Position, 0, DemandType.AuxPID, 45);

    rightmotor.selectProfileSlot(0, 0);
    rightmotor.set(TalonFXControlMode.MotionMagic, 45);
    System.out.println("target set or smth");
  }

  /*
  public void positionToRobotAngle(double angle) {
    var position = degreesPositionToNativePot(angle);
      motorController.set(
        TalonSRXControlMode.Position, 
        MathUtil.clamp(position, -400 + 1, -156 - 1));
  }

  public static double degreesPositionToNativePot(double degrees) {
    return ( - ((360 - (-400 * (183 / (-400 + 156)))) - (180 - 183 / 2))) / (183 / (-400 + 156));
  }
  */

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

  public void ConfigureMotorsForBalancePitch(){
    leftmotor.setSafetyEnabled(false);
    rightmotor.setSafetyEnabled(false);
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.slot0.kP = 0.4;
    talonConfig.slot0.kD = 0.0;
    talonConfig.remoteFilter0.remoteSensorDeviceID = 4;
    talonConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.GadgeteerPigeon_Pitch;
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;

    rightmotor.configFactoryDefault();
    rightmotor.configAllSettings(talonConfig);
    rightmotor.setNeutralMode(NeutralMode.Brake);
    rightmotor.configClosedLoopPeakOutput(0, 0.3);
  }

  public void birdTurnRight(double position) {
    rightmotor.set(ControlMode.Position, position);
  }

  public void balanceRobotToPitch(double pitch) {
    ConfigureMotorsForBalancePitch();
    rightmotor.set(ControlMode.Position, pitch);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
