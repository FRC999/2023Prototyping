// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

/*
 * This year we're not combining the code for multiple robots in the same project
 * So the constants will be declared as "final"
 */

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class RobotDriveChassisConstants { // configure the physical properties unique to the robot
    // here, such as dimensions, wheel diameter etc
    public static final double wheelDiameter = 5; // inches
    public static final double distanceBetweenWheels = 21.5; // inches
    public static final double chassisLength = 32.0; // inches, not including bumpers
    public static final double chassisWidth = 22.5; // inches, not including bumpers
    public static final int encoderUnitsPerMotorRotation = 2048;

    // TODO: check and change the constants below

    public static final double encoderGearReduction = 11.25;
    public static final int encoderUnitsPerRobotRotation = 66500;// thats the SUM of the two (this is just a rough
    // guess, and should be measured)
    static final double clicksPerFoot = 1.021*(120615/10);
    public static final int tickPerInch = (int)(clicksPerFoot / 12); // (int) (2048/(4*Math.PI));
    public static final int tolerance = 1*tickPerInch;
  }

  public static final class OIConstants {
    public static final int driverControllerPort = 1;
    public static final int turnControllerPort = 0;
  }

  public static final class DriveConstants {

    public static boolean isInvertdGearBox = false;

    public static int[] leftMotorPortID = new int[] { 1,2 };
    public static int[] rightMotorPortID = new int[] { 3,4 };

    public static int[] kLeftEncoderPorts = new int[] { 1 };
    public static int[] kRightEncoderPorts = new int[] { 3 };


    // Sensor phase - to ensure that sensor is positive when the output is positive
    public static boolean[] SensorPhase =  {false,false};
    // Invert motors
    public static boolean[] MotorInvert =  {true,false};

    public static boolean kLeftEncoderReversed = false;
    public static boolean kRightEncoderReversed = true;

    // TODO: Measure EncoderCPR - they're not attached to the wheelshaft, but rather to the Falcons
    public static final int kEncoderCPR = 1024;
    public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (RobotDriveChassisConstants.wheelDiameter * Math.PI) / (double) kEncoderCPR;

    // MotionMagic constants section

    // Closed loop constants

    //TODO: check and modify all MotionMagic constants

    /**
     * Talon FX supports multiple (cascaded) PID loops. For
     * now we just want the primary one.
     */
    public static final int kPIDLoopIdx = 0;

    // How long we wait for a configuration change to happen before we give up and
    // report a failure in milliseconds
    public final static int configureTimeoutMs = 30;

    public static int[] maximumLinearError;
    public static int[] maximumAngleError;
    
    // Full motor output value
    public final static int fullMotorOutput = 1023;
    // How many milliseconds between each closed loop call
    public final static int closedLoopPeriodMs = 1;
    // Motor neutral dead-band, set to the minimum 0.1%
    public final static double NeutralDeadband = 0.001;

    public final static int Izone_0 = 500;
    public static double PeakOutput_0 = 1;

    /**
     * Talon PID methods often demand slot ID's, so we wil keep this here I do not
     * think we actually need it with Falcons anymore
     */
    public final static int SLOT_0 = 0;

    // Gains for MotionMagic
    public final static double motionMagicPidP_Value = 0.75;// * fullMotorOutput / encoderUnitsPerShaftRotation;
    //public final static double motionMagicPidP_Value = 0.2;
    public final static double motionMagicPidI_Value = 0.005;// * fullMotorOutput / encoderUnitsPerShaftRotation;
    //public final static double motionMagicPidI_Value = 0.0;
    public final static double motionMagicPidD_Value = 0.01;
    //public final static double motionMagicPidD_Value = 0.0;
    public final static double motionMagicPidF_Value = 2;
    //public final static double motionMagicPidF_Value = 0.2;

    public final static int motionMagicCruiseVelocity = 2250 * 3;
    public final static int motionMagicAcceleration = 2250 * 3;
    public final static int motionMagicSmoothing = 3;

    // Deadband values
    public final static double deadbandX = 0.1;
    public final static double deadbandY = 0.1;
    public final static double deadbandZ = 0.1;

    // The difference between the left and right side encoder values when the robot
    // is rotated 180 degrees
    // Allowable error to exit movement methods
    public static int defaultAcceptableError = 250;

    // Make smoother turns - see Cheezy Driving
    public static double turnAdjust = 0.6;

    //autoRoutineConstants

    //encoder ticks per foot
    public static int[] ticksPerFoot = new int[] {};

    //rotational ticks per degree
    public static int[] ticksPerDegree = new int[] {};

    // **** Software trajectory values; get from characterization
    public static final double ksVolts = 0.15219;
    public static final double kvVoltSecondsPerMeter = 2.1748;
    public static final double kaVoltSecondsSquaredPerMeter = 0.49391;
    public static final DifferentialDriveKinematics kDriveKinematics = 
      new DifferentialDriveKinematics(
        Units.inchesToMeters(RobotDriveChassisConstants.distanceBetweenWheels)
      );
    public static final double trajectoryRioPidP_Value = 0.054151 ;
    public static final double trajectoryRioPidD_Value = 0;
    public static final double trajectoryRioPidI_Value = 0;
    
    // Default max values for trajectories - m/s and m/s^2
    public static final double maxVelocityDefault = 2;
    public static final double maxAccelerationDefault = 1;

  }

  public static final class PigeonIMUConstants {
    // CAN ID of Pigeon2
    public static int pigeonIMUId = 11;
}

}
