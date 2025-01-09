// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.swervelib.math.Matter;
import edu.wpi.first.math.util.Units;
import frc.robot.swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  //Robot size and basic parameters
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  //Can IDs
  public static final int KIntakeMotorID = 8;
  public static final int KLauncherBeltMotorID = 6;
  public static final int KLauncherPivotMotorID = 7;
  public static final int KLeftLauncherMotorID = 9;
  public static final int KRightLauncherMotorID = 10;

  // launcher tint PID
  public static final double KlauncherTiltP = 0.09;
  public static final double KlauncherTiltI = 0.0001;
  public static final double KlauncherTiltD = 0.012;
  public static final double KlauncherTiltIZ = 0;
  public static final double KlauncherTiltFF = 0;
  public static final double KlauncherTiltOutputMin = -1;
  public static final double KlauncherTiltOUtputMax = 1;

  public static final double KRlauncherP = 0;
  public static final double KRlauncherI = 0;
  public static final double KRlauncherD = 0.00;
  public static final double KRlauncherIZ = 0;
  public static final double KRlauncherFF = 0.9;
  public static final double KRlauncherOutputMin = -1;
  public static final double KRlauncherOUtputMax = 1;

  public static final double KLlauncherP = 0;
  public static final double KLlauncherI = 0;
  public static final double KLlauncherD = 0.00;
  public static final double KLlauncherIZ = 0;
  public static final double KLlauncherFF = 0.9;
  public static final double KLlauncherOutputMin = -1;
  public static final double KLlauncherOUtputMax = 1;

  public static final class Auton{

    public static final PIDFConfig xAutoPID     = new PIDFConfig(3, 0, 0);
    public static final PIDFConfig yAutoPID     = new PIDFConfig(3, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.6, 0, .1);

    public static final double MAX_SPEED        = 4;
    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase{
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }
  public static class OperatorConstants {
    //Deadband for joysticks on controllers
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;

    //Controller port numbers
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }
}
