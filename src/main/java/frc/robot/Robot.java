// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;

import com.reduxrobotics.canand.ReduxJNI.Helper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private ShuffleBoardHandler shuffle = new ShuffleBoardHandler();

  private static SendableChooser m_chooser;

  private Timer disabledTimer;

  private SwerveSubsystem driveBase = SwerveSubsystem.getInstance();

  private LimelightHelpers helper = new LimelightHelpers();

  private LimelightTarget_Fiducial fiducailHelper = new LimelightTarget_Fiducial();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    m_chooser = new SendableChooser<String>();

    SmartDashboard.putData("Autons", m_chooser);

    disabledTimer = new Timer();

    shuffle.setUp();

    if (LimelightHelpers.getTV("limelight-launch") && LimelightHelpers.getTA("limelight-launch") != 0){
      Pose2d pos = LimelightHelpers.getBotPose2d_wpiBlue("limelight-launch");
      //Pose2d pos2d = new Pose2d(pos.getX(), pos.getY(), new Rotation2d(pos.getRotation().getAngle()));
      double timestamp = Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline("limelight-launch")/1000.0) - (LimelightHelpers.getLatency_Capture("limelight-launch")/1000.0);
      driveBase.updateOdometry(pos, timestamp);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    if (LimelightHelpers.getTV("limelight-launch") && LimelightHelpers.getTA("limelight-launch") != 0){
      Pose2d pos = LimelightHelpers.getBotPose2d_wpiBlue("limelight-launch");
      Pose2d betterPos = new Pose2d(new Translation2d(pos.getX(),pos.getY()), driveBase.getHeading());
      //Pose2d pos2d = new Pose2d(pos.getX(), pos.getY(), new Rotation2d(pos.getRotation().getAngle()));
      double timestamp = Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline("limelight-launch")/1000.0) - (LimelightHelpers.getLatency_Capture("limelight-launch")/1000.0);
      driveBase.updateOdometry(betterPos, timestamp);
    }
    shuffle.update();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }


  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.Drivebase.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(m_chooser.getSelected());

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);
  }
  

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    try
    {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e)
    {
      throw new RuntimeException(e);
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
