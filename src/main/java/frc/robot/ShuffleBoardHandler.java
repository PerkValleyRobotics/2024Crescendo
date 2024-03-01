package frc.robot;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShuffleBoardHandler {

  private ShuffleboardTab tab;

  private IntakeSubsystem intake;
  private LauncherSubsystem launcher;
  private SwerveSubsystem swerve;
  private VisionSubsystem vision;

  private GenericEntry launcherSetpoint, launcherEncoder, launcherVelocity, launcherAngleSpeaker, intakeState, odometryX, odometryY, odometryH, angleToSpeaker, odometryUpdated;
  private ShuffleboardLayout launcherGrid, odometryGrid;

  // private boolean isAddedLimelightLaunch;
  // private boolean isAddedCam0;

  public ShuffleBoardHandler() {
    tab = Shuffleboard.getTab("Happy");

    intake = IntakeSubsystem.getInstance();
    launcher = LauncherSubsystem.getInstance();
    swerve = SwerveSubsystem.getInstance();
    vision = VisionSubsystem.getInstance();

    CameraServer.startAutomaticCapture();
  }

  public void setUp() {
    launcherGrid = tab.getLayout("Launcher", BuiltInLayouts.kGrid).withProperties(Map.of("Number of Columns",1,"Number of Rows",3)).withPosition(0,0).withSize(1,3);
    odometryGrid = tab.getLayout("Odometry", BuiltInLayouts.kGrid).withProperties(Map.of("Number of Columns",1,"Number of Rows",3)).withPosition(1,0).withSize(1,3);

    launcherSetpoint = launcherGrid.add("launcherSetpoint", launcher.getSetpoint()).withPosition(0,0).getEntry();
    launcherEncoder = launcherGrid.add("launcherEncoder", launcher.getAngle()).getEntry();
    launcherVelocity = launcherGrid.add("launcherVelocity", launcher.getVelocity()).getEntry();
    launcherAngleSpeaker = launcherGrid.add("speakerAngle",-2.24601*swerve.triangulateDistanceToSpeaker(false)+9.8).getEntry();
    intakeState = tab.add("intakeState", intake.getToggle()).withPosition(0,3).withSize(4,1).getEntry();
    odometryX = odometryGrid.add("odometryX", swerve.getPose().getX()).getEntry();
    odometryY = odometryGrid.add("odometryY", swerve.getPose().getY()).getEntry();
    odometryH = odometryGrid.add("odometryH", swerve.getPose().getRotation().getDegrees()).getEntry();
    angleToSpeaker = odometryGrid.add("angleToSpeaker", -swerve.getAngleToSpeaker(false)).getEntry();
    odometryUpdated = tab.add("odometryUpdated?", vision.getArea()!=0).withPosition(4,3).withSize(4,1).getEntry();


    if (CameraServer.getServer("limelight-launch") != null) tab.add("limelightLauncher", CameraServer.getServer("limelight-launch")).withPosition(2,0).withSize(3,3).getEntry();
    if (CameraServer.getServer("USB Camera 0") != null) tab.add("noteCam", CameraServer.getServer("USB Camera 0")).withPosition(5,0).withSize(3,3).getEntry();
  }

  public void update() {
    launcherSetpoint.setDouble(launcher.getSetpoint());
    launcherEncoder.setDouble(launcher.getAngle());
    launcherVelocity.setDouble(launcher.getVelocity()[0]);
    launcherAngleSpeaker.setDouble(-2.24601*swerve.triangulateDistanceToSpeaker(false)+9.8);
    intakeState.setBoolean(intake.getToggle());
    odometryX.setDouble(swerve.getPose().getX());
    odometryY.setDouble(swerve.getPose().getY());
    odometryH.setDouble(swerve.getPose().getRotation().getDegrees());
    angleToSpeaker.setDouble(-swerve.getAngleToSpeaker(false));
    odometryUpdated.setBoolean(vision.getArea()!=0);

    // if (CameraServer.getServer("limelight-launch") != null) tab.add("limelightLauncher", CameraServer.getServer("limelight-launch")).withPosition(2,0).withSize(3,3).getEntry();
    // if (CameraServer.getServer("USB Camera 0") != null) tab.add("noteCam", CameraServer.getServer("USB Camera 0")).withPosition(5,0).withSize(3,3).getEntry();
  }
}
