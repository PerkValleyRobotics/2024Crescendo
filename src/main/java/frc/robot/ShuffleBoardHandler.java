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

  private GenericEntry launcherSetpoint, launcherEncoder, launcherVelocity, intakeState, odometryX, odometryY, odometryH;
  private ShuffleboardLayout launcherGrid, odometryGrid;

  public ShuffleBoardHandler() {
    tab = Shuffleboard.getTab("Happy");

    intake = IntakeSubsystem.getInstance();
    launcher = LauncherSubsystem.getInstance();
    swerve = SwerveSubsystem.getInstance();
    vision = VisionSubsystem.getInstance();
  }

  public void setUp() {
   /*
    * Launcher set point
      Intake up or down
      Limelight video
      Launcher velocity
      Odometry
      ?Pathfind location
      Launcher encoder
    */
    launcherGrid = tab.getLayout("Launcher", BuiltInLayouts.kGrid).withProperties(Map.of("Number of Columns",1,"Number of Rows",3)).withPosition(0,0);
    odometryGrid = tab.getLayout("Odometry", BuiltInLayouts.kGrid).withProperties(Map.of("Number of Columns",1,"Number of Rows",3)).withPosition(1,0);

    launcherSetpoint = launcherGrid.add("launcherSetpoint", launcher.getSetpoint()).withPosition(0,0).getEntry();
    launcherEncoder = launcherGrid.add("launcherEncoder", launcher.getAngle()).getEntry();
    launcherVelocity = launcherGrid.add("launcherVelocity", launcher.getVelocity()[0]).getEntry();
    intakeState = tab.add("intakeState", intake.getToggle()).getEntry();
    odometryX = odometryGrid.add("odometryX", swerve.getPose().getX()).getEntry();
    odometryY = odometryGrid.add("odometryY", swerve.getPose().getY()).getEntry();
    odometryH = odometryGrid.add("odometryH", swerve.getPose().getRotation().getDegrees()).getEntry();

    if (CameraServer.getServer("limelight-launch") != null) tab.add("limelightLauncher", CameraServer.getServer("limelight-launch"));
  }

  public void update() {
    launcherSetpoint.setDouble(launcher.getSetpoint());
    launcherEncoder.setDouble(launcher.getAngle());
    launcherVelocity.setDouble(launcher.getVelocity()[0]);
    intakeState.setBoolean(intake.getToggle());
    odometryX.setDouble(swerve.getPose().getX());
    odometryY.setDouble(swerve.getPose().getY());
    odometryH.setDouble(swerve.getPose().getRotation().getDegrees());
  }
}
