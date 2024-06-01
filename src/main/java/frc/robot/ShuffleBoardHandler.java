package frc.robot;

import java.sql.Driver;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.reduxrobotics.sensors.canandcolor.CanandcolorProximityConfig.SamplingPeriod;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSubsystem;
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
  private AmpSubsystem ampy;

  private GenericEntry launcherSetpoint, launcherEncoder, launcherVelocity, launcherAngleSpeaker, intakeState, odometryX, odometryY, odometryH, angleToSpeaker, odometryUpdated, driverColor, distanceToSpeaker, autoChooser;
  private ShuffleboardLayout launcherGrid, odometryGrid;

  private Optional<Alliance> ally = DriverStation.getAlliance();

  // private SendableChooser<Command> autChooser;

  // private boolean isAddedLimelightLaunch;
  // private boolean isAddedCam0;

  // private Encoder revE = new Encoder(0);

  public ShuffleBoardHandler() {
    tab = Shuffleboard.getTab("Happy");

    intake = IntakeSubsystem.getInstance();
    launcher = LauncherSubsystem.getInstance();
    swerve = SwerveSubsystem.getInstance();
    vision = VisionSubsystem.getInstance();
    ampy = AmpSubsystem.getInstance();

    CameraServer.startAutomaticCapture();

    
  }

  public void setUp() {
    launcherGrid = tab.getLayout("Launcher", BuiltInLayouts.kGrid).withProperties(Map.of("Number of Columns",1,"Number of Rows",3)).withPosition(0,0).withSize(1,3);
    odometryGrid = tab.getLayout("Odometry", BuiltInLayouts.kGrid).withProperties(Map.of("Number of Columns",1,"Number of Rows",3)).withPosition(1,0).withSize(1,3);

    // autChooser = AutoBuilder.buildAutoChooser();
    // tab.add("Auto Chooser", autChooser).withSize(1,1).withPosition(4,3);

    launcherSetpoint = launcherGrid.add("launcherSetpoint", launcher.getSetpoint()).withPosition(0,0).getEntry();
    launcherEncoder = launcherGrid.add("launcherEncoder", launcher.getAngle()).getEntry();
    launcherVelocity = launcherGrid.add("launcherVelocity", launcher.getVelocity()).getEntry();
    launcherAngleSpeaker = launcherGrid.add("speakerAngle",-3.06*swerve.triangulateDistanceToSpeaker()+11.9).getEntry();
    intakeState = tab.add("intakeState", intake.getToggle()).withPosition(0,3).withSize(3,1).getEntry();
    driverColor = tab.add("driverColor", ally.isPresent()?(ally.get()==Alliance.Red?"Red":"Blue"):"N/A").withSize(2,1).withPosition(3,3).getEntry();
    odometryX = odometryGrid.add("odometryX", swerve.getPose().getX()).getEntry();
    odometryY = odometryGrid.add("odometryY", swerve.getPose().getY()).getEntry();
    odometryH = odometryGrid.add("odometryH", swerve.getPose().getRotation().getDegrees()).getEntry();
    angleToSpeaker = odometryGrid.add("angleToSpeaker", -swerve.getAngleToSpeaker()).getEntry();
    distanceToSpeaker = odometryGrid.add("distanceToSpeaker", swerve.triangulateDistanceToSpeaker()).getEntry();
    odometryUpdated = tab.add("odometryUpdated?", vision.getArea()!=0).withPosition(5,3).withSize(3,1).getEntry();


    //if (CameraServer.getServer("limelight-launch").isValid()) tab.add("limelightLauncher", CameraServer.getServer("limelight-launch")).withPosition(2,0).withSize(3,3).getEntry();
    // if (CameraServer.getServer("USB Camera 0") != null) tab.add("noteCam", CameraServer.getServer("USB Camera 0")).withPosition(5,0).withSize(3,3).getEntry();
  }

  public void update() {
    // SmartDashboard.putNumber("revEAbsolute",revE.getAbsolutePosition());
    SmartDashboard.putNumber("ServoAngle", ampy.getAngle());

    ally = DriverStation.getAlliance();

    launcherSetpoint.setDouble(launcher.getSetpoint());
    launcherEncoder.setDouble(launcher.getAngle());
    launcherVelocity.setDouble(launcher.getVelocity()[0]);
    launcherAngleSpeaker.setDouble(-3.06*swerve.triangulateDistanceToSpeaker()+11.9);
    intakeState.setBoolean(intake.getToggle());
    driverColor.setString(ally.isPresent()?(ally.get()==Alliance.Red?"Red":"Blue"):"N/A");
    odometryX.setDouble(swerve.getPose().getX());
    odometryY.setDouble(swerve.getPose().getY());
    odometryH.setDouble(swerve.getPose().getRotation().getDegrees());
    angleToSpeaker.setDouble(-swerve.getAngleToSpeaker());
    distanceToSpeaker.setDouble(swerve.triangulateDistanceToSpeaker());
    odometryUpdated.setBoolean(vision.getArea()!=0);

    SmartDashboard.putBoolean("testColorThing", SmartDashboard.getString("driverColor", "Blue")=="Blue");

    //if (CameraServer.getServer("limelight-launch") != null) tab.add(CameraServer.getServer("limelight-launch")).getEntry();
    // if (CameraServer.getServer("USB Camera 0") != null) tab.add("noteCam", CameraServer.getServer("USB Camera 0")).withPosition(5,0).withSize(3,3).getEntry();
  }
}
