package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase{

    private final SwerveDrive swerveDrive;

    private static SwerveSubsystem instance;

    public double maxSpeed = Units.feetToMeters(15);


    public SwerveSubsystem(File directory) {

        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(150.0/7.0, 1);

        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(3.75), 6.75, 42);
        System.out.println("\"conversionFactor\": {");
        System.out.println("\t\"angle\": " + angleConversionFactor + ",");
        System.out.println("\t\"drive\": " + driveConversionFactor);
        System.out.println("}");
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try
        {
        swerveDrive = new SwerveParser(directory).createSwerveDrive(maxSpeed);
        } catch (Exception e)
        {
        throw new RuntimeException(e);
        }

        
    }

    public static SwerveSubsystem getInstance(){
        if (instance == null){
            instance = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "Neo"));
            return instance;
        } else {
            return instance;
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isopenloop){
      swerveDrive.drive(translation,
                        rotation,
                        fieldRelative,
                        isopenloop); // Open loop is disabled since it shouldn't be used most of the time.
    }

    public void putpossmartdashboard(){
       SmartDashboard.putNumber("Module 0", swerveDrive.getModulePositions()[0].distanceMeters);
    }

    public SwerveDriveKinematics getKinematics(){
        return swerveDrive.kinematics;
    }

    public void resetOdometry(Pose2d initialHolonomicPose){
      swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public Pose2d getPose(){
        return swerveDrive.getPose();
    }
        
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
      swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void postTrajectory(Trajectory trajectory){
      swerveDrive.postTrajectory(trajectory);
    }

    public void zeroGyro(){
        swerveDrive.zeroGyro();
        System.out.println("zeroing");
    }

    public void setMotorBrake(boolean brake){
      swerveDrive.setMotorIdleMode(brake);
    }

    public Rotation2d getHeading(){
      return swerveDrive.getYaw();
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY){
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians());
    }

    public ChassisSpeeds getFieldVelocity(){
        return swerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotVelocity(){
        return swerveDrive.getRobotVelocity();
    }

    public SwerveController getSwerveController(){
      return swerveDrive.swerveController;
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration(){
        return swerveDrive.swerveDriveConfiguration;
    }

    public void lock(){
      swerveDrive.lockPose();
    }

    public Rotation2d getPitch(){
      return swerveDrive.getPitch();
    }

    public void addFakeVisionReading(){
        swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
    }
}







        


    