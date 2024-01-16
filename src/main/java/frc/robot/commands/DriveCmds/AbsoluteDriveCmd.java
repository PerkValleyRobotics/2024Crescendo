package frc.robot.commands.DriveCmds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class AbsoluteDriveCmd extends Command {

  private final SwerveSubsystem swerve;
  private final DoubleSupplier vX, vY;
  private final DoubleSupplier headingHorizontal, headingVertical;
  private final BooleanSupplier turnOffAutoTurn;
  private boolean autoturn;
  private final boolean isOpenLoop;

  public AbsoluteDriveCmd(
    SwerveSubsystem swerve,
    DoubleSupplier vX,
    DoubleSupplier vY,
    DoubleSupplier headingHorizontal,
    DoubleSupplier headingVertical,
    BooleanSupplier turnOffAutoTurn,
    boolean isOpenLoop
  ) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingHorizontal = headingHorizontal;
    this.headingVertical = headingVertical;
    this.isOpenLoop = isOpenLoop;
    this.turnOffAutoTurn = turnOffAutoTurn;
    this.autoturn = false;

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    if (turnOffAutoTurn.getAsBoolean()){
        autoturn = true;
    }

    if (!DriverStation.isAutonomous()) {
    // Get the desired chassis speeds based on a 2 joystick module.

    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(
      vX.getAsDouble()/1,
      vY.getAsDouble()/1,
      headingHorizontal.getAsDouble(),
      headingVertical.getAsDouble()
    );

    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation =
      SwerveMath.limitVelocity(
        translation,
        swerve.getFieldVelocity(),
        swerve.getPose(),
        Constants.LOOP_TIME,
        Constants.ROBOT_MASS,
        List.of(Constants.CHASSIS),
        swerve.getSwerveDriveConfiguration()
      );

    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    if (Math.hypot(headingHorizontal.getAsDouble(),headingVertical.getAsDouble()) < 0.5 & !autoturn) {
      swerve.drive(translation, 0, true, isOpenLoop);
    } else {
      swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond,true, isOpenLoop);
    }
    // Make the robot move

    }
    SmartDashboard.putNumber("x pos", swerve.getPose().getTranslation().getX());
    SmartDashboard.putNumber("y pos", swerve.getPose().getTranslation().getY());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}