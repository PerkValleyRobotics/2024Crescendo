// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Pathfinding;

// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import frc.robot.subsystems.VisionSubsystem;
// import swervelib.SwerveController;
// import frc.robot.subsystems.SwerveSubsystem;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;

// public class StraightToPoseCmd extends CommandBase {
//   /** Creates a new TurnToTagCmd. */
//   private SwerveSubsystem swerve;

//   private ProfiledPIDController controllerx, controllery;
//   private PIDController controllerRotation;
//   private TrapezoidProfile trapezoidProfile;

//   public StraightToPoseCmd(SwerveSubsystem swerve, double xSetPoint, double ySetPoint, double heading) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.swerve = swerve;

//     controllerx = new ProfiledPIDController(0.03, 0.02, 0.0015, new TrapezoidProfile.Constraints(Units.feetToMeters(10), Units.feetToMeters(2))); 

//     //controllerx.setSetpoint(xSetPoint);
//     controllerx.setGoal(xSetPoint);
//     controllerx.setTolerance(.1);

//     controllery = new ProfiledPIDController(0.275, 0.05, 0, new TrapezoidProfile.Constraints(Units.feetToMeters(10), Units.feetToMeters(2))); 

//     controllery.setGoal(ySetPoint);
//     controllery.setTolerance(.1);

//     controllerRotation = new PIDController(0.02, 0.005, 0); 

//     controllerRotation.setSetpoint(heading);

//     addRequirements(swerve);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     swerve.drive(SwerveController.getTranslation2d(swerve.getTargetSpeeds(controllery.calculate(swerve.getPose().getX()), controllerx.calculate(swerve.getPose().getY()), 0, 0)), -controllerRotation.calculate(swerve.getPose().getRotation().getDegrees()), true);
     

//     SmartDashboard.putBoolean("code run", true);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished(){
//     return controllerx.atSetpoint() && controllery.atSetpoint();
//   }
// }
