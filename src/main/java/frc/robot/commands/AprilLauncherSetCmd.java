// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.VisionSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;
// import edu.wpi.first.math.controller.PIDController;

// public class AprilLauncherSetCmd extends Command {
//   /** Creates a new AprilLauncherSetCmd. */
//   private VisionSubsystem vision;
//   private SwerveSubsystem swerve;
//   private PIDController PIDController;


//   public AprilLauncherSetCmd(VisionSubsystem vision, SwerveSubsystem swerve) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.vision = vision;
//     this.swerve = swerve;
//     PIDController = new PIDController(0.09, 0.01, 0);

//     PIDController.setSetpoint(0);

//     addRequirements(vision,swerve);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//   if(Math.abs(vision.getX()) > 0.05 && vision.getv())
//   {
//     swerve.drive(new Translation2d(0,0), -PIDController.calculate(vision.getRotation()), true);
//   }
//   else swerve.drive(new Translation2d(0,0),0,true);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
