// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.DriveCmds;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;
// import frc.robot.swervelib.SwerveController;

// public class TrackNoteCmd extends Command {
//   /** Creates a new TrackNoteCmd. */
//   SwerveSubsystem subsystem;
//   VisionSubsystem vision;
//   private ProfiledPIDController controllerx, controllery;
//   private PIDController controllerRotation;
//   public TrackNoteCmd(SwerveSubsystem subsystem, VisionSubsystem vSubsystem) {
//     this.subsystem = subsystem;
//     this.vision = vSubsystem;
//     addRequirements(subsystem,vSubsystem);
    
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}
  
//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//    if (Math.abs(vision.getArea()) != 0 && vision.getv()){
//       subsystem.drive(SwerveController.getTranslation2d(subsystem.getTargetSpeeds(controllery.calculate(vision.getArea()), controllerx.calculate(vision.getX()), 0, 0)), -controllerRotation.calculate(vision.getRotation()), false);
//      }
//      else {subsystem.drive(new Translation2d(0, 0), 0, false);
//     }
    

//      SmartDashboard.putBoolean("code run", true);
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
