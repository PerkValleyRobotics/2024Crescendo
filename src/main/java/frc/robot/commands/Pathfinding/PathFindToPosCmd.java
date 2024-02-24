// package frc.robot.commands.Pathfinding;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.PathfindingSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;

// public class PathFindToPosCmd extends CommandBase {
//     private SwerveSubsystem swerve;
//     private PathfindingSubsystem pathing;

//     private double[] targetPos = new double[3];
//     private double[][] posList;
//     private StraightToPoseCmd[] pathLineCmdList;

//     public PathFindToPosCmd(SwerveSubsystem swerve, PathfindingSubsystem pathing, double[] targetPos) {
//         this.swerve = swerve;
//         this.pathing = pathing;
//         this.targetPos = targetPos;

//         addRequirements(swerve);
//         addRequirements(pathing);
//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {
//         posList = pathing.pathFindToPose(new double[] {swerve.getPose().getX(), swerve.getPose().getY(), swerve.getPose().getRotation().getDegrees()}, targetPos);

//         pathLineCmdList = new StraightToPoseCmd[posList.length];

//         for (int i = 0; i < posList.length; i++) {
//             pathLineCmdList[i] = new StraightToPoseCmd(swerve, posList[i][0], posList[i][1], posList[i][2]);
//         }
//         Command group = new SequentialCommandGroup(new StraightToPoseCmd(swerve, 1,0,0));
//         group.execute();
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
       
//     }

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished(){
//         return false;
//     }
    
// }
