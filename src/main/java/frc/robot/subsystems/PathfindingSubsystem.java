package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PathfindingSubsystem extends SubsystemBase {
    private double frameTopSideLength;
    private double frameBottomSideLength;
    private double cornerToCenterLength;

    public PathfindingSubsystem(double frameTopSideLength, double frameBottomSideLength) {
        this.frameTopSideLength = frameTopSideLength;
        this.frameBottomSideLength = frameBottomSideLength;
    }

    public PathfindingSubsystem(double frameTopSideLength, double frameBottomSideLength, double cornerToCenterLength) {
        this.frameTopSideLength = frameTopSideLength;
        this.frameBottomSideLength = frameBottomSideLength;
        this.cornerToCenterLength = cornerToCenterLength;
    }

    public double[][] pathFindToPose(double[] startPos, double[] endPos) {
        return new double[][] {{0,0,0},{1,0,90}};
    }
}
