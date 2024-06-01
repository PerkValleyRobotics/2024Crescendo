package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpSubsystem extends SubsystemBase{

    private static AmpSubsystem instance;

    private boolean ampPos = false;

    private Servo ampServo;

    public AmpSubsystem(){
      ampPos = false;
      ampServo = new Servo(0);
    }

    public void setServoReference(double angle) {
        ampServo.set(angle);
    }

    public double getAngle() {
        return ampServo.getAngle();
    }

    public void toggleServo() {
        if (ampPos) setServoReference(0.666);
        else setServoReference(0);

        ampPos = !ampPos;
    }

    public static  AmpSubsystem getInstance() {
        if (instance == null){
          instance = new AmpSubsystem();
          return instance;
        } else {
          return instance;
        }
      }
}
