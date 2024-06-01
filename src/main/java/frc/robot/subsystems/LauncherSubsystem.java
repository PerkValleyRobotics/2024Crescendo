// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

  private static LauncherSubsystem instance;

  private CANSparkMax left;
  private CANSparkMax right;
  private CANSparkMax rotation;

  private SparkPIDController leftPIDController;
  private SparkPIDController rightPIDController;
  private SparkPIDController rotationPIDController;

  private PIDController pivotAbs;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private RelativeEncoder rotationEncoder;
  // private DutyCycleEncoder revE = new DutyCycleEncoder(0);

  private double leftSet;
  private double rightSet;
  private double setpoint;

  public LauncherSubsystem() {
    left = new CANSparkMax(Constants.KLeftLauncherMotorID, MotorType.kBrushless);
    right = new CANSparkMax(Constants.KRightLauncherMotorID, MotorType.kBrushless);
    rotation = new CANSparkMax(Constants.KLauncherPivotMotorID,MotorType.kBrushless);
    // rotation = new CANSparkMax(revE,MotorType.kBrushless);
    // left.set
    // pivotAbs = new PIDController(0.075, 0, 0);


    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();
    rotation.restoreFactoryDefaults();
    
    rotationPIDController = rotation.getPIDController();
  
    leftPIDController = left.getPIDController();
    rightPIDController = right.getPIDController();

    leftEncoder = left.getEncoder();
    rightEncoder = right.getEncoder();
    rotationEncoder = rotation.getEncoder();

    leftPIDController.setP(Constants.KLlauncherP);
    leftPIDController.setI(Constants.KLlauncherI);
    leftPIDController.setD(Constants.KLlauncherD);
    leftPIDController.setIZone(Constants.KLlauncherIZ);
    leftPIDController.setFF(Constants.KLlauncherFF);
    leftPIDController.setOutputRange(-1, 1);
    
    rightPIDController.setP(Constants.KRlauncherP);
    rightPIDController.setI(Constants.KRlauncherI);
    rightPIDController.setD(Constants.KRlauncherD);
    rightPIDController.setIZone(Constants.KRlauncherIZ);
    rightPIDController.setFF(Constants.KRlauncherFF);
    rightPIDController.setOutputRange(-1, 1);

    rotationPIDController.setP(Constants.KlauncherTiltP);
    rotationPIDController.setI(Constants.KlauncherTiltI);
    rotationPIDController.setD(Constants.KlauncherTiltD);
    rotationPIDController.setIZone(Constants.KlauncherTiltIZ);
    rotationPIDController.setFF(Constants.KlauncherTiltFF);
    rotationPIDController.setOutputRange(Constants.KlauncherTiltOutputMin, Constants.KlauncherTiltOUtputMax);

    leftSet = 0;
    rightSet = 0;

    rotationEncoder.setPosition(-2.3);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Left Launcher Velo: ", leftEncoder.getVelocity());
    // SmartDashboard.putNumber("Right Launcher Velo: ", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Left SetRPM: ", leftSet);
    SmartDashboard.putNumber("Right SetRPM: ", rightSet);
  }

  // public void setPivotAbsRef(double pos) {
  //   pivotAbs.setSetpoint(pos);
  // }
  // public void calcSetAbs() {
  //   rotation.set(pivotAbs.calculate(revE.getAbsolutePosition()));
  // }

  public void setLeftReference(double rpm){
    leftPIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity); 
    leftSet = rpm;
  }

  public void setRightReference(double rpm){
    rightPIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity); 
    rightSet = rpm;
  }

  public void setReference(double rpm){
    setLeftReference(rpm);
    setRightReference(-rpm);
  }

  public void setAngle(double setpoint) {
    this.setpoint = setpoint;
    rotationPIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition); 
  }

  public double getSetpoint() {
    return setpoint;
  }

  public double getAngle() {
    return rotationEncoder.getPosition();
  }

  public void resetEncoder() {
    rotationEncoder.setPosition(0);
  }

  public double[] getVelocity() {
    return new double[] {leftEncoder.getVelocity(), rightEncoder.getVelocity()};
  }

  public void setSpeed(double speed) {
    setLeft(speed);
    setRight(speed);
  }

  public void setLeft(double speed){
    left.set(speed);
  }

  public void setRight(double speed){
    right.set(-speed);
  }

  

  public static LauncherSubsystem getInstance(){
    if (instance == null){
      instance = new LauncherSubsystem();
      return instance;
    } else {
      return instance;
    }
  }
}
