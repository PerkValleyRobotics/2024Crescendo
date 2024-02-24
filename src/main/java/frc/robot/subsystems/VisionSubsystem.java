// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {

  private static VisionSubsystem instance;
  /** Creates a new Vision. */
  private double x = 0.0;
  private double y = 0.0;
  private double area = 0.0;
  private double v;
  private double[] campose;
  private double[] def = {0,0,0,0,0,0};

  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tv;
  private NetworkTableEntry camposeentry;

  public VisionSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");

    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    camposeentry = table.getEntry("camerapose_targetspace");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    v = tv.getDouble(0.0);
    campose = camposeentry.getDoubleArray(def);
    //System.out.println(campose[4]);
  }


  public double getX(){
    return x;
  }

  public double gety(){
    return y;
  }

  public double getArea(){
    return area;
  }

  public boolean getv() {
    return v == 1 ? true : false;
  }

  public double getRotation() {
    return campose[4];
  }

  public double getTZ() {
    return campose[2];
  }

  public static VisionSubsystem getInstance(){
    if (instance == null){
      instance = new VisionSubsystem();
      return instance;
    } else {
      return instance;
    }
  }
}
