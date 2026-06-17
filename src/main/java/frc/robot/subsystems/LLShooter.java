// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LLShooter extends SubsystemBase {
  /** Creates a new LLShooter. */
  public LLShooter() {

  }

  public double getAimAssist() {
    double drivespeed = LimelightHelpers.getTX("")*0.03;
        return drivespeed;
  }

  public boolean getTargetExistance() {
    boolean hasTarget = LimelightHelpers.getTV("");
    return hasTarget;
  }

  public double getTargetHorizontalOffset() {
    double tx = LimelightHelpers.getTX("");
    return tx;
  }

  public double getTargetVerticalOffset() {
    double ty = LimelightHelpers.getTY("");
    return ty;
  }

  public double getTargetArea() {
    double ta = LimelightHelpers.getTA("");
    return ta;
  }

  public int getTargetCount() {
    int targetcount = LimelightHelpers.getTargetCount("");
    return targetcount;
  }

  public double getBotCentricTagDistance() {
    double distanceToTargets = LimelightHelpers.avgTagDist("");
    return distanceToTargets;
  }

  public double getCameraCentricTagDistance(){
    double distanceToCamera = LimelightHelpers.distToCamera("");
    return distanceToCamera;
  }

  public int getAprilTagID() {
    int aprilTagID = LimelightHelpers.getid("");
    return aprilTagID;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
