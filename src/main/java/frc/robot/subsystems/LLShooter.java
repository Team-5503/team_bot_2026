// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LLShooter extends SubsystemBase {
  /** Creates a new LLShooter. */

    private final int BOTH_ALLIANCE_PIPELINE = 0; //Default Fallback
    private final int RED_ALLIANCE_PIPELINE = 1;
    private final int BLUE_ALLIANCE_PIPELINE = 2;

  public LLShooter() {
    LimelightHelpers.setPipelineIndex("", BOTH_ALLIANCE_PIPELINE);
  }

  public static double getAimAssist() {
    double drivespeed = LimelightHelpers.getTX("")*0.005;
        return -drivespeed;
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

  LimelightHelpers.setPipelineIndex("", BOTH_ALLIANCE_PIPELINE);

  Optional<Alliance> allianceColor = DriverStation.getAlliance();

  if (allianceColor.isPresent()){
    if (allianceColor.get() == Alliance.Red){
    LimelightHelpers.setPipelineIndex("", RED_ALLIANCE_PIPELINE);
    }
    else if (allianceColor.get() == Alliance.Blue){
    LimelightHelpers.setPipelineIndex("", BLUE_ALLIANCE_PIPELINE);
    }
  }
  }
}
