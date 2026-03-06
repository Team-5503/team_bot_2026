// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IndexConstants;

// TODO: set up the configuration to adjust for the fact that the motor is a neo 550 (thanks Mr. Weltch)

public class Index extends SubsystemBase {

  SparkMax index;

  private SparkClosedLoopController closedLoopControllerI;
  private RelativeEncoder indexEncoder;
  private SparkMaxConfig indexConfig;


  /** Creates a new Index. */
  public Index() {
     index = new SparkMax(IndexConstants.kCanID, MotorType.kBrushless); // will grip to balls

    indexEncoder = index.getEncoder(); // encoder of main motor
    closedLoopControllerI = index.getClosedLoopController(); // closed loop controller of main motor

    configure();
  }


 private void configure() {
    indexConfig = new SparkMaxConfig();
    indexConfig
      .inverted(IndexConstants.kInverted)
      .smartCurrentLimit(IndexConstants.kStallLimit, IndexConstants.kFreeLimit)
      .idleMode(IndexConstants.kIdleMode); 
    indexConfig.closedLoop
      .feedbackSensor(IndexConstants.kSensor) 
      .p(IndexConstants.kP)
      .i(IndexConstants.kI)
      .d(IndexConstants.kD)
      .outputRange(IndexConstants.kMinOutputLimit,IndexConstants.kMaxOutputLimit);
    indexConfig.softLimit
      .forwardSoftLimitEnabled(false)
      .forwardSoftLimit(IndexConstants.kForwardSoftLimit) 
      .reverseSoftLimitEnabled(false)
      .reverseSoftLimit(IndexConstants.kReverseSoftLimit);
    indexConfig.encoder
      .positionConversionFactor(IndexConstants.kPositionCoversionFactor);
    
    
      index.configure(indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
  }

  public void spin(double rpm){
    closedLoopControllerI.setSetpoint(rpm, ControlType.kVelocity);
  }

  public void stop(){
    index.stopMotor();
  }

  private double getVelocity(){
    return indexEncoder.getVelocity();
  }

  private double getError() {
    return Math.abs(Math.abs(getVelocity()) - Math.abs(closedLoopControllerI.getSetpoint()));
  }

  private boolean isAtSetpoint(){
    return (getError() < IndexConstants.kTolerance);
  }



 /*
   * COMMANDS THAT DO NOT SET ANYthing
   * TODO: SEE IF WE NEED TO MOVE THIS TO ITS OWN COMMAND FILE
   */

  public Command waitUntilAtSetpoint() {
    return new WaitUntilCommand(() -> {
      // TEST FOR IF PIVOTERROR IS IN TOLERANCE OF TARGETPOSITION
      return isAtSetpoint();
    });
  }
  /*
   * COMMANDS TO SET POSITIONS ( because we can't call commands that call for the same subsystem,
   * but you can call two commands that are in the same subsystem)
   */

   public Command setRPM(double rpm){
    return runOnce(() -> {
      spin(rpm);
    });
   }
   public Command stopMotors(){
    return runOnce(()-> {
      stop();
    });
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
