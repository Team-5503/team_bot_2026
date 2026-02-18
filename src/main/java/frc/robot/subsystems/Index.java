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

import frc.robot.Constants.ShooterConstants;

public class Index extends SubsystemBase {

  SparkMax index;

  private SparkClosedLoopController closedLoopControllerI;
  private RelativeEncoder indexEncoder;
  private SparkMaxConfig indexConfig;


  /** Creates a new Index. */
  public Index() {
     index = new SparkMax(0, MotorType.kBrushless); // will grip to balls

    indexEncoder = index.getEncoder(); // encoder of main motor
    closedLoopControllerI = index.getClosedLoopController(); // closed loop controller of main motor

    configure();
  }


 private void configure() {
    indexConfig = new SparkMaxConfig();
    indexConfig
      .inverted(ShooterConstants.kLInverted)
      .smartCurrentLimit(ShooterConstants.kStallLimit, ShooterConstants.kFreeLimit)
      .idleMode(ShooterConstants.kIdleMode); 
    indexConfig.closedLoop
      .feedbackSensor(ShooterConstants.kSensor) 
      .p(ShooterConstants.kP)
      .i(ShooterConstants.kI)
      .d(ShooterConstants.kD)
      .outputRange(ShooterConstants.kMinOutputLimit,ShooterConstants.kMaxOutputLimit);
    indexConfig.softLimit
      .forwardSoftLimitEnabled(false)
      .forwardSoftLimit(ShooterConstants.kForwardSoftLimit) 
      .reverseSoftLimitEnabled(false)
      .reverseSoftLimit(ShooterConstants.kReverseSoftLimit);
    indexConfig.encoder
      .positionConversionFactor(ShooterConstants.kPositionCoversionFactor);
    
    
      index.configure(indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
  }

public void spin(double rpm){
    closedLoopControllerI.setSetpoint(rpm, ControlType.kVelocity);
  }

private double getVelocity(){
    return indexEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
