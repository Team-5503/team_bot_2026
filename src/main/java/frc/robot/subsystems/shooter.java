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

public class shooter extends SubsystemBase {
  /** Creates a new shooter. */
  SparkMax shooterL, shooterR;

  private SparkClosedLoopController closedLoopControllerS;
  private RelativeEncoder shooterEncoder;
  private SparkMaxConfig shooterLConfig;
  private SparkMaxConfig shooterRConfig;
  public shooter() {
    shooterL = new SparkMax(0, MotorType.kBrushless); // main motor
    shooterR = new SparkMax(0, MotorType.kBrushless); // will follow in reverse

    shooterEncoder = shooterL.getEncoder(); // encoder of main motor
    closedLoopControllerS = shooterL.getClosedLoopController(); // closed loop controller of main motor

    configure();
  }

  private void configure() {
    shooterLConfig = new SparkMaxConfig();
    shooterRConfig =new SparkMaxConfig();
    shooterLConfig
      .inverted(ShooterConstants.kLInverted)
      .smartCurrentLimit(ShooterConstants.kStallLimit, ShooterConstants.kFreeLimit)
      .idleMode(ShooterConstants.kIdleMode); 
    shooterLConfig.closedLoop
      .feedbackSensor(ShooterConstants.kSensor) 
      .p(ShooterConstants.kP)
      .i(ShooterConstants.kI)
      .d(ShooterConstants.kD)
      .outputRange(ShooterConstants.kMinOutputLimit,ShooterConstants.kMaxOutputLimit);
    shooterLConfig.softLimit
      .forwardSoftLimitEnabled(false)
      .forwardSoftLimit(ShooterConstants.kForwardSoftLimit) 
      .reverseSoftLimitEnabled(false)
      .reverseSoftLimit(ShooterConstants.kReverseSoftLimit);
    shooterLConfig.encoder
      .positionConversionFactor(ShooterConstants.kPositionCoversionFactor);
    
    shooterRConfig
      .inverted(ShooterConstants.kRInverted)
      .smartCurrentLimit(ShooterConstants.kStallLimit, ShooterConstants.kFreeLimit)
      .idleMode(ShooterConstants.kIdleMode)
      .follow(shooterL);

      shooterL.configure(shooterLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      shooterR.configure(shooterRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
