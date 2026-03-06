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

  private SparkClosedLoopController closedLoopControllerL, closedLoopControllerR;
  private RelativeEncoder shooterEncoder;
  private SparkMaxConfig shooterLConfig;
  private SparkMaxConfig shooterRConfig;
  public shooter() {
    shooterL = new SparkMax(ShooterConstants.kLCanID, MotorType.kBrushless); // main motor
    shooterR = new SparkMax(ShooterConstants.kRCanRID, MotorType.kBrushless); // will follow in reverse

    shooterEncoder = shooterL.getEncoder(); // encoder of main motor
    closedLoopControllerL = shooterL.getClosedLoopController(); // closed loop controller of left motor
    closedLoopControllerR = shooterR.getClosedLoopController(); // closed loop controller of left motor

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
      .apply(shooterLConfig)
      .inverted(ShooterConstants.kRInverted);

      shooterL.configure(shooterLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      // shooterR.configure(shooterRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void spin(double rpm){
    closedLoopControllerL.setSetpoint(rpm, ControlType.kVelocity);
    closedLoopControllerR.setSetpoint(rpm, ControlType.kVelocity);
  }

  public void stop(){
    shooterL.stopMotor();
    shooterR.stopMotor();
  }

  private double getVelocity(){
    return shooterEncoder.getVelocity();
  }

  private double getError() {
    return Math.abs(Math.abs(getVelocity()) - Math.abs(closedLoopControllerL.getSetpoint()));
  }

  private boolean isAtSetpoint(){
    return (getError() < ShooterConstants.kTolerance);
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
