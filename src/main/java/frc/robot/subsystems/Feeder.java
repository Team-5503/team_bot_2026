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

import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IndexConstants;

public class Feeder extends SubsystemBase {

  SparkMax feeder;

  private SparkClosedLoopController closedLoopControllerF;
  private RelativeEncoder feederEncoder;
  private SparkMaxConfig feederConfig;


  /** Creates a new Feeder. */
  public Feeder() {
     feeder = new SparkMax(FeederConstants.kCanID, MotorType.kBrushless); // will grip to balls

    feederEncoder = feeder.getEncoder(); // encoder of main motor
    closedLoopControllerF = feeder.getClosedLoopController(); // closed loop controller of main motor

    configure();
  }


 private void configure() {
    feederConfig = new SparkMaxConfig();
    feederConfig
      .inverted(FeederConstants.kInverted)
      .smartCurrentLimit(FeederConstants.kStallLimit, FeederConstants.kFreeLimit)
      .idleMode(FeederConstants.kIdleMode); 
    feederConfig.closedLoop
      .feedbackSensor(FeederConstants.kSensor) 
      .p(FeederConstants.kP)
      .i(FeederConstants.kI)
      .d(FeederConstants.kD)
      .outputRange(FeederConstants.kMinOutputLimit,FeederConstants.kMaxOutputLimit);
    feederConfig.softLimit
      .forwardSoftLimitEnabled(false)
      .forwardSoftLimit(FeederConstants.kForwardSoftLimit) 
      .reverseSoftLimitEnabled(false)
      .reverseSoftLimit(FeederConstants.kReverseSoftLimit);
    feederConfig.encoder
      .positionConversionFactor(FeederConstants.kPositionCoversionFactor);
    
    
      feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
  }

public void spin(double rpm){
    closedLoopControllerF.setSetpoint(rpm, ControlType.kVelocity);
  }

  public void stop(){
    feeder.stopMotor();
  }

  private double getVelocity(){
    return feederEncoder.getVelocity();
  }

  private double getError() {
    return Math.abs(Math.abs(getVelocity()) - Math.abs(closedLoopControllerF.getSetpoint()));
  }

  private boolean isAtSetpoint(){
    return (getError() < FeederConstants.kTolerance);
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