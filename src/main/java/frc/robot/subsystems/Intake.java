// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;

public class Intake extends SubsystemBase {
  SparkMax intake, pivot;

  private SparkClosedLoopController closedLoopControllerP;
  private RelativeEncoder pivotEncoder;
  private AbsoluteEncoder absolutePivotEncoder;
  private SparkClosedLoopController closedLoopControllerI;
  private RelativeEncoder intakeEncoder;
  private SparkMaxConfig intakeConfig;
  private SparkMaxConfig pivotConfig;
  private double targetPos; // TODO: set up for pivot
  /** Creates a new Intake. */
  public Intake() {
    intake = new SparkMax(IntakeConstants.kCanID, MotorType.kBrushless); // will grip the balls
    pivot = new SparkMax(PivotConstants.kCanID, MotorType.kBrushless);

    intakeEncoder = intake.getEncoder(); // encoder of main motor

    pivotEncoder = pivot.getEncoder(); // internal encoder of pivot
    absolutePivotEncoder = pivot.getAbsoluteEncoder(); // the absolute encoder of pivot

    closedLoopControllerI = intake.getClosedLoopController(); // closed loop controller of main motor
    closedLoopControllerP = pivot.getClosedLoopController();
    targetPos = 0;

    configure();
  }
  

  private void configure() {
    intakeConfig = new SparkMaxConfig();
    intakeConfig
      .inverted(IntakeConstants.kInverted)
      .smartCurrentLimit(IntakeConstants.kStallLimit, IntakeConstants.kFreeLimit)
      .idleMode(IntakeConstants.kIdleMode); 
    intakeConfig.closedLoop
      .feedbackSensor(IntakeConstants.kSensor) 
      .p(IntakeConstants.kP)
      .i(IntakeConstants.kI)
      .d(IntakeConstants.kD)
      .outputRange(IntakeConstants.kMinOutputLimit,IntakeConstants.kMaxOutputLimit);
    intakeConfig.softLimit
      .forwardSoftLimitEnabled(false)
      .forwardSoftLimit(IntakeConstants.kForwardSoftLimit) 
      .reverseSoftLimitEnabled(false)
      .reverseSoftLimit(IntakeConstants.kReverseSoftLimit);
    intakeConfig.encoder
      .positionConversionFactor(IntakeConstants.kPositionCoversionFactor);


    // for the pivot
    pivotConfig = new SparkMaxConfig();
    pivotConfig
      .inverted(PivotConstants.kInverted)
      .smartCurrentLimit(PivotConstants.kStallLimit, PivotConstants.kFreeLimit)
      .idleMode(PivotConstants.kIdleMode); 
    pivotConfig.closedLoop
      .feedbackSensor(PivotConstants.kSensor) 
      .positionWrappingEnabled(false)
      .positionWrappingInputRange(0, 360)
      .p(PivotConstants.kP)
      .i(PivotConstants.kI)
      .d(PivotConstants.kD)
      .outputRange(PivotConstants.kMinOutputLimit,PivotConstants.kMaxOutputLimit);
    pivotConfig.softLimit
      .forwardSoftLimitEnabled(false)
      //.forwardSoftLimit(PivotConstants.kForwardSoftLimit) 
      .reverseSoftLimitEnabled(false);
      //.reverseSoftLimit(PivotConstants.kReverseSoftLimit);
    pivotConfig.encoder
      .positionConversionFactor(PivotConstants.kPositionCoversionFactor);
    pivotConfig.absoluteEncoder
    .positionConversionFactor(PivotConstants.kPositionCoversionFactor)
    .zeroCentered(false)
    .inverted(PivotConstants.kAbsoluteEncoderInverted);

    
    
      intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
  }
  // functions to give targets to the motors

  public void setPosition(double pos){
    closedLoopControllerP.setSetpoint(pos, ControlType.kPosition);
  }
  // functions to return info on the encoders for future use

  

  public void spin(double rpm){
    // closedLoopControllerI.setSetpoint(rpm, ControlType.kVelocity); TODO: fix
    intake.set(rpm);
  }

  public void stop(){
    intake.stopMotor();
    pivot.stopMotor();
  }

  private double getVelocity(){
    return intakeEncoder.getVelocity();
  }

  private double getPos(){
    return absolutePivotEncoder.getPosition();
  }

  private double getError() {
    return Math.abs(Math.abs(getVelocity()) - Math.abs(closedLoopControllerI.getSetpoint()));
  }

  private double getPosError() {
    return Math.abs(Math.abs(getPos()) - Math.abs(closedLoopControllerP.getSetpoint()));
  }

  private boolean isAtRPM(){
    return (getError() < IntakeConstants.kTolerance);
  }

  private boolean isAtPos(){
    return (getPosError() < PivotConstants.kTolerance);
  }




 /*
   * COMMANDS THAT DO NOT SET ANYthing
   * TODO: SEE IF WE NEED TO MOVE THIS TO ITS OWN COMMAND FILE
   */

  public Command waitUntilAtVelocity() {
    return new WaitUntilCommand(() -> {
      // TEST FOR IF PIVOTERROR IS IN TOLERANCE OF TARGETPOSITION
      return isAtRPM();
    });
  }

  public Command waitUntilAtPos() {
    return new WaitUntilCommand(() -> {
      // TEST FOR IF PIVOTERROR IS IN TOLERANCE OF TARGETPOSITION
      return isAtPos();
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

   public Command setPos(double pos){
    return runOnce(() -> {
      setPosition(pos);
      targetPos = pos;
    });
   }

   public Command shift(){
    return runOnce(() -> {
    if (targetPos == PivotConstants.kintake){
      targetPos = PivotConstants.kSlide;
      setPosition(targetPos);
    }
    else{
      targetPos = PivotConstants.kintake;
      setPosition(targetPos);
    }
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
    SmartDashboard.putNumber("pivot positon", pivotEncoder.getPosition());
    SmartDashboard.putNumber("intake RPM", intakeEncoder.getVelocity());
  }
}
