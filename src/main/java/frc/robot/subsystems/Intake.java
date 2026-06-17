// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import static edu.wpi.first.units.Units.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.BaseStatusSignal;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;

public class Intake extends SubsystemBase {
  SparkMax intake, pivot;

  // private SparkClosedLoopController closedLoopControllerP;
  // private RelativeEncoder pivotEncoder;
  // private AbsoluteEncoder absolutePivotEncoder;
  private SparkClosedLoopController closedLoopControllerI;
  private RelativeEncoder intakeEncoder;
  private SparkMaxConfig intakeConfig;
  // private SparkMaxConfig pivotConfig;
  private double targetPos; // TODO: set up for pivot

  private TalonFX m_fx = new TalonFX(23, new CANBus("rio"));
  private CANcoder cancoder = new CANcoder(26, new CANBus("rio"));

  private PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  /* Start at position 0, use slot 1 */
  private PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);

  private double troubleshootingPercent = 0;
  private double troubleshootingPos = 0;
  private boolean Troubleshooting = false;
  /** Creates a new Intake. */
  public Intake() {
    intake = new SparkMax(IntakeConstants.kCanID, MotorType.kBrushless); // will grip the balls
    // pivot = new SparkMax(PivotConstants.kCanID, MotorType.kBrushless);

    intakeEncoder = intake.getEncoder(); // encoder of main motor

    // pivotEncoder = pivot.getEncoder(); // internal encoder of pivot
    // absolutePivotEncoder = pivot.getAbsoluteEncoder(); // the absolute encoder of pivot

    closedLoopControllerI = intake.getClosedLoopController(); // closed loop controller of main motor
    // closedLoopControllerP = pivot.getClosedLoopController();
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
    // pivotConfig = new SparkMaxConfig();
    // pivotConfig
    //   .inverted(PivotConstants.kInverted)
    //   .smartCurrentLimit(PivotConstants.kStallLimit, PivotConstants.kFreeLimit)
    //   .idleMode(PivotConstants.kIdleMode); 
    // pivotConfig.closedLoop
    //   .feedbackSensor(PivotConstants.kSensor) 
    //   .positionWrappingEnabled(false)
    //   .positionWrappingInputRange(0, 360)
    //   .p(PivotConstants.kP)
    //   .i(PivotConstants.kI)
    //   .d(PivotConstants.kD)
    //   .minOutput(0)
    //   .outputRange(PivotConstants.kMinOutputLimit,PivotConstants.kMaxOutputLimit);
    // pivotConfig.softLimit
    //   .forwardSoftLimitEnabled(false)
    //   //.forwardSoftLimit(PivotConstants.kForwardSoftLimit) 
    //   .reverseSoftLimitEnabled(true)
    //   .reverseSoftLimit(PivotConstants.kReverseSoftLimit);
      
    // pivotConfig.encoder
    //   .positionConversionFactor(PivotConstants.kPositionCoversionFactor);
    // pivotConfig.absoluteEncoder
    // .positionConversionFactor(PivotConstants.kPositionCoversionFactor)
    // .zeroCentered(false)
    // .inverted(PivotConstants.kAbsoluteEncoderInverted);

    
    
      intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    configs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    configs.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
    configs.Slot0.withKS(0);
    configs.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
    configs.Slot0.withKG(0);
    // Peak output of 8 V
    configs.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    configs.Slot1.kP = 60; // An error of 1 rotation results in 60 A output
    configs.Slot1.kI = 0; // No output for integrated error
    configs.Slot1.kD = 6; // A velocity of 1 rps results in 6 A output
    // Peak output of 120 A
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(120))
      .withPeakReverseTorqueCurrent(Amps.of(-120));
    
    configs.Feedback.withRemoteCANcoder(cancoder);

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_fx.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    var toApply = new CANcoderConfiguration();
    // toApply.MagnetSensor.withMagnetOffset(-0.04296875);
    

    /* User can change the configs if they want, or leave it empty for factory-default */
    cancoder.getConfigurator().apply(toApply);

    /* Speed up signals to an appropriate rate */
    BaseStatusSignal.setUpdateFrequencyForAll(100, cancoder.getPosition(), cancoder.getVelocity());

      
  }
  // functions to give targets to the motors

  public void setPosition(double pos){
    // closedLoopControllerP.setSetpoint(pos, ControlType.kPosition);
    m_fx.setControl(m_positionVoltage.withPosition(pos));
  }
  // functions to return info on the encoders for future use

  

  public void spin(double rpm){
    // closedLoopControllerI.setSetpoint(rpm, ControlType.kVelocity); TODO: fix
    if (Troubleshooting){
      intake.set(troubleshootingPercent);
      // closedLoopControllerP.setSetpoint(troubleshootingPos, ControlType.kPosition);
    }
    else{
      intake.set(rpm);
    }
  }

  public void stop(){
    intake.stopMotor();
    pivot.stopMotor();
  }

  private double getVelocity(){
    return intakeEncoder.getVelocity();
  }

  private double getPos(){
    return cancoder.getAbsolutePosition().getValueAsDouble();
  }

  // private double getError() {
  //   return Math.abs(Math.abs(getVelocity()) - Math.abs(closedLoopControllerI.getSetpoint()));
  // }

  // private double getPosError() {
  //   return Math.abs(Math.abs(getPos()) - Math.abs(closedLoopControllerP.getSetpoint()));
  // }

  // private boolean isAtRPM(){
  //   return (getError() < IntakeConstants.kTolerance);
  // }

  // private boolean isAtPos(){
  //   return (getPosError() < PivotConstants.kTolerance);
  // }




 /*
   * COMMANDS THAT DO NOT SET ANYthing
   * TODO: SEE IF WE NEED TO MOVE THIS TO ITS OWN COMMAND FILE
   */

  // public Command waitUntilAtVelocity() {
  //   return new WaitUntilCommand(() -> {
  //     // TEST FOR IF PIVOTERROR IS IN TOLERANCE OF TARGETPOSITION
  //     return isAtRPM();
  //   });
  // }

  // public Command waitUntilAtPos() {
  //   return new WaitUntilCommand(() -> {
  //     // TEST FOR IF PIVOTERROR IS IN TOLERANCE OF TARGETPOSITION
  //     return isAtPos();
  //   });
  // }
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
    // SmartDashboard.putNumber("pivot positon", pivotEncoder.getPosition());
    SmartDashboard.putNumber("intake RPM", getVelocity());
    // troubleshootingPercent = SmartDashboard.getNumber("troubleshooting Percent for Intake", 0);
    // troubleshootingPos = SmartDashboard.getNumber("troubleshooting position for arm", 2);
    // Troubleshooting = SmartDashboard.getBoolean("is we troubleshooting intake", false);
    SmartDashboard.putNumber("pivot voltage", m_fx.getSupplyVoltage().getValueAsDouble());
    SmartDashboard.putNumber("pivot current", m_fx.getSupplyVoltage().getValueAsDouble());
    SmartDashboard.putNumber("pivot temp", m_fx.getProcessorTemp().getValueAsDouble());
    SmartDashboard.putNumber("pivot position", getPos());
    SmartDashboard.putNumber("intake voltage", intake.getBusVoltage());
    SmartDashboard.putNumber("intake current", intake.getOutputCurrent());
    SmartDashboard.putNumber("intake temp", intake.getMotorTemperature());
  }
}
