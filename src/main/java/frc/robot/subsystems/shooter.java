// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.Constants.ShooterConstants;

public class shooter extends SubsystemBase { // TODO: recode to be pheonix6 motors
  /** Creates a new shooter. */
  SparkMax shooterL, shooterR;
  private final CANBus canBus = new CANBus("rio");
  private final TalonFX shooterFxL = new TalonFX(24, canBus); // TODO: change can id
  private final TalonFX shooterFxR = new TalonFX(25, canBus); // TODO: change can id

  private NeutralOut m_stopDaFuckingMotorPlease = new NeutralOut(); //TODO: change name

  /* Be able to switch which control request to use based on a button press */
  /* Start at velocity 0, use slot 0 */
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  /* Start at velocity 0, use slot 1 */
  private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);

  private SparkClosedLoopController closedLoopControllerL, closedLoopControllerR;
  private RelativeEncoder shooterLEncoder, shooterREncoder;
  private SparkMaxConfig shooterLConfig;
  private SparkMaxConfig shooterRConfig;
  public shooter() {
    // shooterL = new SparkMax(ShooterConstants.kLCanID, MotorType.kBrushless); // main motor
    // shooterR = new SparkMax(ShooterConstants.kRCanRID, MotorType.kBrushless); // will follow in reverse

    // shooterLEncoder = shooterL.getEncoder(); // encoder of main motor
    // shooterREncoder = shooterR.getEncoder();
    // closedLoopControllerL = shooterL.getClosedLoopController(); // closed loop controller of left motor
    // closedLoopControllerR = shooterR.getClosedLoopController(); // closed loop controller of left motor

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
      .idleMode(ShooterConstants.kIdleMode); 
    shooterRConfig.closedLoop
      .feedbackSensor(ShooterConstants.kSensor) 
      .p(ShooterConstants.kP)
      .i(ShooterConstants.kI)
      .d(ShooterConstants.kD)
      .outputRange(ShooterConstants.kMinOutputLimit,ShooterConstants.kMaxOutputLimit);
    shooterRConfig.softLimit
      .forwardSoftLimitEnabled(false)
      .forwardSoftLimit(ShooterConstants.kForwardSoftLimit) 
      .reverseSoftLimitEnabled(false)
      .reverseSoftLimit(ShooterConstants.kReverseSoftLimit);
    shooterRConfig.encoder
      .positionConversionFactor(ShooterConstants.kPositionCoversionFactor);

      // shooterL.configure(shooterLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      // shooterR.configure(shooterRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      // TalonFx configuation(totally not taken from the examples repo)
      TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0.0; // No output for error derivative
    // Peak output of 8 volts
    configs.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    configs.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
    configs.Slot1.kI = 0; // No output for integrated error
    configs.Slot1.kD = 2.5; // No output for error derivative
    // Peak output of 40 A
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
      .withPeakReverseTorqueCurrent(Amps.of(-40));
    configs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = shooterFxL.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    shooterFxR.setControl(new Follower(shooterFxL.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  public void spin(double rpm){
    // closedLoopControllerL.setSetpoint(rpm, ControlType.kVelocity);
    // closedLoopControllerR.setSetpoint(rpm, ControlType.kVelocity);
    shooterFxL.setControl(m_velocityVoltage.withVelocity(rpm));
  }

  public void stop(){
    // shooterL.stopMotor();
    // shooterR.stopMotor();
    // talonFx
    shooterFxL.setControl(m_stopDaFuckingMotorPlease);
  }

  private double getVelocity(){
    return shooterFxL.getVelocity().getValueAsDouble();
  }

  private double getError() {
    return Math.abs(Math.abs(getVelocity()) - Math.abs(closedLoopControllerL.getSetpoint())); // TODO: findout how to do this with krakens
  }

  private boolean isAtSetpoint(){
    return (getError() < ShooterConstants.kTolerance); // TODO: findout how to do this with krakens
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
    SmartDashboard.putNumber("left shooter RPM", getVelocity());
    // SmartDashboard.putNumber("left shooter setpoint", closedLoopControllerL.getSetpoint());
    // SmartDashboard.putNumber("Right shooter setpoint", closedLoopControllerR.getSetpoint());
    // SmartDashboard.putNumber("shooter Left current", shooterL.getOutputCurrent());
    // SmartDashboard.putNumber("shooter Left voltage", shooterL.getBusVoltage());

  }
}
