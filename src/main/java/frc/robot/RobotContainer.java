// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric botdrive = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentricFacingAngle face = new SwerveRequest.FieldCentricFacingAngle();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final shooter shooter = new shooter();

    public final Intake intake = new Intake();

    public final Index index = new Index();

    public final Feeder feeder = new Feeder();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // named commands for auto
        NamedCommands.registerCommand(
        "intake", 
        intake.setPos(PivotConstants.kintake)
        .andThen(intake.setRPM(IntakeConstants.kIntake))
        );
        NamedCommands.registerCommand(
        "aim",
        shooter.setRPM(ShooterConstants.klob)
        .alongWith(feeder.setRPM(FeederConstants.kFeed))
        );
        NamedCommands.registerCommand(
            "shoot",
            index.setRPM(IndexConstants.kIndex)
            .alongWith(intake.setPos(PivotConstants.kSlide))
        );
        NamedCommands.registerCommand(
            "unclog",
            index.setRPM(IndexConstants.kUnclog)
            .alongWith(feeder.setRPM(FeederConstants.kUnclog))
        );


        autoChooser = AutoBuilder.buildAutoChooser("middle shoot");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                botdrive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on d-pad up press.
        joystick.povUp().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // main triggers for the superstructure

        final Trigger tIntake = joystick.leftBumper(); // sets the pivot to it's intake position and runs motor with a toggle
        final Trigger tAim = joystick.leftTrigger(.5); // aims the robot towards its target and sets the motor rpm depending on the distance from target (hold)
        final Trigger tShoot = joystick.rightTrigger(.5); // runs the index system to feed to shooter
        final Trigger tShift = joystick.rightBumper();

        // what the triggers do TODO: change after belton
        tIntake.onTrue(
            intake.setPos(PivotConstants.kintake)
            .andThen(intake.setRPM(IntakeConstants.kIntake))
        );
        tIntake.onFalse(
            intake.stopMotors()
            .andThen(intake.setPos(PivotConstants.kintake))
        );

        tAim.onTrue(
            shooter.setRPM(ShooterConstants.klob)
            
            .alongWith(feeder.setRPM(FeederConstants.kFeed))
        );
        tAim.onFalse(
            shooter.stopMotors()
            .alongWith(feeder.stopMotors())
        );

        tShoot.onTrue(
            new RepeatCommand(
                index.setRPM(IndexConstants.kIndex)
                .andThen(new WaitCommand(1.5))
                .andThen(index.setRPM(IndexConstants.kUnclog))
                .andThen(new WaitCommand(.25))
            )
            
        );
        tShoot.onFalse(
            index.stopMotors()
        );
        tShift.onTrue(
            intake.shift()
        );
        
    }

    public Command getAutonomousCommand() {
        // return autoChooser.getSelected(); TODO: set up correctly
        return drivetrain.applyRequest(() ->
                drive.withVelocityX(0) // Drive forward with negative Y (forward)
                    .withVelocityY(0) // Drive left with negative X (left)
                    .withRotationalRate(0) // Drive counterclockwise with negative X (left)
            );
        
    }
}
