// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import org.photonvision.PhotonCamera;

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
import frc.robot.subsystems.LLShooter;
import frc.robot.subsystems.shooter;

public class RobotContainer {
    private double MaxSpeed = (1.0/3.0) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.02).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentric aimdrive = new SwerveRequest.FieldCentric()
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

    public final LLShooter limelight = new LLShooter();

    // public final Feeder feeder = new Feeder();

    public PhotonCamera ll3 = new PhotonCamera("ll3");

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // named commands for auto
        NamedCommands.registerCommand( 
        "intake", 
        intake.setRPM(IntakeConstants.kIntake)
        ); 
        NamedCommands.registerCommand( 
        "pivot to intake", 
        intake.setPos(PivotConstants.kintake)
        ); 
        NamedCommands.registerCommand( 
        "pivot to stow", 
        intake.setPos(PivotConstants.kStow)
        ); 
        NamedCommands.registerCommand(
        "aim 2-3 ft",
        shooter.setRPM(ShooterConstants.k2to3ft)
        );
        NamedCommands.registerCommand(
        "aim from tower",
        shooter.setRPM(ShooterConstants.ktower)
        );
        NamedCommands.registerCommand(
            "shoot",
            new RepeatCommand(
                index.setRPM(IndexConstants.kIndex)
                .andThen(new WaitCommand(1))
                .andThen(index.setRPM(IndexConstants.kUnclog))
                .andThen(new WaitCommand(.25))
            )
        );
        NamedCommands.registerCommand(
        "stop shooter",
        shooter.stopMotors()
        .alongWith(index.stopMotors())
        );
        NamedCommands.registerCommand(
            "unclog",
            index.setRPM(IndexConstants.kUnclog)
            //.alongWith(feeder.setRPM(FeederConstants.kUnclog))
        );
        NamedCommands.registerCommand(
            "autoaim",
            shooter.setRPM(ShooterConstants.ktower)
            .alongWith(drivetrain.applyRequest(() ->
                drive.withVelocityX(0 * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(LLShooter.getAimAssist() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ))
        );


        autoChooser = AutoBuilder.buildAutoChooser("middle shoot");
        
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        FollowPathCommand.warmupCommand().schedule();

        ll3.setDriverMode(true);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        intake.setDefaultCommand(intake.setRPM(IntakeConstants.kIntake));

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
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on d-pad up press.
        joystick.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // main triggers for the superstructure

        final Trigger tIntake = joystick.leftBumper(); // sets the pivot to it's intake position and runs motor with a toggle
        final Trigger tAim = joystick.leftTrigger(.3); // aims the robot towards its target and sets the motor rpm depending on the distance from target (hold)
        final Trigger tShoot = joystick.rightTrigger(.5); // runs the index system to feed to shooter
        final Trigger tShift = joystick.rightBumper();
        final Trigger tUnclogIntake = joystick.a();
        final Trigger tUnclogIndex = joystick.b();

        // what the triggers do 
        tIntake.onTrue( 
            intake.setRPM(IntakeConstants.kIntake)
            //.andThen(intake.setPos(PivotConstants.kintake))
        );
        tIntake.onFalse( 
            intake.stopMotors()
            
        );

        tAim.onTrue(
            shooter.setRPM(ShooterConstants.ktower)
            .alongWith(drivetrain.applyRequest(() ->
                botdrive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(LLShooter.getAimAssist() - joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ))
            
            //.alongWith(feeder.setRPM(FeederConstants.kFeed))
        );
        tAim.onFalse(
            shooter.stopMotors()
            //.alongWith(feeder.stopMotors())
        );

        tShoot.onTrue(
            new RepeatCommand(
                index.setRPM(IndexConstants.kIndex)
                .andThen(new WaitCommand(1))
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

        tUnclogIntake.onTrue( 
            intake.setRPM(IntakeConstants.kOutake)
            //.andThen(intake.setPos(PivotConstants.kintake))
        );
        tUnclogIntake.onFalse( 
            intake.setRPM(0)
            
        );

        tUnclogIndex.onTrue(
            index.setRPM(IndexConstants.kUnclog)
        );
        tUnclogIndex.onFalse(
            index.setRPM(0)
        );

        
        
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected(); //TODO: set up correctly
        // return drivetrain.applyRequest(() ->
        //         drive.withVelocityX(0) // Drive forward with negative Y (forward)
        //             .withVelocityY(0) // Drive left with negative X (left)
        //             .withRotationalRate(0) // Drive counterclockwise with negative X (left)
        //     );
        
    }
}
