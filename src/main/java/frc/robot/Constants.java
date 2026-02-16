// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.generated.TunerConstants;

import com.revrobotics.spark.FeedbackSensor;

/** holds the constants for every part of the robot.
    If it shouldn't be changed, it should be here. */
public class Constants {
    // for our camera system
    public static class VisionConstants {

    }
    // for the bot's auto actions
    public static class AutoConstants {

    }
    // for the bot's intake subsystem
    public static class IntakeConstants {

    }
    // For the bot's indexer subsystem
    public static class IndexConstants {

    }
    // for the feeder into the shooter
    public static class FeederConstants {

    }
    // For the shooter subsystem
    public static class ShooterConstants {
        // can id TODO: change to be accurate
        public static final int kLCanID = 0;
        public static final int kRCanRID = 0;

        // config TODO: change to accurate
        public static final boolean kLInverted = false;
        public static final boolean kRInverted = true;
        public static final int kStallLimit = 80;
        public static final int kFreeLimit = 40;
        public static final IdleMode kIdleMode = IdleMode.kCoast;

        // pid constants TODO: change to be hopefully accurate and pray it works
        public static final FeedbackSensor kSensor = FeedbackSensor.kPrimaryEncoder;
        public static final double kP = .02;
        public static final double kI = 0;
        public static final double kD = .05;
        public static final double kFf = 0;
        public static final double kMinOutputLimit = -.8;
        public static final double kMaxOutputLimit = .8;

        // soft limits (will be unused for shooter)
        public static final double kForwardSoftLimit = 63;
        public static final double kReverseSoftLimit = -1;

        // encoder constants TODO: change to be accurate
        public static final double kPositionCoversionFactor = 1;
        // tolerance to compare the shooter speed error with
        public static final double kTolerance = 40; // TODO: change if needed
    }
}
