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
public final class Constants {
    // for our camera system
    public static class VisionConstants {
         // can id TODO: change to be accurate
        public static final int kCanID = 0;

        // config TODO: change to accurate
        public static final boolean kInverted = false;
        
        public static final int kStallLimit = 80;
        public static final int kFreeLimit = 40;
        public static final IdleMode kIdleMode = IdleMode.kCoast;

        // pid constants TODO: change to be hopefully accurate and pray it works
        public static final FeedbackSensor kSensor = FeedbackSensor.kPrimaryEncoder;
        public static final double kP = .001;
        public static final double kI = 0;
        public static final double kD = 0;
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
    // for the bot's auto actions
    public static class AutoConstants {
         public final double kWidthNLength = 27;
         public final double kwidthNLengthWithBumpers = 33;
        
    }
    // for the intake's pivot
    public static class PivotConstants {
        // ids
        public static final int kCanID = 41;
        // config
        public static final boolean kInverted = true;
        public static final int kStallLimit = 70;
        public static final int kFreeLimit = 50;
        public static final IdleMode kIdleMode = IdleMode.kBrake; // TODO: change to brake

        public static final double kOffset = 0;
        public static final boolean kAbsoluteEncoderInverted = true;

        // closed loop
        public static final FeedbackSensor kSensor = FeedbackSensor.kPrimaryEncoder;
        public static final double kP = .005;
        public static final double kI = 0;
        public static final double kD = .001;
        public static final double kFf = 0;
        public static final double kMinOutputLimit = -1;
        public static final double kMaxOutputLimit = 1;

        // soft limit
        public static final double kForwardSoftLimit = 130;
        public static final double kReverseSoftLimit = -60;

        // encoder 
        public static final double kPositionCoversionFactor = 1;
        public static final double kTolerance = 5;

        // set modes TODO: change after testing
        public static final double kStow = 4; 
        public static final double kSlide = 3;
        public static final double kintake = 1.2; 

    }
    // for the bot's intake subsystem
    public static class IntakeConstants {
         // can id TODO: change to be accurate
        public static final int kCanID = 2;

        // config TODO: change to accurate
        public static final boolean kInverted = true;
        
        public static final int kStallLimit = 60;
        public static final int kFreeLimit = 40;
        public static final IdleMode kIdleMode = IdleMode.kCoast;

        // pid constants TODO: change to be hopefully accurate and pray it works
        public static final FeedbackSensor kSensor = FeedbackSensor.kPrimaryEncoder;
        public static final double kP = .001;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kMinOutputLimit = -.8;
        public static final double kMaxOutputLimit = .8;

        // soft limits (will be unused for shooter)
        public static final double kForwardSoftLimit = 63;
        public static final double kReverseSoftLimit = -1;

        // encoder constants TODO: change to be accurate
        public static final double kPositionCoversionFactor = 1;

        // tolerance to compare the shooter speed error with
        public static final double kTolerance = 20; // TODO: change if needed
        // set speeds
        public static final double kIntake = .5;
        public static final double kOutake = -.5;
    }
    // For the bot's indexer subsystem
    public static class IndexConstants {
         // can id TODO: change to be accurate
        public static final int kCanID = 6;

        // config TODO: change to accurate
        public static final boolean kInverted = false;
        
        public static final int kStallLimit = 60;
        public static final int kFreeLimit = 40;
        public static final IdleMode kIdleMode = IdleMode.kCoast;

        // pid constants TODO: change to be hopefully accurate and pray it works
        public static final FeedbackSensor kSensor = FeedbackSensor.kPrimaryEncoder;
        public static final double kP = .001;
        public static final double kI = 0;
        public static final double kD = 0.001;
        public static final double kMinOutputLimit = -.8;
        public static final double kMaxOutputLimit = .8;

        // soft limits (will be unused for shooter)
        public static final double kForwardSoftLimit = 63;
        public static final double kReverseSoftLimit = -1;

        // encoder constants TODO: change to be accurate
        public static final double kPositionCoversionFactor = 1;

        // tolerance to compare the shooter speed error with
        public static final double kTolerance = 40; // TODO: change if needed

        // set speeds
        public static final double kIndex = .8;
        public static final double kUnclog = -.6;
    }
    // for the feeder into the shooter
    public static class FeederConstants {
         // can id TODO: change to be accurate
        public static final int kCanID = 3;

        // config TODO: change to accurate
        public static final boolean kInverted = false;
        
        public static final int kStallLimit = 60;
        public static final int kFreeLimit = 40;
        public static final IdleMode kIdleMode = IdleMode.kCoast;

        // pid constants TODO: change to be hopefully accurate and pray it works
        public static final FeedbackSensor kSensor = FeedbackSensor.kPrimaryEncoder;
        public static final double kP = .001;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kMinOutputLimit = -1;
        public static final double kMaxOutputLimit = 1;

        // soft limits (will be unused for shooter)
        public static final double kForwardSoftLimit = 63;
        public static final double kReverseSoftLimit = -1;

        // encoder constants TODO: change to be accurate
        public static final double kPositionCoversionFactor = 1;

        // tolerance to compare the shooter speed error with
        public static final double kTolerance = 40; // TODO: change if needed

        // set speeds
        public static final double kFeed = .8;
        public static final double kUnclog = -.8;
    }
    // For the shooter subsystem
    public static class ShooterConstants {
        // can id TODO: change to be accurate
        public static final int kLCanID = 5;
        public static final int kRCanRID = 4;

        // config TODO: change to accurate
        public static final boolean kLInverted = true;
        public static final boolean kRInverted = false;
        public static final int kStallLimit = 80;
        public static final int kFreeLimit = 40;
        public static final IdleMode kIdleMode = IdleMode.kCoast;

        // pid constants TODO: change to be hopefully accurate and pray it works
        public static final FeedbackSensor kSensor = FeedbackSensor.kPrimaryEncoder;
        public static final double kP = .03;
        public static final double kI = 0;
        public static final double kD = 0.01;
        public static final double kMinOutputLimit = -1;
        public static final double kMaxOutputLimit = 1;

        // soft limits (will be unused for shooter)
        public static final double kForwardSoftLimit = 63;
        public static final double kReverseSoftLimit = -1;

        // encoder constants TODO: change to be accurate
        public static final double kPositionCoversionFactor = 1;

        // tolerance to compare the shooter speed error with
        public static final double kTolerance = 40; // TODO: change if needed

        // variables for use
        public static final double kMaxRPM = 6000;

        // set speeds
        public static final double klob = 4000;
        public static final double kWhyWouldYouUseThis = -600;
    }
}
