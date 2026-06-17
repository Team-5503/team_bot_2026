// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.PortForwarder;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    // private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
    //     .withTimestampReplay()
    //     .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        SignalLogger.enableAutoLogging(false);
        SignalLogger.stop();
        StatusLogger.disableAutoLogging();
        StatusLogger.stop();

         // (robotIP):5801 will now point to a Limelight3A's (id 0) web interface stream:
        // (robotIP):5800 will now point to a Limelight3A's (id 0) video stream:
        PortForwarder.add(5801, "172.29.0.1", 5801);
        PortForwarder.add(5802, "172.29.0.1", 5802);
        PortForwarder.add(5803, "172.29.0.1", 5803);
        PortForwarder.add(5804, "172.29.0.1", 5804);
        PortForwarder.add(5805, "172.29.0.1", 5805);
        PortForwarder.add(5806, "172.29.0.1", 5806);
        PortForwarder.add(5807, "172.29.0.1", 5807);
        PortForwarder.add(5808, "172.29.0.1", 5808);
        PortForwarder.add(5809, "172.29.0.1", 5809);

        // (robotIP):5811 will now point to a Limelight3A's (id 1) web interface stream:
        // (robotIP):5810 will now point to a Limelight3A's (id 1) video stream:
        PortForwarder.add(5811, "172.28.0.1", 5801);
        PortForwarder.add(5812, "172.28.0.1", 5802);
        PortForwarder.add(5813, "172.28.0.1", 5803);
        PortForwarder.add(5814, "172.28.0.1", 5804);
        PortForwarder.add(5815, "172.28.0.1", 5805);
        PortForwarder.add(5816, "172.28.0.1", 5806);
        PortForwarder.add(5817, "172.28.0.1", 5807);
        PortForwarder.add(5818, "172.28.0.1", 5808);
        PortForwarder.add(5819, "172.28.0.1", 5809);
        CameraServer.startAutomaticCapture();
    }

    @Override
    public void robotPeriodic() {
        // m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
