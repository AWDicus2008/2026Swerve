// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    //Motor vars
    //R/L as viewed from front label on CAD, Intake arm front in real
    //Left faces opposite from the center, right, and feeder motors
    //Clockwise(-) to shoot, CounterClockwise(+) to reverse
    //^ to be switched later
    TalonFX m_shooterLeft = new TalonFX(15);
    TalonFX m_shooterCenter = new TalonFX(16);
    TalonFX m_shooterRight = new TalonFX(17);
    TalonFX m_shooterFeeder = new TalonFX(18);

    TalonFX m_climber = new TalonFX(19);

    TalonFX m_intakeArm = new TalonFX(20);
    TalonFX m_intakeWheels = new TalonFX(21);

    //Motor Config vars
    TalonFXConfiguration c_shooterLeft = new TalonFXConfiguration();
    TalonFXConfiguration c_shooterCenter = new TalonFXConfiguration();
    TalonFXConfiguration c_shooterRight = new TalonFXConfiguration();
    TalonFXConfiguration c_shooterFeeder = new TalonFXConfiguration();

    TalonFXConfiguration c_climber = new TalonFXConfiguration();

    TalonFXConfiguration c_intakeArm = new TalonFXConfiguration();
    TalonFXConfiguration c_intakeWheels = new TalonFXConfiguration();

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        //afformentioned later
        c_shooterCenter.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_shooterCenter.getConfigurator().apply(c_shooterCenter);

        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
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
