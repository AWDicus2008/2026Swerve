// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    
    private final RobotContainer m_robotContainer;

    //Motor vars
    //R/L as viewed from front label on CAD, Intake arm front in real
    //Left faces opposite from the center, right, and feeder motors
    //Clockwise(-) to shoot, CounterClockwise(+) to reverse
    //^ to be switched later
    private TalonFX m_shooterLeft = new TalonFX(15);
    private TalonFX m_shooterCenter = new TalonFX(16);
    private TalonFX m_shooterRight = new TalonFX(17);
    private TalonFX m_shooterFeeder = new TalonFX(18);

    private TalonFX m_climber = new TalonFX(19);

    private TalonFX m_intakeArm = new TalonFX(20);
    private TalonFX m_intakeWheels = new TalonFX(21);

    //Motor Config vars
    //private TalonFXConfiguration c_shooterLeft = new TalonFXConfiguration();
    private TalonFXConfiguration c_shooterCenter = new TalonFXConfiguration();
    //private TalonFXConfiguration c_shooterRight = new TalonFXConfiguration();
    //private TalonFXConfiguration c_shooterFeeder = new TalonFXConfiguration();

    //private TalonFXConfiguration c_climber = new TalonFXConfiguration();

    //private TalonFXConfiguration c_intakeArm = new TalonFXConfiguration();
    //private TalonFXConfiguration c_intakeWheels = new TalonFXConfiguration();

    //Controller
    private XboxController m_operator;

    //Multipliers
    private double shootMult = 0.70;
    private double intakeMult = 0.70;
    private double armMult = 0.60;
    private double climbMult = 0.70;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        //afformentioned later
        c_shooterCenter.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_shooterCenter.getConfigurator().apply(c_shooterCenter);

        m_shooterRight.setControl(new Follower(m_shooterCenter.getDeviceID(),  MotorAlignmentValue.Opposed));
        m_shooterFeeder.setControl(new Follower(m_shooterCenter.getDeviceID(),  MotorAlignmentValue.Opposed));
        m_shooterLeft.setControl(new Follower(m_shooterCenter.getDeviceID(),  MotorAlignmentValue.Aligned));

        m_robotContainer = new RobotContainer();
        m_operator = new XboxController(0);
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
    public void autonomousPeriodic() {
        m_shooterCenter.set(shootMult);
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
        if(m_operator.getRightTriggerAxis() > 0.2)
        {//Shoot
            m_shooterCenter.set(shootMult * m_operator.getRightTriggerAxis());
        }
        else if(m_operator.getRightBumperButton())
        {//Unstuck shooter column
            m_shooterCenter.set(-shootMult);
        }
        else
        {//Prevent infinite shooting
            m_shooterCenter.set(0);
        }

        if(m_operator.getLeftTriggerAxis() > 0.2)
        {//Intake balls
            m_intakeWheels.set(intakeMult);
        }
        else
        {//Prevent infinite intake
            m_intakeWheels.set(0);
        }

        if(m_operator.getXButton())
        {//Lower intake arm
            m_intakeArm.set(armMult);
        }
        else if(m_operator.getBButton())
        {//Raise intake arm
            m_intakeArm.set(-armMult);
        }
        else
        {//Prevent infinite movement
            m_intakeArm.set(0);
        }

        if(m_operator.getYButton())
        {
            m_climber.set(climbMult);
        }
        else if(m_operator.getAButton())
        {
            m_climber.set(-climbMult);
        }
        else
        {
            m_climber.set(0);
        }
    }

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
