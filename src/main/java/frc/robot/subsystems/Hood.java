package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;


public class Hood extends SubsystemBase {
    // PWM port 0
    private final Servo la_left = new Servo(0);
    private final Servo la_right = new Servo(1);
    
    
    // Positions are 0.01mm to 0.77mm
    private static final Distance kServoLength = Millimeters.of(100);
    private static final LinearVelocity kMaxServoSpeed = Millimeters.of(20).per(Second);
    private static final double kMinPosition = 0.01;
    private static final double kMaxPosition = 0.77;
    private static final double kPositionTolerance = 0.01;

    public void extend() {
        la_left.set(kMaxPosition);
        la_right.set(kMaxPosition);
    }

    public void retract() {
        la_left.set(kMinPosition);
        la_right.set(kMinPosition);
    }

    public void stop() {
        // Optional: Actuonix servos usually don't need a "stop", 
        // they stop at their position, but you can set to current if needed.

    }
}