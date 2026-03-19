package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Hood;

public class MoveLinActs extends InstantCommand {
    private final Hood m_subsystem;
    private final boolean m_extend;

    public MoveLinActs(Hood subsystem, boolean extend) {
        m_subsystem = subsystem;
        m_extend = extend;
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        if (m_extend) {
            m_subsystem.extend();
        } else {
            m_subsystem.retract();
        }
    }
}