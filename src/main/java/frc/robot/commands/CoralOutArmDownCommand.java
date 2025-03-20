package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.RollerSubsystem;

public class CoralOutArmDownCommand {
    private final CoralOutCommand m_coral;
    private final ArmDownCommand m_arm;

    public CoralOutArmDownCommand(RollerSubsystem roller, ArmSubsystem arm, double time1, double time2) {
        m_coral = new CoralOutCommand(roller, time1);
        m_arm = new ArmDownCommand(arm, time2);
    }
}
