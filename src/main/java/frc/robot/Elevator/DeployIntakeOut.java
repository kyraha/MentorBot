package frc.robot.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class DeployIntakeOut extends Command{
    IntakeArmSubsystem arm;
    public DeployIntakeOut(IntakeArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.updateConfig();
        arm.intakeOut();
    }

    @Override
    public void end(boolean interrupted) {
        arm.intakeIn();
    }
}
