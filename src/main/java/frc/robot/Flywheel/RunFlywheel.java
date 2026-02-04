package frc.robot.Flywheel;

import edu.wpi.first.wpilibj2.command.Command;

public class RunFlywheel extends Command{
    public final FlywheelSubsystem flyWheel;

    public RunFlywheel(FlywheelSubsystem subsystem) {
        flyWheel = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        flyWheel.revAtVelocity(100);
    }

    @Override
    public void end(boolean interrupted) {
        flyWheel.stopShooter();
    }
}
