package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class FakeDriving extends Command {
    SwerveChassis chassis;
    Timer timer;
    public FakeDriving(Subsystem subsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        chassis = (SwerveChassis) subsystem;
        timer = new Timer();
        addRequirements(chassis);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var x = timer.get() % 10;
        var y = (timer.get() / 10) % 7;
        var pose = new Pose2d(x, y, Rotation2d.kZero);
        chassis.resetOdometry(pose);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
