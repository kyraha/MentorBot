package frc.robot.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;

public class PelicanDriver extends Command {
    private final CommandSwerveDrivetrain chassis;
    private final OI operator;
    private final PelicanLimiter limiter = new PelicanLimiter();


    public PelicanDriver(CommandSwerveDrivetrain drivetrainToUse, OI operatorInterface) {
        chassis = drivetrainToUse;
        operator = operatorInterface;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        ChassisSpeeds HIDSpeeds = operator.getSpeedsFromHID();
        // TBD: scale the speeds by the hight of the elevator
        ChassisSpeeds limitedSpeeds = limiter.accelLimitVectorDrive(HIDSpeeds);
        chassis.driveOperatorOriented(limitedSpeeds);
    }

}
