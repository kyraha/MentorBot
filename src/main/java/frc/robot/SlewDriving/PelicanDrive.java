package frc.robot.SlewDriving;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain.SwerveChassis;

public class PelicanDrive extends Command {
    static final double turnRateMin = 4;
    static final double deadband = 0.05; // In fractions of max speed
    ContinuousSlewRateLimiter thetaLimiter = new ContinuousSlewRateLimiter(0);
    SlewRateLimiter driveLimiter = new SlewRateLimiter(0.5, -2, 0);
    SlewRateLimiter rotationLimiter = new SlewRateLimiter(2); // Radians per second squared
    private boolean isThetaKnown = false;
    private SwerveChassis chassis;
    private GenericHID operController;
    private Time dT;
    private final SwerveRequest.FieldCentric driveRequest;

    public PelicanDrive(SwerveChassis drivetrainToUse, GenericHID controller, double periodSeconds) {
        // Use addRequirements() here to declare subsystem dependencies.
        chassis = drivetrainToUse;
        this.addRequirements(chassis);
        operController = controller;
        dT = Seconds.of(periodSeconds);
        driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(deadband * chassis.kMaxSpeed)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isThetaKnown = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        final var xStick = -operController.getRawAxis(1);
        final var yStick = -operController.getRawAxis(0);
        final var wStick = -operController.getRawAxis(2); // PS5: 2, XBox: 4

        var request = accelLimitVectorDrive(
            xStick * chassis.kMaxSpeed,
            yStick * chassis.kMaxSpeed,
            wStick * chassis.kMaxAngularSpeed
        );

        chassis.applyRequest(request, dT);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        chassis.driveVelocities(new ChassisSpeeds(0, 0, 0), dT);
        // idleRequest.apply(..., ...);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public SwerveRequest.FieldCentric accelLimitVectorDrive(double xAxis, double yAxis, double rotation) {
        Translation2d vector = new Translation2d(xAxis, yAxis);
        vector = limitThetaMag(vector);
        rotation = rotationLimiter.calculate(rotation);
        return driveRequest.withVelocityX(vector.getX()).withVelocityY(vector.getY()).withRotationalRate(rotation);
    }

    public Translation2d limitThetaMag(Translation2d vector) {
        double desiredMag = vector.getNorm();
        double desiredAngle = vector.getAngle().getRadians();

        if (isThetaKnown) {
            // We were moving so the previous angle is real and the previous magnitude is non-zero
            double cos = Math.cos(thetaLimiter.getDelta(desiredAngle));
            if (cos > 0) {
                // Delta angle is "small" so drive with new angle, but limit its rate of change
                double lastMag = driveLimiter.lastValue(); // preserve last magnitude for avarage
                double limitedMag = driveLimiter.calculate(desiredMag * cos);
                double limit = 2.0 * turnRateMin / (lastMag + limitedMag);
                thetaLimiter.updateLmitis(limit, -limit);
                Rotation2d limitedRot = new Rotation2d(thetaLimiter.calculate(desiredAngle));
                return new Translation2d(limitedMag, limitedRot);
            }
            else {
                // Delta angle is too wide, so keep the current angle and decelerate to zero
                double lastAngle = thetaLimiter.lastValue();
                double limitedMag = driveLimiter.calculate(0);
                if (limitedMag < deadband * chassis.kMaxSpeed) {
                    // When we're under the deadband we're effectively stopped
                    isThetaKnown = false;
                    return Translation2d.kZero;
                }
                // Still decelerating? Then keep the last angle
                thetaLimiter.reset(lastAngle);
                return new Translation2d(limitedMag, new Rotation2d(lastAngle));
            }
        }
        else {
            // Theta is not known, so thetaLimiter is not used
            if(desiredMag < deadband * chassis.kMaxSpeed) {
                // Still within deadband? Then stay stopped but reset driveLimiter
                driveLimiter.reset(0);
                return Translation2d.kZero;
            }
            else {
                // Wake up and start moving
                isThetaKnown = true;
                Rotation2d newAngle = vector.getAngle();
                thetaLimiter.reset(newAngle.getRadians());
                desiredMag = driveLimiter.calculate(desiredMag);
                return new Translation2d(desiredMag, newAngle);
            }
        }
    }
}
