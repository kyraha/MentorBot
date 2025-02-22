package frc.robot.SlewDriving;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
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
    public enum DriveMode {
        Stopped,
        Moving
    }

    private DriveMode mode;
    private SwerveChassis chassis;
    private GenericHID operController;
    private Time dT;
    private final SwerveRequest.FieldCentric driveRequest;
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    public PelicanDrive(SwerveChassis drivetrainToUse, GenericHID controller, double periodSeconds) {
        // Use addRequirements() here to declare subsystem dependencies.
        chassis = drivetrainToUse;
        operController = controller;
        dT = Seconds.of(periodSeconds);
        driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(deadband * chassis.kMaxSpeed)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mode = DriveMode.Stopped;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        final var xStick = -operController.getRawAxis(1);
        final var yStick = -operController.getRawAxis(0);
        final var wStick = -operController.getRawAxis(2);

        var request = accelLimitVectorDrive(
            xStick * chassis.kMaxSpeed,
            yStick * chassis.kMaxSpeed,
            wStick * chassis.kMaxAngularSpeed
        );

        var newSpeeds = new ChassisSpeeds(
            MathUtil.applyDeadband(request.VelocityX, deadband * chassis.kMaxSpeed),
            MathUtil.applyDeadband(request.VelocityY, deadband * chassis.kMaxSpeed),
            MathUtil.applyDeadband(request.RotationalRate, deadband * chassis.kMaxAngularSpeed)
        );

        chassis.driveVelocities(newSpeeds, dT);
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
        double mag = vector.getNorm();
        if(mag > deadband * chassis.kMaxSpeed) {
            if (mode == DriveMode.Stopped) {
                // We were idling so we can move in any direction
                mode = DriveMode.Moving;
                Rotation2d newAngle = vector.getAngle();
                thetaLimiter.reset(newAngle.getRadians());
                mag = driveLimiter.calculate(mag);
                vector = new Translation2d(mag, newAngle);
            }
            else {
                // We were moving so the previous angle is real
                double angle = thetaLimiter.lastValue();
                double cos = Math.cos(thetaLimiter.getDelta(vector.getAngle().getRadians()));
                if (cos > 0) {
                    // Delta angle is small so drive with new angle
                    mag = driveLimiter.calculate(mag * cos);
                    double limit = turnRateMin / mag;
                    thetaLimiter.updateLmitis(limit, -limit);
                    angle = thetaLimiter.calculate(vector.getAngle().getRadians());
                }
                else {
                    // Delta angle is wide so keep the current angle and decelerate to zero
                    thetaLimiter.reset(angle);
                    mag = driveLimiter.calculate(0);
                    if (mag < deadband * chassis.kMaxSpeed) {
                        // When we're under the deadband we're effectively stopped
                        mode = DriveMode.Stopped;
                    }
                }
                vector = new Translation2d(mag, new Rotation2d(angle));
            }
        }
        else {
            // Under deadband means we're idling
            driveLimiter.reset(0);
            thetaLimiter.reset(0);
            mode = DriveMode.Stopped;
        }
        return driveRequest.withVelocityX(vector.getX()).withVelocityY(vector.getY()).withRotationalRate(rotation);
    }
}
