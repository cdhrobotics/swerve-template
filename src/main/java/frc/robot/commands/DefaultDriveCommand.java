package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final SlewRateLimiter m_xLimiter;
    private final SlewRateLimiter m_yLimiter;
    private final SlewRateLimiter m_roRateLimiter;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            SlewRateLimiter xLimiter,
            SlewRateLimiter yLimiter,
            SlewRateLimiter roRateLimiter) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_xLimiter = xLimiter;
        this.m_yLimiter = yLimiter;
        this.m_roRateLimiter = roRateLimiter;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {

        double xValue = m_translationXSupplier.getAsDouble();
        double yValue = m_translationYSupplier.getAsDouble();
        double rotateValue = m_rotationSupplier.getAsDouble();

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_xLimiter.calculate(Math.copySign(
                                xValue * xValue,
                                xValue)),
                        m_yLimiter.calculate(Math.copySign(yValue * yValue,
                                yValue)),
                        m_roRateLimiter.calculate(Math.copySign(rotateValue * rotateValue, rotateValue)),

                        m_drivetrainSubsystem.getGyroscopeRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
