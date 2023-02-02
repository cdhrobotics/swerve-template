package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter turnLimiter;

    private final double DEADBAND = 0.05;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        xLimiter = m_drivetrainSubsystem.getXLimiter();
        yLimiter = m_drivetrainSubsystem.getYLimiter();
        turnLimiter = m_drivetrainSubsystem.getTurnLimiter();

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        if(m_drivetrainSubsystem.getSelectorDriveMode() == m_drivetrainSubsystem.kField) {
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    -modifyAxis(m_translationXSupplier.getAsDouble(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                    modifyAxis(m_translationYSupplier.getAsDouble(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                    getTurnValue(),
                    m_drivetrainSubsystem.getGyroscopeRotation()
                )
                
            );
            System.out.println("Calling Field");
        } else {
            m_drivetrainSubsystem.drive(
                new ChassisSpeeds(
                    -modifyAxis(m_translationXSupplier.getAsDouble(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                    modifyAxis(m_translationYSupplier.getAsDouble(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                    getTurnValue()
                )
            );
            System.out.println("Calling Robot");
        }

        SmartDashboard.putNumber("FL Module angle: " , this.m_drivetrainSubsystem.m_frontLeftModule.getSteerAngle());
        SmartDashboard.putNumber("BL Module angle: " , this.m_drivetrainSubsystem.m_backLeftModule.getSteerAngle());
        // System.out.println(this.m_drivetrainSubsystem.m_frontLeftModule.getSteerMotor().getAbsoluteAngle();
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    protected double getTurnValue() {
        return -modifyAxis(m_rotationSupplier.getAsDouble(), turnLimiter) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    private double modifyAxis(double value, SlewRateLimiter limiter) {
        // Deadband
        value = deadband(value, 0.05);
        // Square the axis for finer control at lower values
        value = limiter.calculate(Math.copySign(value * value, value));
        
        return value;
    }

    private double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

}

