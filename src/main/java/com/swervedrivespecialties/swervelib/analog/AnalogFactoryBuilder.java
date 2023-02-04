package com.swervedrivespecialties.swervelib.analog;

import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoderFactory;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class AnalogFactoryBuilder {

    public AbsoluteEncoderFactory<AnalogAbsoluteConfiguration> build() {
        return configuration -> {
            AnalogEncoder encoder = new AnalogEncoder(configuration.getId());

            return new EncoderImplementation(encoder);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {

        private final AnalogEncoder analogEncoder;

        private EncoderImplementation(AnalogEncoder analogEncoder) {
            this.analogEncoder = analogEncoder;
        }

        @Override
        public double getAbsoluteAngle() {
            // double absoluteAngle = analogEncoder.getAbsolutePosition();

            // // absoluteAngle %= 2.0 * Math.PI;
            // // if (absoluteAngle < 0.0) {
            // //     absoluteAngle += 2.0 * Math.PI;
            // // }

            // // This magic number is to get us to 360
            // return Math.toRadians(absoluteAngle * 6.271777); 

            double absoluteAngle = Math.toRadians(analogEncoder.get());
            absoluteAngle %= 2.0 * Math.PI;
            if (absoluteAngle < 0.0) {
                absoluteAngle += 2.0 * Math.PI;
            }
            return absoluteAngle;
        }

        @Override
        public double getRawAbsoluteAngle() {
            double absoluteRawAngle = analogEncoder.getAbsolutePosition();
            return absoluteRawAngle;
        }
    }
}