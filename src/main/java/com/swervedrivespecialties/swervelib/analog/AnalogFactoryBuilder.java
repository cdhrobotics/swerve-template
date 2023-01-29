package com.swervedrivespecialties.swervelib.analog;

import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoderFactory;

import edu.wpi.first.wpilibj.AnalogEncoder;

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
            return analogEncoder.get();
        }
    }
}