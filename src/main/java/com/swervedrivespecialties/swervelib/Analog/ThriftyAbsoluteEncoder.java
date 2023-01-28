package com.swervedrivespecialties.swervelib.Analog;


import com.revrobotics.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.ctre.CtreUtils;

import edu.wpi.first.wpilibj.AnalogEncoder;

public class ThriftyAbsoluteEncoder implements com.swervedrivespecialties.swervelib.AbsoluteEncoder {
    private final int ATTEMPTS = 3; // TODO: Allow changing number of tries for getting correct position

    private final AnalogEncoder analogEncoder;

    private ThriftyAbsoluteEncoder(AnalogEncoder analogEncoder) {
        this.analogEncoder = analogEncoder;
    }

    @Override
    public double getAbsoluteAngle() {
        double angle = analogEncoder.get();
        return angle;
    }

}
