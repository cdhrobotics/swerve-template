package com.swervedrivespecialties.swervelib.analog;

public class AnalogAbsoluteConfiguration {
    private final int id;
    private final double offset;

    public AnalogAbsoluteConfiguration(int id, double offset) {
        this.id = id;
        this.offset = offset;
    }

    public int getId() {
        return id;
    }

    public double getOffset() {
        return offset;
    }
}
