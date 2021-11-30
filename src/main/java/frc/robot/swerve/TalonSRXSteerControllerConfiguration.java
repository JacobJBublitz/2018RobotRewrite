package frc.robot.swerve;

public class TalonSRXSteerControllerConfiguration {
    private final int motorId;
    private final double offset;

    public TalonSRXSteerControllerConfiguration(int motorId, double offset) {
        this.motorId = motorId;
        this.offset = offset;
    }

    public int getMotorId() {
        return motorId;
    }

    public double getOffset() {
        return offset;
    }

}
