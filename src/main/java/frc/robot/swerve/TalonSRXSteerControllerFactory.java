package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SteerController;
import com.swervedrivespecialties.swervelib.SteerControllerFactory;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

public class TalonSRXSteerControllerFactory implements SteerControllerFactory<TalonSRXSteerControllerFactory.Implementation, TalonSRXSteerControllerConfiguration> {

    private static final double SENSOR_POSITION_COEFFICIENT = (2.0 * Math.PI) / 1024.0;

    @Override
    public void addDashboardEntries(ShuffleboardContainer container, Implementation controller) {
        SteerControllerFactory.super.addDashboardEntries(container, controller);

        container.addNumber("Current Steer Reading", controller::getRawReading);
    }

    @Override
    public TalonSRXSteerControllerFactory.Implementation create(TalonSRXSteerControllerConfiguration steerConfiguration, ModuleConfiguration moduleConfiguration) {
        TalonSRX motor = new TalonSRX(steerConfiguration.getMotorId());
        motor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 50);
        motor.configFeedbackNotContinuous(true, 50);
        motor.setSensorPhase(true);
        motor.config_kP(0, 30, 50);
        motor.config_kI(0, 0.001, 50);
        motor.config_kD(0, 200, 50);
        motor.config_kF(0, 0, 50);
        motor.setNeutralMode(NeutralMode.Brake);
        return new Implementation(steerConfiguration.getOffset(), motor);
    }

    public static class Implementation implements SteerController{
        private final double offset;
        private final TalonSRX motor;
        private double referenceAngle = 0;


        public Implementation(double offset, TalonSRX motor) {
            this.offset = offset;
            this.motor = motor;
        }

        @Override
        public double getReferenceAngle() {
            return referenceAngle;
        }

        @Override
        public void setReferenceAngle(double referenceAngleRadians) {
            double rawReferenceAngle = ((referenceAngleRadians - offset) / SENSOR_POSITION_COEFFICIENT) % 1024.0;
            if (rawReferenceAngle > 0){
                rawReferenceAngle -= 1024.0;
            }
            referenceAngle = Math.toRadians(rawReferenceAngle);
            motor.set(TalonSRXControlMode.Position, rawReferenceAngle);
        }

        @Override
        public double getStateAngle() {
            double angleRadians = (motor.getSelectedSensorPosition() * SENSOR_POSITION_COEFFICIENT + offset) % ((2 * Math.PI));
            if (angleRadians < 0) {
                angleRadians += (2 * Math.PI);
        }
            return angleRadians;
        }
        public double getRawReading(){
            return motor.getSelectedSensorPosition();
        }
    }
}
