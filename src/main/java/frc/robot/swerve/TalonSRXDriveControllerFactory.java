package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.DriveControllerFactory;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;

public class TalonSRXDriveControllerFactory implements DriveControllerFactory<TalonSRXDriveControllerFactory.Implementation, Integer> {

    @Override
    public Implementation create(Integer id, ModuleConfiguration moduleConfiguration) {
        TalonSRX motor = new TalonSRX(id);
        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.voltageCompSaturation = 12;
        motor.configAllSettings(config);
        motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 50);
        motor.enableVoltageCompensation(true);
        motor.setNeutralMode(NeutralMode.Brake);
        double sensorPositionCoefficient = ((1.0/80.0) * moduleConfiguration.getDriveReduction()) * (moduleConfiguration.getWheelDiameter() * Math.PI);
        double sensorVelocityCoeffecient = sensorPositionCoefficient * 10.0;
        return new Implementation(motor, sensorPositionCoefficient, sensorVelocityCoeffecient);
    }

    public static class Implementation implements DriveController {
        private final TalonSRX motor;
        private final double sensorPositionCoefficient;
        private final double sensorVelocityCoeffecient;

        public Implementation(TalonSRX motor, double sensorPositionCoefficient, double sensorVelocityCoeffecient) {
            this.motor = motor;
            this.sensorPositionCoefficient = sensorPositionCoefficient;
            this.sensorVelocityCoeffecient = sensorVelocityCoeffecient;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            motor.set(TalonSRXControlMode.PercentOutput, voltage / 12.0);
        }

        @Override
        public double getStateVelocity() {
            return motor.getSelectedSensorVelocity() * sensorVelocityCoeffecient;
        }
    }
}
