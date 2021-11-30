package frc.robot.swerve;

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
        return new Implementation(motor);
    }

    public static class Implementation implements DriveController {
        private final TalonSRX motor;

        public Implementation(TalonSRX motor) {
            this.motor = motor;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            motor.set(TalonSRXControlMode.PercentOutput, voltage / 12.0);
        }

        @Override
        public double getStateVelocity() {
            return motor.getSelectedSensorVelocity() / 36.65 * 0.0254;
        }
    }
}
