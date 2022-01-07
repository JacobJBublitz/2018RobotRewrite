package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.SysIdGeneralMechanismLogger;

public class CharacterizeDrivetrainCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final SysIdGeneralMechanismLogger logger = new SysIdGeneralMechanismLogger();

    public CharacterizeDrivetrainCommand(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setPosition(new Pose2d()); // Reset the drivetrain position

        logger.initLogging();
    }

    @Override
    public void execute() {
        // Update the data
        double position = drivetrain.getPosition().getX();
        double velocity = drivetrain.getVelocity().x;
        logger.log(position, velocity);

        // Get the voltage
        double voltage = logger.getMotorVoltage();

        // Drive at new voltage
        drivetrain.drive(
                new ChassisSpeeds(
                        voltage / RobotController.getBatteryVoltage() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                        0.0,
                        0.0
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        logger.sendData();
    }
}
