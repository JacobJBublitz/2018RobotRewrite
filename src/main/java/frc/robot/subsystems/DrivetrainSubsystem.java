package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.SwerveModuleFactory;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.swerve.TalonSRXDriveControllerFactory;
import frc.robot.swerve.TalonSRXSteerControllerConfiguration;
import frc.robot.swerve.TalonSRXSteerControllerFactory;

public class DrivetrainSubsystem extends SubsystemBase {
    public static final double MAX_VOLTAGE = 12.0;
    public static final ModuleConfiguration MK1_CONFIGURATION = new ModuleConfiguration(
            0.1016,
            (18.0 / 26.0) * (15.0/ 60.0),
            false,
            1.0,
            false
    );
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5330.0 / 60.0 *
            MK1_CONFIGURATION.getDriveReduction() *
            MK1_CONFIGURATION.getWheelDiameter() * Math.PI;
    public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final double VELOCITY_CONSTANT = MAX_VOLTAGE / MAX_VELOCITY_METERS_PER_SECOND;
    public static final double ACCELERATION_CONSTANT = MAX_VOLTAGE / 6.858;

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

    private final SwerveDriveOdometry odometry;

    private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);

    private ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();
    private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();

    public DrivetrainSubsystem() {
        var moduleFactory = new SwerveModuleFactory<>(
                MK1_CONFIGURATION,
                new TalonSRXDriveControllerFactory(),
                new TalonSRXSteerControllerFactory()
        );
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
        frontLeftModule = moduleFactory.create(shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0), Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                new TalonSRXSteerControllerConfiguration(Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
                        Constants.FRONT_LEFT_MODULE_STEER_OFFSET));
        frontRightModule = moduleFactory.create(shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0), Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                new TalonSRXSteerControllerConfiguration(Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                        Constants.FRONT_RIGHT_MODULE_STEER_OFFSET));
        backLeftModule = moduleFactory.create(shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0), Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                new TalonSRXSteerControllerConfiguration(Constants.BACK_LEFT_MODULE_STEER_MOTOR,
                        Constants.BACK_LEFT_MODULE_STEER_OFFSET));
        backRightModule = moduleFactory.create(shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0), Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                new TalonSRXSteerControllerConfiguration(Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
                        Constants.BACK_RIGHT_MODULE_STEER_OFFSET));
        odometry = new SwerveDriveOdometry(kinematics, getGyroscopeRotation());
        shuffleboardTab.addNumber("Drive Rotation Velocity", ()->Math.toDegrees(targetChassisSpeeds.omegaRadiansPerSecond));
        shuffleboardTab.addNumber("Max Angular Velocity", ()-> Math.toDegrees(MAX_ANGULAR_VELOCITY));
        shuffleboardTab.addNumber("Drivetrain Angle", () -> getPosition().getRotation().getDegrees());
        shuffleboardTab.addNumber("PositionX", ()->odometry.getPoseMeters().getX());
        shuffleboardTab.addNumber("PositionY", ()->odometry.getPoseMeters().getY());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.targetChassisSpeeds = chassisSpeeds;
    }

    public void zeroRotation(){
        odometry.resetPosition(new Pose2d(getPosition().getX(), getPosition().getY(), new Rotation2d()), getGyroscopeRotation());
    }

    public Rotation2d getGyroscopeRotation(){
        if (m_navx.isMagnetometerCalibrated()) {
            return Rotation2d.fromDegrees(m_navx.getFusedHeading());
        }
        return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
    }

    public Pose2d getPosition(){
        return odometry.getPoseMeters();
    }

    public void setPosition(Pose2d pose) {
        odometry.resetPosition(pose, getGyroscopeRotation());
    }

    public void setPosition(Vector2d position) {
        odometry.resetPosition(
                new Pose2d(position.x, position.y, getPosition().getRotation()),
                getGyroscopeRotation()
        );
    }

    public Vector2d getVelocity() {
        return new Vector2d(
                currentChassisSpeeds.vxMetersPerSecond,
                currentChassisSpeeds.vyMetersPerSecond
        );
    }

    public Command createTrajectoryFollowerCommand (Trajectory trajectory){
        PIDController xController = new PIDController(25,0,2);
        PIDController yController = new PIDController(25,0,2);
        ProfiledPIDController thetaController = new ProfiledPIDController(5,0,0, new TrapezoidProfile.Constraints(1,1));

        return new SwerveControllerCommand(
                trajectory,
                this::getPosition,
                kinematics,
                xController,
                yController,
                thetaController,
                states -> targetChassisSpeeds = kinematics.toChassisSpeeds(states),
                this
        );
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetChassisSpeeds);
        SwerveDriveKinematics.normalizeWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        SwerveModuleState currentFrontLeftModuleState = new SwerveModuleState(frontLeftModule.getDriveVelocity(), new Rotation2d(frontLeftModule.getSteerAngle()));
        SwerveModuleState currentFrontRightModuleState = new SwerveModuleState(frontRightModule.getDriveVelocity(), new Rotation2d(frontRightModule.getSteerAngle()));
        SwerveModuleState currentBackLeftModuleState = new SwerveModuleState(backLeftModule.getDriveVelocity(), new Rotation2d(backLeftModule.getSteerAngle()));
        SwerveModuleState currentBackRightModuleState = new SwerveModuleState(backRightModule.getDriveVelocity(), new Rotation2d(backRightModule.getSteerAngle()));

        Rotation2d gyroAngle = getGyroscopeRotation();
        currentChassisSpeeds = kinematics.toChassisSpeeds(currentFrontLeftModuleState, currentFrontRightModuleState, currentBackLeftModuleState, currentBackRightModuleState);
        odometry.update(gyroAngle, currentFrontLeftModuleState, currentFrontRightModuleState, currentBackLeftModuleState, currentBackRightModuleState);

        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }
}
