package frc.robot.subsystems.swerve;

import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {
    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroIOInputs;
    private SDSSwerveModule[] modules;
    private Rotation2d rawGyroRotation;

    private final Alert gyroDisconnectedAlert = new Alert(
        "Disconnected gyro, using kinematics as fallback.",
        Alert.AlertType.kError
    );

    private SwerveModulePosition[] modulePositions;
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;

    private double personalSpeedFactor;

    public SwerveDrive(GyroIO gyro, SDSModuleIO FLModuleIO, SDSModuleIO FRModuleIO, SDSModuleIO BLModuleIO, SDSModuleIO BRModuleIO) {
        initComponents(gyro, FLModuleIO, FRModuleIO, BLModuleIO, BRModuleIO);
        initMathModels();
    }

    private void initComponents(GyroIO gyro, SDSModuleIO FLModuleIO, SDSModuleIO FRModuleIO, SDSModuleIO BLModuleIO, SDSModuleIO BRModuleIO) {
        this.gyroIO = gyro;
        modules = new SDSSwerveModule[] {
            new SDSSwerveModule("0 Front Left", FLModuleIO, 0),
            new SDSSwerveModule("1 Front Right", FLModuleIO, 1),
            new SDSSwerveModule("2 Back Left", FLModuleIO,2),
            new SDSSwerveModule("3 Back Right", FLModuleIO,3),
        };
        gyroIOInputs = new GyroIOInputsAutoLogged();
    }

    private void initMathModels() {
        personalSpeedFactor = 1;
        
        rawGyroRotation = new Rotation2d();
        modulePositions = Arrays.stream(modules).map(module -> module.getPosition()).toArray(SwerveModulePosition[]::new);
        kinematics = new SwerveDriveKinematics(
            new Translation2d(SwerveConstants.kWheelDistanceMeters / 2, SwerveConstants.kWheelDistanceMeters / 2),  // FL
            new Translation2d(SwerveConstants.kWheelDistanceMeters / 2, -SwerveConstants.kWheelDistanceMeters / 2), // FR
            new Translation2d(-SwerveConstants.kWheelDistanceMeters / 2, SwerveConstants.kWheelDistanceMeters / 2), // BL
            new Translation2d(-SwerveConstants.kWheelDistanceMeters / 2, -SwerveConstants.kWheelDistanceMeters / 2) // BR
        );
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, modulePositions, new Pose2d());
    }

    // TODO Check with andy
    private double adjustAxisInput(double controllerInput, double deadband, double minThreshold, double steepness) {
        return Math.abs(controllerInput) > deadband ? (
            Math.signum(controllerInput) * ((Math.abs(controllerInput) - deadband) / (1 - deadband))
        ) : 0;
    }

    /**
     * 
     * @param xInput
     * @param yInput
     * @param omegaInput
     * @param robotCentric
     * @param noOptimize
     * @return
     */
    public Command runDriveInputs(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier omegaInput, BooleanSupplier robotCentric, BooleanSupplier noOptimize) {
        return run(() -> {
            double deadband = 0.2; // min axis input (account for stickdrift)
            double minThreshold = 0.1; // min robot input (account for friction resistance)
            double steepness = 1.8; // Power to raise the input to account for deadband

            SmartDashboard.putNumber("Axes/xInput", -yInput.getAsDouble());
            SmartDashboard.putNumber("Axes/yInput", -xInput.getAsDouble());
            SmartDashboard.putNumber("Axes/omegaInput", -omegaInput.getAsDouble());

            double vx = adjustAxisInput(-yInput.getAsDouble(), deadband, minThreshold, steepness);
            double vy = adjustAxisInput(-xInput.getAsDouble(), deadband, minThreshold, steepness);
            double omega = adjustAxisInput(-omegaInput.getAsDouble(), deadband, minThreshold, steepness); // omega is the turn (angular velocity)

            SmartDashboard.putNumber("Swerve/vx", vx);
            SmartDashboard.putNumber("Swerve/vy", vy);
            SmartDashboard.putNumber("Swerve/omega", omega);

            adjustDriveSpeeds(vx, vy, omega, !robotCentric.getAsBoolean(), !noOptimize.getAsBoolean());
        });
    }

    public void adjustDriveSpeeds(double vx, double vy, double omega, boolean fieldRelative, boolean optimize) {
        double mag = Math.hypot(vx, vy); // Magnitude of the chassis linear velocity
        double dir = Math.atan2(vy, vx); // Angle in radians of the chassis angular velocity

        mag *= SwerveConstants.kMagVelLimit * personalSpeedFactor;
        mag /= Math.min(
            Math.sqrt(1 + Math.pow(Math.sin(dir), 2)),
            Math.sqrt(1 + Math.pow(Math.cos(dir), 2))
        );
        omega *= SwerveConstants.kRotVelLimit;

        SmartDashboard.putNumber("Swerve/mag", mag);
        SmartDashboard.putNumber("Swerve/dir", dir);

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(mag * Math.cos(dir), mag * Math.sin(dir), omega);
        runChassisSpeeds(chassisSpeeds, fieldRelative, optimize);
    }

    public void runChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean fieldRelative, boolean optimize) {
        ChassisSpeeds adjustedSpeeds = chassisSpeeds;

        if (fieldRelative) {
            adjustedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, rawGyroRotation); // TODO changed for simplicity of code
        }

        adjustedSpeeds = ChassisSpeeds.discretize(adjustedSpeeds, LoggedRobot.defaultPeriodSecs);
        SwerveModuleState[] moduleSetpoints = kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleSetpoints, SwerveConstants.kMaxWheelSpeed);

        Logger.recordOutput("Swerve/States/Setpoints", moduleSetpoints);
        Logger.recordOutput("Swerve/ChassisSpeeds/Setpoints", adjustedSpeeds);

        setRawModuleSetpoints(moduleSetpoints, optimize);

        if (optimize) Logger.recordOutput("Swerve/States/SetpointsOptimized", moduleSetpoints);
    }

    public void setRawModuleSetpoints(SwerveModuleState[] states, boolean optimize) {
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(states[i], optimize);
        }
    }

    public Command runTestDrive() {
        return runOnce(() -> {
            SwerveModuleState testSwerveState = new SwerveModuleState(
                Preferences.getDouble("kSwerveTestDrive", SwerveConstants.kDefaultTestDrive),
                new Rotation2d(Preferences.getDouble("kSwerveTestTurn", SwerveConstants.kDefaultTestTurn))
            );

            modules[0].setDesiredState(testSwerveState);
            modules[1].setDesiredState(testSwerveState);
            modules[2].setDesiredState(testSwerveState);
            modules[3].setDesiredState(testSwerveState);
        });
    }

    public Command runStopDrive() {
        return runOnce(() -> {
            for (SDSSwerveModule module : modules) {
                module.stopDrive();
            }
        });
    }

    public Command runUpdateControlConstants() {
        return runOnce(() -> {
            for (SDSSwerveModule module : modules) {
                module.updateControlConstants();
            }
        });
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroIOInputs);
        Logger.processInputs("Swerve/Gryo", gyroIOInputs);

        for (SDSSwerveModule module : modules) {
            module.periodic();
        }

        if (DriverStation.isDisabled()) {
            for (SDSSwerveModule module : modules) {
                module.stopDrive();
            }

            Logger.recordOutput("Swerve/States/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("Swerve/States/SetpointsOptimized", new SwerveModuleState[] {});
        }

        SwerveModulePosition[] updatedModulePositions = new SwerveModulePosition[4];
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

        for (int i = 0; i < 4; i++) {
            updatedModulePositions[i] = modules[i].getPosition();
            moduleDeltas[i] = new SwerveModulePosition(
                updatedModulePositions[i].distanceMeters - modulePositions[i].distanceMeters,
                updatedModulePositions[i].angle.minus(modulePositions[i].angle)
            );
        }

        if (gyroIOInputs.connected) {
            rawGyroRotation = gyroIOInputs.yawPosition;
        } else {
            Twist2d twist = kinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }

        poseEstimator.update(rawGyroRotation, updatedModulePositions);
        gyroDisconnectedAlert.set(!gyroIOInputs.connected); // TODO update if not sim
    }
}
