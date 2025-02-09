package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.ControlConstants;

public class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kAuxiliaryControllerPort = 1;
    }

    public static class SwerveConstants {
        public static final double kWheelDistanceMeters = Units.inchesToMeters(20);

        public static final int kPigeonCANID = 0;

        public static final int kFLTurnCANID = 1;
        public static final int kFRTurnCANID = 2;
        public static final int kBLTurnCANID = 3;
        public static final int kBRTurnCANID = 4;

        public static final int kFLDriveCANID = 5;
        public static final int kFRDriveCANID = 6;
        public static final int kBLDriveCANID = 7;
        public static final int kBRDriveCANID = 8;

        public static final double kDriveGearRatio = 6.12 / 1; // rotor rotations per wheel rotations
		public static final double kInternalNEOEncoderCPR = 42 / 1; // counts on encoder counts per revolution
		public static final double kWheelRadiusMeters = Units.inchesToMeters(4 / 2); // meters per revolution (wheel circumference)
        public static final double kDrivePositionConversionFactor = Units.rotationsToRadians(1) / (kDriveGearRatio * kInternalNEOEncoderCPR);
		public static final double kDriveVelocityConversionFactor = Units.rotationsToRadians(1) / (kDriveGearRatio * kInternalNEOEncoderCPR);
		public static final double kTurnPositionConversionFactor = Units.rotationsToRadians(1);
		public static final double kTurnVelocityConversionFactor = Units.rotationsToRadians(1);

		public static final double kMaxWheelSpeed = 8; // m/s
		public static final double kMagVelLimit = 2.5; // m/s
		public static final double kDirVelLimit = 10; // rad/s
		public static final double kRotVelLimit = 2*Math.PI; // rad/s
		public static final double kMagAccelLimit = 48; // m/s^2
		public static final double kRotAccelLimit = 30; // rad/s^2

        public static final double kDefaultTestTurn = 0;
		public static final double kDefaultTestDrive = 0;
    }

    public static class SwerveModuleConstants {
        public static final SparkMaxConfig turnConfig = new SparkMaxConfig();
        public static final SparkMaxConfig driveConfig = new SparkMaxConfig();

        public static final Rotation2d FLZeroRotation = new Rotation2d();
        public static final Rotation2d FRZeroRotation = new Rotation2d();
        public static final Rotation2d BLZeroRotation = new Rotation2d();
        public static final Rotation2d BRZeroRotation = new Rotation2d();

        public static final ControlConstants turnControlConstants = new ControlConstants(
			"swerveModule/turn",
			0.3, // 0.3
			0, // 0, used 0.0001 in the past
			0,
			0,
            0,
            0
		);

		public static final ControlConstants driveControlConstants = new ControlConstants(
			"swerveModule/drive",
			0.00009, // 0.01
			0, // 0, used 0.0001 in the past
			0,
			0.0069, // 0.145
            0,
            0.11
		);

        static {
            turnConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .voltageCompensation(12)
			; turnConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .p(turnControlConstants.kP())
                .i(turnControlConstants.kI())
                .d(turnControlConstants.kD())
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI)
                .outputRange(-1, 1)
			; turnConfig.absoluteEncoder
				.positionConversionFactor(SwerveConstants.kTurnPositionConversionFactor)
				.velocityConversionFactor(SwerveConstants.kTurnVelocityConversionFactor)
			;

            driveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .voltageCompensation(12)
            ; driveConfig.closedLoop
                .p(driveControlConstants.kP())
                .i(driveControlConstants.kI())
                .d(driveControlConstants.kD())
				.velocityFF(driveControlConstants.kV())
                .outputRange(-1, 1)
            ; driveConfig.encoder
				.positionConversionFactor(SwerveConstants.kDrivePositionConversionFactor)
				.velocityConversionFactor(SwerveConstants.kDriveVelocityConversionFactor)
			;
			System.out.println("SwerveModuleConstants initialized");
        }
    }
}
