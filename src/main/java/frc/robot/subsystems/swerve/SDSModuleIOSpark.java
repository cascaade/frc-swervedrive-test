package frc.robot.subsystems.swerve;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SDSModuleIOSpark implements SDSModuleIO {
    /* Motors */
    private SparkMax turnMotor;
    private SparkMax driveMotor;

    /* Motor encoders */
    private SparkAbsoluteEncoder turnEncoder;
    private RelativeEncoder driveEncoder;

    /* PID & Feedforward Controllers */
    private SparkClosedLoopController turnController;
    private SparkClosedLoopController driveController;

    /* Set the offset (zero) position of the turn motor, usually used to account for encoder differences */
    private Rotation2d zeroRotation;

    public SDSModuleIOSpark(int module) {
        /* Init motors based on module index */
        turnMotor = new SparkMax(
            switch(module) {
                case 0 -> SwerveConstants.kFLTurnCANID;
                case 1 -> SwerveConstants.kFRTurnCANID;
                case 2 -> SwerveConstants.kBLTurnCANID;
                case 3 -> SwerveConstants.kBRTurnCANID;
                default -> 0;
            },
            MotorType.kBrushless
        );
        driveMotor = new SparkMax(
            switch(module) {
                case 0 -> SwerveConstants.kFLDriveCANID;
                case 1 -> SwerveConstants.kFRDriveCANID;
                case 2 -> SwerveConstants.kBLDriveCANID;
                case 3 -> SwerveConstants.kBRDriveCANID;
                default -> 0;
            },
            MotorType.kBrushless
        );
        zeroRotation = switch(module) {
            case 0 -> SwerveModuleConstants.FLZeroRotation;
            case 1 -> SwerveModuleConstants.FRZeroRotation;
            case 2 -> SwerveModuleConstants.BLZeroRotation;
            case 3 -> SwerveModuleConstants.BRZeroRotation;
            default -> new Rotation2d();
        };

        /* Define the encoders */
        turnEncoder = turnMotor.getAbsoluteEncoder();
        driveEncoder = driveMotor.getEncoder();
        
        /* Sets the timeout for recieving data back from the CAN motor */
        turnMotor.setCANTimeout(0);
        driveMotor.setCANTimeout(0);
        
        /* Gets the PID controllers */
        turnController = turnMotor.getClosedLoopController();
        driveController = driveMotor.getClosedLoopController();

        /* Apply constant configurations to the motors */
        SparkMaxConfig turnConfig = SwerveModuleConstants.turnConfig;
        SparkMaxConfig driveConfig = SwerveModuleConstants.driveConfig;
        
        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(SDSModuleIOInputs inputs) {
        inputs.turnPosition = new Rotation2d(turnEncoder.getPosition()).minus(zeroRotation);
        inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
        inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
        inputs.turnCurrentAmps = turnMotor.getOutputCurrent();

        inputs.drivePositionRad = driveEncoder.getPosition();
        inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
        inputs.driveVelocityWheelMetersPerSec = inputs.driveVelocityRadPerSec * SwerveConstants.kWheelRadiusMeters;
        inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();
    }

    @Override // TODO remove
    public void updateInputs(SDSModuleIOInputs inputs, Rotation2d turnPositionFallback) {
        inputs.turnPosition = turnPositionFallback;
        inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
        inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
        inputs.turnCurrentAmps = turnMotor.getOutputCurrent();

        inputs.drivePositionRad = driveEncoder.getPosition();
        inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
        inputs.driveVelocityWheelMetersPerSec = inputs.driveVelocityRadPerSec * SwerveConstants.kWheelRadiusMeters;
        inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();
    }

    @Override
    public void setTurnPosition(Rotation2d position) {
        double setPoint = MathUtil.inputModulus(position.rotateBy(zeroRotation).getRadians(), 0, 2*Math.PI);
        turnController.setReference(setPoint, ControlType.kPosition);
    }

    @Override
    public void setDriveVelocityRadPerSec(double velocityRadPerSec) {
        double ffVolts = Math.signum(velocityRadPerSec) * SwerveModuleConstants.driveControlConstants.kS();
        driveController.setReference(velocityRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts, ArbFFUnits.kVoltage);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnMotor.setVoltage(output);
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveMotor.setVoltage(output);
    }

    @Override
    public void updateControlConstants() {
        SparkMaxConfig turnConfig = new SparkMaxConfig();
        SparkMaxConfig driveConfig = new SparkMaxConfig();

        turnConfig.closedLoop.pid(
            SwerveModuleConstants.turnControlConstants.kP(),
            SwerveModuleConstants.turnControlConstants.kI(),
            SwerveModuleConstants.turnControlConstants.kD()
        );
        driveConfig.closedLoop.pid(
            SwerveModuleConstants.driveControlConstants.kP(),
            SwerveModuleConstants.driveControlConstants.kI(),
            SwerveModuleConstants.driveControlConstants.kD()
        ).velocityFF(SwerveModuleConstants.driveControlConstants.kV());

        turnMotor.configure(turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
