package frc.robot.SwerveUtils;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.Constants;

public class SwerveModule {
    private final int moduleId;
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;
    private final CANcoder absEncoder;
    private final SwerveModuleConstants swerveModuleConstants;
    private final PIDController turnPIDController;
    private final StatusSignal<Double> absolutePositionSignal;
    private Rotation2d lastAngle = new Rotation2d(0);
    public SwerveModule(SwerveModuleConstants swerveModuleConstants){
        driveMotor = new CANSparkMax(swerveModuleConstants.driveMotorId, MotorType.kBrushless);
        //driveMotor.restoreFactoryDefaults();
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setInverted(swerveModuleConstants.driveMotorInverted);
        driveMotor.setSmartCurrentLimit(20);

        turnMotor = new CANSparkMax(swerveModuleConstants.turnMotorId, MotorType.kBrushless);
        //turnMotor.restoreFactoryDefaults();
        turnMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setInverted(true);
        turnMotor.setSmartCurrentLimit(20);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(Constants.PhysicalConstants.kDriveRotationsToMeters);
        driveEncoder.setVelocityConversionFactor(Constants.PhysicalConstants.kDriveRPMToMetersPerSec);
        turnEncoder.setPositionConversionFactor(Constants.PhysicalConstants.kTurnRotationsToAngelDeg);
        turnEncoder.setVelocityConversionFactor(Constants.PhysicalConstants.kTurnRPMToDegPerSec);

        absEncoder = new CANcoder(swerveModuleConstants.absoluteEncoderId); //new AnalogPotentiometer(swerveModuleConstants.absoluteEncoderId, 360, swerveModuleConstants.absoluteEncoderOffset);
        absolutePositionSignal = absEncoder.getAbsolutePosition();
        //absEncoder.getConfigurator().apply(new MagnetSensorConfigs().)
        turnPIDController = new PIDController(0.001, 0, 0);
        turnPIDController.enableContinuousInput(-180, 180);

        moduleId = swerveModuleConstants.moduleId;
        this.swerveModuleConstants = swerveModuleConstants;

        resetEncoders();
    }

    public double getAbsAngleDeg(){
        return absolutePositionSignal.refresh().getValue()*360;
    }

    public double getAngleDeg(){
        return Math.IEEEremainder(turnEncoder.getPosition(), 360);
    }

    public Rotation2d getAngleRotation2d(){
        return Rotation2d.fromDegrees(getAbsAngleDeg());
    }

    public double getDriveSpeedMetersPerSec(){
        return driveEncoder.getVelocity();
    }

    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsAngleDeg());
    }

    public SwerveModuleState getSwerveModuleState(){
        return new SwerveModuleState(getDriveSpeedMetersPerSec(), getAngleRotation2d());
    }

    public void stopMotors(){
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public void setSwerveModuleState(SwerveModuleState desiredState){
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.1){
            desiredState.angle = lastAngle;
        }
        desiredState = SwerveModuleState.optimize(desiredState, getAngleRotation2d());
        driveMotor.set(desiredState.speedMetersPerSecond/Constants.PhysicalConstants.kPhysicalMaxSpeedMetersPerSec);
        turnMotor.set(turnPIDController.calculate(getAbsAngleDeg(), desiredState.angle.getDegrees()));
        lastAngle = desiredState.angle;
    }

}
