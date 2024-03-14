package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveUtils.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
    /**
     * The Singleton instance of this SwerveSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private static SwerveSubsystem INSTANCE;

    /**
     * Returns the Singleton instance of this SwerveSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code SwerveSubsystem.getInstance();}
     */
    SwerveModule frontLeft = new SwerveModule(Constants.SwerveModuleConstantsInstances.kFrontLeftModule);
    SwerveModule frontRight = new SwerveModule(Constants.SwerveModuleConstantsInstances.kFrontRightModule);
    SwerveModule backLeft = new SwerveModule(Constants.SwerveModuleConstantsInstances.kBackLeftModule);
    SwerveModule backRight = new SwerveModule(Constants.SwerveModuleConstantsInstances.kBackRightModule);
    @SuppressWarnings("WeakerAccess")
    public static SwerveSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new SwerveSubsystem();
        }
        return INSTANCE;
    }

    /**
     * Creates a new instance of this SwerveSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private SwerveSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading(){
        gyro.reset();
        System.out.println("Gyro reset");
    }

    public double getHeading(){
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    public Rotation2d getHeadingRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public void stopSwerveModuleMotors(){
        frontLeft.stopMotors();
        frontRight.stopMotors();
        backLeft.stopMotors();
        backRight.stopMotors();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.PhysicalConstants.kPhysicalMaxSpeedMetersPerSec);
        frontLeft.setSwerveModuleState(desiredStates[1]);
        frontRight.setSwerveModuleState(desiredStates[0]);
        backLeft.setSwerveModuleState(desiredStates[3]);
        backRight.setSwerveModuleState(desiredStates[2]);
    }
}

