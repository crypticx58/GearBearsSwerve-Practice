package frc.robot.subsystems;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils.InputsManager;

public class OdometrySubsystem extends SubsystemBase {
    //private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private static OdometrySubsystem INSTANCE;
    @SuppressWarnings("WeakerAccess")
    public static OdometrySubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new OdometrySubsystem();
        }
        return INSTANCE;
    }
    private final Field2d fieldVisualizer2d = new Field2d();
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.PhysicalConstants.kDriveKinematics, new Rotation2d(0), swerveSubsystem.getModulePositions(), new Pose2d());
    
    public void resetOdometry(Pose2d newPose){
        swerveDrivePoseEstimator.resetPosition(swerveSubsystem.getHeadingRotation2d(), swerveSubsystem.getModulePositions(), newPose);
    }
    public Pose2d getPose(){
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }
    private OdometrySubsystem() {
        AutoBuilder.configureHolonomic(this::getPose, this::resetOdometry, swerveSubsystem::getRobotRelativeSpeeds, InputsManager.SwerveInputsManager::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(2.75),
                        new PIDConstants(2.75),
                        4.5,
                        Units.inchesToMeters(20),
                        new ReplanningConfig()),
                () -> false,
                this);
        SmartDashboard.putData("Field", fieldVisualizer2d);
    }
    @Override
    public void periodic() {
        super.periodic();
        swerveDrivePoseEstimator.update(swerveSubsystem.getHeadingRotation2d(), swerveSubsystem.getModulePositions());
        fieldVisualizer2d.setRobotPose(getPose());
    }

}

