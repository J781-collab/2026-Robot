package frc.util;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants.defaultSTD;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Vision {
    
    private final String deviceName;
    private final CommandSwerveDrivetrain driveSubsystem;

    public Vision(
        String kDeviceName,
        CommandSwerveDrivetrain kDriveSubsystem
    ) {
        deviceName = kDeviceName;
        driveSubsystem = kDriveSubsystem;
    }

    public Pose2d getEstimatedRoboPose() {
        return LimelightHelpers.getBotPose2d_wpiBlue(deviceName);
    }

    public Pose3d getRobotTagRelativePose() {
        return LimelightHelpers.getBotPose3d_TargetSpace(deviceName);
    }

    public Matrix<N3,N1> getStandardDeviations(){   //Borrowed all this from 3161
        LimelightHelpers.PoseEstimate limelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(deviceName);
        //Get tag count and average position...
        int tagCount = limelightPose.tagCount; //number of tags seen
        double avgDist = limelightPose.avgTagDist; //average distance across all tags seen

        //SmartDashboard.putNumber("Vision avgDist", avgDist);
        SmartDashboard.putNumber(deviceName + " tagCount", tagCount);


        if (tagCount == 1 && avgDist > 4) {
            return VecBuilder.fill(Double.MAX_VALUE,Double.MAX_VALUE,Double.MAX_VALUE);
        }
        else {
            SmartDashboard.putNumberArray("Vision STD", defaultSTD.singleTagStD.times(1 + (Math.pow(avgDist, 2) / 30)).getData());
            return defaultSTD.singleTagStD.times(1 + (Math.pow(avgDist, 2) / 30));
        }
    }
    public double getTimestamp() {
        return Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Capture(deviceName)/1000 - LimelightHelpers.getLatency_Pipeline(deviceName)/1000;
    }

    public boolean getTV() {
        return LimelightHelpers.getTV(deviceName);
    }

}