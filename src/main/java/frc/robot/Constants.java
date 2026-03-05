package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    


    public final static class VisionConstants {
        public static class defaultSTD {
            public static Vector<N3> singleTagStD = VecBuilder.fill(2, 2, 4);
        } 

        public static class AlignmentController {
            public static class StrafeXController {
                public static double P = 1.0;
                public static double I = 0.0;
                public static double D = 0.0;
            }
            public static class StrafeYController {
                public static double P = 1.0;
                public static double I = 0.0;
                public static double D = 0.0;
            }
            public static class RotationController {
                public static double P = 0.001;
                public static double I = 0.0;
                public static double D = 0.0;
            }
        }
    }
    public final static class QuestConstants {
        public Translation2d headsetRobotPose = new Translation2d(0,37);
    }

    public final static class ElevatorConstants {
        public static class ElevatorProfiledPID {
            public static double P = 0.05;
            public static double I = 0;
            public static double D = 1.5;
            public static double MaxVelocity = 0;
            public static double MaxAcceleration = 0;
        }

        public static double maxChassisHeight = 56.5; //inches
        public static double gearCircumference = 5.50093*2; //inches
        public static double ChassisElevationOffset = 1.25;
        public static double gearRatio = 1/9;
        public static int encoderID = 20;
        public static double encoderOffset = 0.0;
        public static int leaderMotorID = 21;
        public static int followMotorID = 22;
    }

    public final static class ArmConstants {
        public static class ArmProfiledPID {
            public static double P = 0.005;
            public static double I = 0.0;
            public static double D = 0.0009;
            public static double MaxVelocity = 0;
            public static double MaxAcceleration = 0;
        }

        // Encoder reading * 360 = degrees
        public static double maxPivotPos = 96;
        public static double minPivotPos = -0.1;

        // Idle and closed position
        public static double idlePosition = 0.0;

        // Open position
        public static double openPosition = 0.0;

        // Score position
        public static double scorePosition = 0.0;

        // Safe pivot position; the elevator can move
        public static double intakeSafePosition = 150;

        public static double intakeSpeed = 0.05;

        public static int encoderID = 34;
        public static double encoderOffset = 0.931;
        public static int leaderMotorID = 31;
        public static int followMotorID = 32;

    }

    public final class EndAffectorConstants {
        public static int affectorMotorID = 33;
        public static int beamBreakPort = 0;
        public static int rangeSensorPort = 1;
    }

    public final static class IntakePivotConstants {
        public static class IntakePivotPID {
            public static double P = 0.005;
            public static double I = 0.0;
            public static double D = 0.0009;
        }

        // Soft limits (degrees)
        public static double maxPivotPos = 100;
        public static double minPivotPos = -60;

        // Preset positions (degrees)
        public static double idlePosition = -45.0;
        public static double intakePosition = 90.0;

        public static int encoderID = 35;
        public static double encoderOffset = 0.0;
        public static int leaderMotorID = 20;
        public static int followMotorID = 0; // Set to 0 if no follower
    }

    public final class ClimberConstants {
        public static int leaderMotorID = 51;
        public static int followMotorID = 52;
    }
}