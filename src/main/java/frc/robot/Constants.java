package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    
    public final static class ShootingConstants {
        /** Set to true for aim compensation (option 2), false for simple speed limiting (option 1). */
        public static boolean ENABLE_AIM_COMPENSATION = false;

        /** Drive speed multiplier while shooting (option 1 fallback). 0.3 = 30% speed. */
        public static double SHOOT_SPEED_MULTIPLIER = 0.3;

        /** Estimated ball exit speed in meters per second.
         *  Calculated from 4" wheels (circumference = π × 0.1016m = 0.3192m)
         *  and mid-range actual RPS (~78 RPS after -1.56 offset):
         *  78.44 × 0.3192 ≈ 25.0 m/s */
        public static double BALL_EXIT_SPEED_MPS = 25.0;

        /** Depth offset (meters) from the AprilTag wall to the center of the hub. 
         *  The aim point is shifted this far behind the tag surface.
         *  Based on field data: hub is ~1.2m deep (front wall x≈11.31, back wall x≈12.52),
         *  so center is ~0.6m from each wall. */
        public static double HUB_DEPTH_OFFSET = 0.6;

        /** Pass shot targets — field-relative (X, Y) coordinates in meters.
         *  Robot picks left vs right based on which side of the field it's on.
         *  Blue alliance: shoot to our own side for later use.
         *  Red alliance: mirrored targets on the red side. */
    
 public static Translation2d BLUE_PASS_LEFT = new Translation2d(1.793, 2.039);
        public static Translation2d BLUE_PASS_RIGHT  = new Translation2d(1.793, 6.149);
        public static Translation2d RED_PASS_LEFT  = new Translation2d(14.693, 6.149);
        public static Translation2d RED_PASS_RIGHT   = new Translation2d(14.53, 2.039);

        

        /** Y-coordinate of the field centerline for left/right detection. */
        public static double FIELD_CENTER_Y = 4.1;

        /** Max shooter speed (RPS) for pass shots. */
        public static double PASS_SHOT_RPS = -100.0;
    }


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
            public static double P = 0.01;
            public static double I = 0.0;
            public static double D = 0.00;
        }

        // Soft limits (degrees)
        public static double maxPivotPos = 145.0; 
        public static double minPivotPos = 0.0;

        // Preset positions (degrees)
        public static double idlePosition = 0.0;
        public static double intakePosition = 145.0;

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