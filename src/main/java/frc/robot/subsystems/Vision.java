package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Estimates the robot's field pose from AprilTags seen by a PhotonVision camera
 * and hands each estimate to a consumer (normally the drivetrain's
 * addVisionMeasurement) so it gets fused with wheel odometry.
 */
public class Vision extends SubsystemBase {
    /** Receives each accepted vision pose estimate. */
    @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }

    /* Must match the camera name shown in the PhotonVision web UI */
    private static final String kCameraName = "limelight";

    /*
     * Where the camera is on the robot, measured from the robot's center at floor level.
     * Translation: +X forward, +Y left, +Z up.
     * Rotation: roll, pitch, yaw. A camera tilted UP has a NEGATIVE pitch.
     * TODO: measure these on the real robot -- pose estimates will be wrong until you do.
     */
    private static final Transform3d kRobotToCamera = new Transform3d(
        new Translation3d(Inches.of(0), Inches.of(0), Inches.of(0)),
        new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0))
    );

    /* The 2026 field layout; use k2026RebuiltAndymark if your event uses AndyMark fields */
    private static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    /* Standard deviations [x, y, theta] -- bigger means "trust vision less" */
    private static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    private static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    /* Ignore single-tag estimates where the tag is farther than this */
    private static final double kMaxSingleTagDistanceMeters = 4.0;
    /* Ignore single-tag estimates with an ambiguity above this (0 = certain, 1 = unusable) */
    private static final double kMaxAmbiguity = 0.2;

    private final PhotonCamera m_camera = new PhotonCamera(kCameraName);
    private final PhotonPoseEstimator m_poseEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCamera);
    private final VisionConsumer m_visionConsumer;

    private final Field2d m_field = new Field2d();

    /**
     * @param visionConsumer Called with each accepted pose estimate,
     *                       e.g. {@code drivetrain::addVisionMeasurement}
     */
    public Vision(VisionConsumer visionConsumer) {
        m_visionConsumer = visionConsumer;
        SmartDashboard.putData("Vision/Field", m_field);
    }

    @Override
    public void periodic() {
        boolean hasEstimate = false;

        for (PhotonPipelineResult result : m_camera.getAllUnreadResults()) {
            /* Prefer the multi-tag estimate solved on the coprocessor, fall back to the best single tag */
            Optional<EstimatedRobotPose> estimate = m_poseEstimator.estimateCoprocMultiTagPose(result);
            if (estimate.isEmpty()) {
                estimate = m_poseEstimator.estimateLowestAmbiguityPose(result);
            }
            if (estimate.isEmpty()) {
                continue;
            }

            EstimatedRobotPose est = estimate.get();
            Optional<Matrix<N3, N1>> stdDevs = getEstimationStdDevs(est);
            if (stdDevs.isEmpty()) {
                continue;
            }

            Pose2d pose = est.estimatedPose.toPose2d();
            m_visionConsumer.accept(pose, est.timestampSeconds, stdDevs.get());
            m_field.setRobotPose(pose);
            hasEstimate = true;
        }

        SmartDashboard.putBoolean("Vision/Connected", m_camera.isConnected());
        SmartDashboard.putBoolean("Vision/HasEstimate", hasEstimate);
    }

    /**
     * Picks how much to trust an estimate based on how many tags were used and how far
     * away they were, or rejects it entirely.
     *
     * @return Standard deviations to use, or empty to throw the estimate away
     */
    private Optional<Matrix<N3, N1>> getEstimationStdDevs(EstimatedRobotPose est) {
        Pose2d pose = est.estimatedPose.toPose2d();

        /* Throw out poses that are off the field */
        if (pose.getX() < 0 || pose.getX() > kTagLayout.getFieldLength()
            || pose.getY() < 0 || pose.getY() > kTagLayout.getFieldWidth()) {
            return Optional.empty();
        }

        int numTags = 0;
        double totalDistance = 0;
        for (PhotonTrackedTarget target : est.targetsUsed) {
            var tagPose = kTagLayout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) {
                continue;
            }
            numTags++;
            totalDistance += tagPose.get().toPose2d().getTranslation().getDistance(pose.getTranslation());
        }
        if (numTags == 0) {
            return Optional.empty();
        }
        double avgDistance = totalDistance / numTags;

        if (numTags == 1) {
            /* A single tag can flip between two possible poses; only trust it when it's clear and close */
            PhotonTrackedTarget target = est.targetsUsed.get(0);
            if (target.getPoseAmbiguity() > kMaxAmbiguity || avgDistance > kMaxSingleTagDistanceMeters) {
                return Optional.empty();
            }
            return Optional.of(kSingleTagStdDevs.times(1 + avgDistance * avgDistance / 30));
        }

        /* Trust estimates less the farther away the tags are */
        return Optional.of(kMultiTagStdDevs.times(1 + avgDistance * avgDistance / 30));
    }
}
