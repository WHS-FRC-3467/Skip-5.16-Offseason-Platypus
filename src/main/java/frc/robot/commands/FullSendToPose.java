// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.PID;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;

import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * Command to autonomously drive the robot to a target pose on the field.
 *
 * <p>Uses profiled PID controllers for both linear (x, y) and angular (rotation) motion. The
 * controllers have tunable gains that can be adjusted through NetworkTables for optimization.
 * Maximum velocities are also tunable.
 */
public class FullSendToPose extends Command {
    private static final double LOOP_PERIOD_SECONDS = 0.02;
    private static final double LINEAR_VELOCITY_TOLERANCE_METERS_PER_SECOND = 1e-2;
    private static final double ANGULAR_VELOCITY_TOLERANCE_RADIANS_PER_SECOND = 1e-2;

    private static final PID DEFAULT_LINEAR_PID = new PID(8.0, 0.0, 0.0);
    private static final TrapezoidProfile.Constraints DEFAULT_LINEAR_CONSTRAINTS =
            new TrapezoidProfile.Constraints(6.0, 12.0);

    private static final PID DEFAULT_ANGULAR_PID = new PID(8.0, 0.0, 0.1);
    private static final TrapezoidProfile.Constraints DEFAULT_ANGULAR_CONSTRAINTS =
            new TrapezoidProfile.Constraints(8.3, 20.0);

    private static final LoggedTunableNumber LINEAR_P =
            new LoggedTunableNumber("FullSendToPose/Linear/P", DEFAULT_LINEAR_PID.P());
    private static final LoggedTunableNumber LINEAR_I =
            new LoggedTunableNumber("FullSendToPose/Linear/I", DEFAULT_LINEAR_PID.I());
    private static final LoggedTunableNumber LINEAR_D =
            new LoggedTunableNumber("FullSendToPose/Linear/D", DEFAULT_LINEAR_PID.D());
    private static final LoggedTunableNumber LINEAR_V =
            new LoggedTunableNumber(
                    "FullSendToPose/Linear/MaxVelocity", DEFAULT_LINEAR_CONSTRAINTS.maxVelocity);
    private static final LoggedTunableNumber LINEAR_A =
            new LoggedTunableNumber(
                    "FullSendToPose/Linear/MaxAcceleration",
                    DEFAULT_LINEAR_CONSTRAINTS.maxAcceleration);

    private static final LoggedTunableNumber ANGULAR_P =
            new LoggedTunableNumber("FullSendToPose/Angular/P", DEFAULT_ANGULAR_PID.P());
    private static final LoggedTunableNumber ANGULAR_I =
            new LoggedTunableNumber("FullSendToPose/Angular/I", DEFAULT_ANGULAR_PID.I());
    private static final LoggedTunableNumber ANGULAR_D =
            new LoggedTunableNumber("FullSendToPose/Angular/D", DEFAULT_ANGULAR_PID.D());
    private static final LoggedTunableNumber ANGULAR_V =
            new LoggedTunableNumber(
                    "FullSendToPose/Angular/MaxVelocity", DEFAULT_ANGULAR_CONSTRAINTS.maxVelocity);
    private static final LoggedTunableNumber ANGULAR_A =
            new LoggedTunableNumber(
                    "FullSendToPose/Angular/MaxAcceleration",
                    DEFAULT_ANGULAR_CONSTRAINTS.maxAcceleration);

    private static final PIDController linearController =
            new PIDController(
                    DEFAULT_LINEAR_PID.P(), DEFAULT_LINEAR_PID.I(), DEFAULT_LINEAR_PID.D());
    private TrapezoidProfile linearProfile = new TrapezoidProfile(DEFAULT_LINEAR_CONSTRAINTS);

    // Current linear goal of the profile, should be null if no goal.
    private TrapezoidProfile.State linearProfileGoal = null;
    // Current linear setpoint of the profile.
    // When a new final goal is commanded, this should be set
    // to the current state of the robot. During the periodic
    // loop of an existing goal, this should be set to the output
    // of the profile. This should only really be null if there
    // is no current state, so before any goal has ever been commanded.
    private TrapezoidProfile.State linearProfileState = null;

    private static final PIDController angularController =
            new PIDController(
                    DEFAULT_ANGULAR_PID.P(), DEFAULT_ANGULAR_PID.I(), DEFAULT_ANGULAR_PID.D());
    private TrapezoidProfile angularProfile = new TrapezoidProfile(DEFAULT_ANGULAR_CONSTRAINTS);

    // Current angular goal of the profile, should be null if no goal.
    private TrapezoidProfile.State angularProfileGoal = null;
    // Current angular setpoint of the profile.
    // When a new final goal is commanded, this should be set
    // to the current state of the robot. During the periodic
    // loop of an existing goal, this should be set to the output
    // of the profile. This should only really be null if there
    // is no current state, so before any goal has ever been commanded.
    private TrapezoidProfile.State angularProfileState = null;

    private final RobotState robotState = RobotState.getInstance();

    private final Drive drive;
    private final Supplier<Pose2d> targetPose;

    private Optional<Double> distanceTolerance = Optional.empty();
    private Optional<Double> angleTolerance = Optional.empty();

    /**
     * Constructs a DriveToPoseBase command.
     *
     * @param drive The drive subsystem to control
     * @param targetPose Supplier providing the target pose to drive to
     */
    public FullSendToPose(Drive drive, Supplier<Pose2d> targetPose) {
        this.drive = drive;
        this.targetPose = targetPose;

        angularController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive);
    }

    /**
     * Sets the distance tolerance for the command to finish
     *
     * @param tolerance Allowable distance to target pose
     */
    public FullSendToPose withDistanceTolerance(Distance tolerance) {
        distanceTolerance = Optional.of(tolerance.in(Meters));
        return this;
    }

    /**
     * Sets the angular tolerance for the command to finish
     *
     * @param tolerance Allowable angle to target pose
     */
    public FullSendToPose withAngularTolerance(Angle tolerance) {
        angleTolerance = Optional.of(tolerance.in(Radians));
        return this;
    }

    /**
     * Sets both distance and angular tolerances for the command to finish
     *
     * @param distanceTolerance Allowable distance to target pose
     * @param angleTolerance Allowable angle to target pose
     */
    public FullSendToPose withTolerance(Distance distanceTolerance, Angle angleTolerance) {
        this.distanceTolerance = Optional.of(distanceTolerance.in(Meters));
        this.angleTolerance = Optional.of(angleTolerance.in(Radians));
        return this;
    }

    @Override
    public void initialize() {
        ChassisSpeeds fieldVelocity =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        drive.getChassisSpeeds(), robotState.getEstimatedPose().getRotation());

        linearController.reset();
        angularController.reset();

        Translation2d translationToTarget =
                targetPose
                        .get()
                        .getTranslation()
                        .minus(robotState.getEstimatedPose().getTranslation());
        resetLinearController(translationToTarget, fieldVelocity);
        resetAngularController(
                robotState.getEstimatedPose().getRotation().getRadians(),
                fieldVelocity.omegaRadiansPerSecond);
    }

    private void resetLinearController(
            Translation2d translationToTarget, ChassisSpeeds fieldVelocity) {
        double distanceToTarget = translationToTarget.getNorm();
        double velocityTowardTarget = 0.0;
        if (distanceToTarget > 1e-9) {
            velocityTowardTarget =
                    (fieldVelocity.vxMetersPerSecond * translationToTarget.getX()
                                    + fieldVelocity.vyMetersPerSecond * translationToTarget.getY())
                            / distanceToTarget;
        }

        linearProfileState = new TrapezoidProfile.State(distanceToTarget, -velocityTowardTarget);
        setLinearGoal();
    }

    private void resetAngularController(double radians, double omegaRadiansPerSecond) {
        angularProfileState = new TrapezoidProfile.State(radians, omegaRadiansPerSecond);
        setAngularGoal(targetPose.get().getRotation().getRadians());
    }

    private void updatePID() {
        linearController.setPID(LINEAR_P.get(), LINEAR_I.get(), LINEAR_D.get());
        linearProfile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(LINEAR_V.get(), LINEAR_A.get()));
        angularController.setPID(ANGULAR_P.get(), ANGULAR_I.get(), ANGULAR_D.get());
        angularProfile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(ANGULAR_V.get(), ANGULAR_A.get()));
    }

    private void setLinearGoal() {
        linearProfileGoal = new TrapezoidProfile.State(0.0, 0.0);
    }

    private void setAngularGoal(double targetRadians) {
        if (angularProfileState == null) {
            angularProfileState =
                    new TrapezoidProfile.State(
                            robotState.getEstimatedPose().getRotation().getRadians(), 0.0);
        }

        double goalRadians =
                angularProfileState.position
                        + MathUtil.angleModulus(targetRadians - angularProfileState.position);
        angularProfileGoal = new TrapezoidProfile.State(goalRadians, 0.0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Checks if tunable values for PID have changed and updates them if so
        updatePID();
        Pose2d target = targetPose.get();
        Pose2d currentPose = robotState.getEstimatedPose();
        setLinearGoal();
        setAngularGoal(target.getRotation().getRadians());

        // Calculate translation and direction to target
        Translation2d translationToTarget =
                target.getTranslation().minus(currentPose.getTranslation());

        Rotation2d directionToTarget = translationToTarget.getAngle();

        // Calculate outputs from controllers
        double linearOutput = 0.0;

        boolean hasValidLinearGoal = linearProfileGoal != null && linearProfileState != null;
        if (hasValidLinearGoal) {
            TrapezoidProfile.State nextState =
                    linearProfile.calculate(
                            LOOP_PERIOD_SECONDS, linearProfileState, linearProfileGoal);
            linearProfileState = nextState;

            double feedforwardOutput = nextState.velocity;
            double feedbackOutput =
                    linearController.calculate(translationToTarget.getNorm(), nextState.position);

            double totalOutput = feedforwardOutput + feedbackOutput;
            linearOutput = MathUtil.clamp(-totalOutput, -LINEAR_V.get(), LINEAR_V.get());
        }

        double angularOutput = 0.0;

        boolean hasValidAngularGoal = angularProfileGoal != null && angularProfileState != null;
        if (hasValidAngularGoal) {
            TrapezoidProfile.State nextState =
                    angularProfile.calculate(
                            LOOP_PERIOD_SECONDS, angularProfileState, angularProfileGoal);
            angularProfileState = nextState;

            double feedforwardOutput = nextState.velocity;
            double feedbackOutput =
                    angularController.calculate(
                            currentPose.getRotation().getRadians(), nextState.position);

            double totalOutput = feedforwardOutput + feedbackOutput;
            angularOutput = MathUtil.clamp(totalOutput, -ANGULAR_V.get(), ANGULAR_V.get());
        }

        // Convert to robot-relative speeds and set request velocities
        var fieldRelativeSpeed =
                new ChassisSpeeds(
                        linearOutput * Math.cos(directionToTarget.getRadians()),
                        linearOutput * Math.sin(directionToTarget.getRadians()),
                        angularOutput);

        drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        fieldRelativeSpeed, currentPose.getRotation()));

        Logger.recordOutput("FullSendToPose/Target Pose", target);
        Logger.recordOutput("FullSendToPose/Distance To Target (m)", translationToTarget.getNorm());
        Logger.recordOutput(
                "FullSendToPose/Linear Angle To Target (deg)", directionToTarget.getDegrees());
        Logger.recordOutput(
                "FullSendToPose/LinearController/ErrorMeters", linearController.getError());
        Logger.recordOutput(
                "FullSendToPose/LinearController/ProfilePositionMeters",
                linearProfileState.position);
        Logger.recordOutput(
                "FullSendToPose/LinearController/ProfileVelocityMetersPerSec",
                linearProfileState.velocity);
        Logger.recordOutput(
                "FullSendToPose/LinearController/ProfileGoalMeters", linearProfileGoal.position);
        Logger.recordOutput("FullSendToPose/LinearController/OutputMetersPerSec", linearOutput);
        Logger.recordOutput(
                "FullSendToPose/AngularController/ErrorRads", angularController.getError());
        Logger.recordOutput(
                "FullSendToPose/AngularController/ProfilePositionRads",
                angularProfileState.position);
        Logger.recordOutput(
                "FullSendToPose/AngularController/ProfileVelocityRadsPerSec",
                angularProfileState.velocity);
        Logger.recordOutput(
                "FullSendToPose/AngularController/ProfileGoalRads", angularProfileGoal.position);
        Logger.recordOutput("FullSendToPose/AngularController/OutputRadsPerSec", angularOutput);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (linearProfileGoal == null
                || linearProfileState == null
                || angularProfileGoal == null
                || angularProfileState == null) {
            return false;
        }

        double linearError = linearProfileGoal.position - linearProfileState.position;
        boolean withinDistanceTolerance =
                distanceTolerance
                        .map(
                                tolerance ->
                                        Math.abs(linearController.getError()) < tolerance
                                                && Math.abs(linearError) < tolerance
                                                && Math.abs(linearProfileState.velocity)
                                                        < LINEAR_VELOCITY_TOLERANCE_METERS_PER_SECOND)
                        .orElse(true);

        double angularError =
                MathUtil.angleModulus(angularProfileGoal.position - angularProfileState.position);
        boolean withinAngularTolerance =
                angleTolerance
                        .map(
                                tolerance ->
                                        Math.abs(angularController.getError()) < tolerance
                                                && Math.abs(angularError) < tolerance
                                                && Math.abs(angularProfileState.velocity)
                                                        < ANGULAR_VELOCITY_TOLERANCE_RADIANS_PER_SECOND)
                        .orElse(true);

        Logger.recordOutput(
                "FullSendToPose/Distance Tolerance Present", distanceTolerance.isPresent());
        Logger.recordOutput("FullSendToPose/Within Distance Tolerance", withinDistanceTolerance);
        Logger.recordOutput("FullSendToPose/Angular Tolerance Present", angleTolerance.isPresent());
        Logger.recordOutput("FullSendToPose/Within Angular Tolerance", withinAngularTolerance);

        if (distanceTolerance.isPresent() && angleTolerance.isPresent()) {
            return withinDistanceTolerance && withinAngularTolerance;
        }

        if (distanceTolerance.isPresent()) {
            return withinDistanceTolerance;
        }

        if (angleTolerance.isPresent()) {
            return withinAngularTolerance;
        }

        return false;
    }

    /**
     * Gets the current distance error to the target pose.
     *
     * @return The distance error in meters
     */
    public Distance getDistanceError() {
        return Meters.of(linearController.getError());
    }

    /**
     * Gets the current angular error to the target pose.
     *
     * @return The angular error in radians
     */
    public Angle getAngularError() {
        return Radians.of(angularController.getError());
    }
}
