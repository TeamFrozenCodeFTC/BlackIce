package org.firstinspires.ftc.teamcode.blackice.core;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.blackice.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blackice.drivetrain.DrivetrainConfig;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackice.localizers.Localizer;
import org.firstinspires.ftc.teamcode.blackice.localizers.LocalizerConfig;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Follower {
    /**
     * Responsible for turning the robot and making sure it is facing the correct
     * direction.
     */
    public final PDController headingController;
    
    /**
     * Responsible holding a given pose and giving translational power for the robot to
     * stay on the path. Tune this as aggressively as possible without the robot shaking
     * while holding a pose. Error is in distance from target point (inches).
     */
    public final PredictiveBrakingController positionalController;

    public final Drivetrain drivetrain;
    public final Localizer localizer;
    
    public PoseTolerance poseTolerance;
    public MotionTolerance motionTolerance;
    
    boolean isBraking = false;
    Vector brakingVector;
    double deltaTime;
    double lastTime = System.nanoTime();

    public Follower(PDController headingController,
                    PredictiveBrakingController positionalController,
                    DrivetrainConfig drivetrain,
                    LocalizerConfig localizer,
                    HardwareMap hardwareMap,
                    PoseTolerance poseTolerance,
                    MotionTolerance motionTolerance) {
        this.headingController = headingController;
        this.positionalController = positionalController;
        this.drivetrain = drivetrain.build(hardwareMap);
        this.localizer = localizer.build(hardwareMap);
        this.drivetrain.zeroPowerFloatMode();
        this.poseTolerance = poseTolerance;
        this.motionTolerance = motionTolerance;
    }

    public void reset() {
        headingController.reset();
        isBraking = false;
        brakingVector = null;
    }
    
    public boolean isStoppedAt(Pose pose) {
        return poseTolerance.atPose(pose, localizer.getPose())
            && motionTolerance.isStopped(localizer.getVelocity(), localizer.getAngularVelocity());
    }

    /**
     * Instructs the drivetrain to follow a vector relative to the field.
     */
    public void followFieldVector(Vector fieldVector, double turnPower) {
        Vector robotVector = localizer.toRobotRelativeVector(fieldVector);
        drivetrain.followVector(robotVector, turnPower);
    }
    
    /**
     * Returns true if the robot is within braking distance of the target pose.
     */
    public void holdPose(Pose pose) {
        holdPose(pose, 1);
    }
    
    /**
     * Returns true if the robot is within braking distance of the target pose.
     */
    public boolean holdPose(Pose pose, double maxPower) {
        Vector holdPower = computeHoldPower(pose.getPosition());
        double powerMag = holdPower.computeMagnitude();
        if (powerMag > maxPower) {
            holdPower = holdPower.times(maxPower / powerMag);
        }

        if (localizer.getVelocity().computeMagnitude() < 0.1 && powerMag < 0.1) {
            holdPower = new Vector(0, 0);
        }

        double turnPower = Math.min(maxPower,
            computeHeadingCorrectionPower(Math.toRadians(pose.getHeading())));

        if (localizer.getAngularVelocity() < Math.toRadians(1) && Math.abs(turnPower) < 0.1) {
            turnPower = 0;
        }

        followFieldVector(holdPower, turnPower);
        
        return powerMag < 1;
    }

    public Vector computeHoldPower(Vector position) {
        Vector error =
            position.minus(localizer.getPose().getPosition());

        return error.map(localizer.getVelocity(),
                         positionalController::computeOutput);
    }

    public double computeHeadingCorrectionPower(double targetHeading) {
        double headingError =
            AngleUnit.RADIANS.normalize(targetHeading - localizer.getPose().getHeading());

        return headingController.computeOutput(headingError, deltaTime);
    }
    
    private double calculateDeltaTime() {
        double currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;
        return deltaTime;
    }
    
    public void update() {
        deltaTime = calculateDeltaTime();
        localizer.update(deltaTime);
    }
    
    public void setCurrentPose(Pose pose) {
        localizer.setCurrentPose(pose.getPosition().getX(), pose.getPosition().getY(),
                        pose.getHeading());
        localizer.update(deltaTime);
    }
}
