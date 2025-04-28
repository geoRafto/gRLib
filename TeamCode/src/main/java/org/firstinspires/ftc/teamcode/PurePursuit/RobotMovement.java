package org.firstinspires.ftc.teamcode.PurePursuit;

import static org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction.atTarget;
import static org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction.calculateCircleIntersection;
import static org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction.map;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.PurePursuit.Base.Controllers.PIDFEx;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Vector;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.Localization.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotMap;

import java.util.ArrayList;

public class RobotMovement {

    /*--The Upper Grid--*/
    private RobotMap robotMap;
    private PinpointLocalizer localizer;
    private PIDFEx
        upperParallelPID,
        lowerParallelPID,
        upperPerpendicularPID,
        lowerPerpendicularPID,
        upperRotationalPID,
        lowerRotationalPID;

    /*-- Theta Interpolation --*/
    public enum ThetaInterpolation {
        TANGENTIAL,
        ANALOGICAL,
        CONSTANT,
        HYBRID
    }

    private ThetaInterpolation thetaInterpolation;

    private double
        finalTargetTheta,
        realTranslationalEndDistance,
        realTranslationalStartDistance,
        realThetaStartDistance,
        realThetaEndDistance;

    /*-- Logic --*/
    private boolean
        isFinished,
        nullDetected,
        autoStartingPosition,
        firstCycle,
        isReversed;

    private Vector currentTo_Point;

    /*-- Temp --*/
    private Pose currentPose = new Pose(0,0,0);

    /*-- Constructor --*/
    public RobotMovement(RobotMap robotMap, Pose startingPose) {
        this.robotMap = robotMap;
        localizer = new PinpointLocalizer(robotMap, startingPose);
        initializeControllers();
    }

    private void initializeControllers() {
        upperParallelPID = new PIDFEx(RobotConstants.getUpperParallelPID());
        lowerParallelPID = new PIDFEx(RobotConstants.getLowerParallelPID());
        upperPerpendicularPID = new PIDFEx(RobotConstants.getUpperPerpendicularPID());
        lowerPerpendicularPID = new PIDFEx(RobotConstants.getLowerPerpendicularPID());
        upperRotationalPID = new PIDFEx(RobotConstants.getUpperRotationalPID());
        lowerRotationalPID = new PIDFEx(RobotConstants.getLowerRotationalPID());
    }

    private void updateControllerCoefficients() {
        upperParallelPID.setCofficients(RobotConstants.getUpperParallelPID());
        lowerParallelPID.setCofficients(RobotConstants.getLowerParallelPID());
        upperPerpendicularPID.setCofficients(RobotConstants.getUpperPerpendicularPID());
        lowerPerpendicularPID.setCofficients(RobotConstants.getLowerPerpendicularPID());
        upperRotationalPID.setCofficients(RobotConstants.getUpperRotationalPID());
        lowerRotationalPID.setCofficients(RobotConstants.getLowerRotationalPID());
    }

    private void updateControllerCoefficients(Pose confs) {
        upperParallelPID.setP(confs.getX());
        lowerParallelPID.setCofficients(RobotConstants.getLowerParallelPID());
        upperPerpendicularPID.setP(confs.getY());
        lowerPerpendicularPID.setCofficients(RobotConstants.getLowerPerpendicularPID());
        upperRotationalPID.setP(confs.getTheta());
        lowerRotationalPID.setCofficients(RobotConstants.getLowerRotationalPID());
    }

    public void updateLocalizer() {
        currentPose = localizer.getPose();
    }

    /*-- Async Pure Pursuit Logic --*/
    public void followPathUpdate(ArrayList <Pose> points) {

        Pose motorsPower;
        Pose followPoint = points.get(0);

        Pose start;
        Pose end;

        Pose realEnd = points.get(points.size() - 1);
        Pose realStart = new Pose(0,0,0);

        updateLocalizer();
        if (autoStartingPosition && firstCycle) {
            points.add(0, currentPose);
        }

        if (firstCycle) {
            realStart = points.get(0);
        }

        updateLocalizer();
        updateControllerCoefficients();

        realTranslationalEndDistance = Math.hypot(realEnd.getX() - currentPose.getX(),
                                                  realEnd.getY() - currentPose.getY());

        realTranslationalStartDistance = Math.hypot(realStart.getX() - currentPose.getX(),
                                                    realStart.getY() - currentPose.getY());

        realThetaStartDistance = realStart.getTheta() - currentPose.getTheta();

        realThetaEndDistance = realEnd.getTheta() - currentPose.getTheta();

        double currentRadius = map(
            Math.hypot(Math.abs(localizer.getVelocity().getX()),
                       Math.abs(localizer.getVelocity().getY())),
            0,
            RobotConstants.getMaxVelocity(),
            RobotConstants.getMinRadiusRange(),
            RobotConstants.getMaxRadiusRange());

        if (atTarget(currentPose, realEnd)) {
            isFinished = true;
        }

        if (realTranslationalEndDistance <= currentRadius) {

            currentTo_Point = realEnd.getVec();

        } else {

            for (int i = points.size() - 2; i >= 0; --i) {
                start = points.get(i);
                end = points.get(i + 1);

                currentTo_Point = calculateCircleIntersection(
                    currentPose.getVec(),
                    currentRadius,
                    start.getVec(),
                    end.getVec()
                );

                if (currentTo_Point != null) {
                    nullDetected = false;
                    followPoint.setVec(currentTo_Point);
                    break;

                } else {
                    nullDetected = true;
                    currentRadius += currentRadius / 10;
                }
            }
        }

        updateControllerCoefficients(
            calculateCurrentCoefficients(realTranslationalStartDistance, realTranslationalEndDistance,
                                         realThetaStartDistance, realThetaEndDistance)
        );
        followPoint.setTheta(calculateCurrentTheta(currentPose, currentTo_Point, realEnd));
        motorsPower = goToPoint(followPoint, currentPose, realTranslationalEndDistance, realThetaEndDistance);
        robotCentricMovement(motorsPower);

        firstCycle = false;
    }

    /*-- Control Magic --*/
    public Pose goToPoint(Pose followPose, Pose currentPose, double error, double thetaError) {
        Pose answers = new Pose(0,0,0);

        double relative_x =
            currentPose.getX() + (followPose.getX() - currentPose.getX()) / Math.cos(currentPose.getTheta());
        double relative_y =
            currentPose.getY() + (followPose.getY() - currentPose.getY()) * Math.tan(currentPose.getTheta());

        upperParallelPID.setSetPoint(relative_x);
        lowerParallelPID.setSetPoint(relative_x);
        upperPerpendicularPID.setSetPoint(relative_y);
        lowerPerpendicularPID.setSetPoint(relative_y);
        upperRotationalPID.setSetPoint(followPose.getTheta());
        lowerRotationalPID.setSetPoint(followPose.getTheta());

        if (error <= RobotConstants.getLowerPIDThreshold_X()) {
            answers.setX(lowerParallelPID.calculate(currentPose.getX()));
        } else {
            answers.setX(upperParallelPID.calculate(currentPose.getX()));
        }

        if (error <= RobotConstants.getLowerPIDThreshold_Y()) {
            answers.setY(lowerPerpendicularPID.calculate(currentPose.getY()));
        } else {
            answers.setY(upperPerpendicularPID.calculate(currentPose.getY()));
        }

        if (thetaError <= RobotConstants.getRotationalLowerPIDThreshold()) {
            answers.setTheta(lowerRotationalPID.calculate(currentPose.getTheta()));
        } else {
            answers.setTheta(upperRotationalPID.calculate(currentPose.getTheta()));
        }

        return answers;
    }

    /*-- Velocity Control Magic --*/
    public Pose calculateCurrentCoefficients(double errorStart, double errorEnd,
                                             double thetaErrorStart, double thetaErrorEnd) {
        Pose answer = new Pose(0,0,0);

        double distEnd_X = errorEnd - RobotConstants.getLowerPIDThreshold_X();
        double distEnd_Y = errorEnd - RobotConstants.getLowerPIDThreshold_Y();
        double distEnd_theta = thetaErrorEnd - RobotConstants.getRotationalLowerPIDThreshold();

        double trigger_X =
            (RobotConstants.getUpperParallelPID().getkP() - RobotConstants.getLowerParallelPID().getkP()) / RobotConstants.getMaxDecceleration();
        double trigger_Y =
            (RobotConstants.getUpperPerpendicularPID().getkP() - RobotConstants.getLowerPerpendicularPID().getkP()) / RobotConstants.getMaxDecceleration();
        double trigger_Theta =
            (RobotConstants.getUpperRotationalPID().getkP() - RobotConstants.getLowerRotationalPID().getkP()) / RobotConstants.getMaxRotationalDecceleration();

        if (errorEnd <= trigger_X) {
            answer.setX(Range.clip(
                RobotConstants.getMaxDecceleration() * distEnd_X + RobotConstants.getLowerParallelPID().getkP(),
                RobotConstants.getLowerParallelPID().getkP(),
                RobotConstants.getUpperParallelPID().getkP()
            ));
        } else {
            answer.setX(Range.clip(
                RobotConstants.getMaxAcceleration() * errorStart + RobotConstants.getLowerParallelPID().getkP(),
                RobotConstants.getLowerParallelPID().getkP(),
                RobotConstants.getUpperParallelPID().getkP()
            ));
        }

        if (errorEnd <= trigger_Y) {
            answer.setY(Range.clip(
                RobotConstants.getMaxDecceleration() * distEnd_Y + RobotConstants.getLowerPerpendicularPID().getkP(),
                RobotConstants.getLowerPerpendicularPID().getkP(),
                RobotConstants.getUpperPerpendicularPID().getkP()
            ));
        } else {
            answer.setY(Range.clip(
                RobotConstants.getMaxAcceleration() * errorStart + RobotConstants.getLowerPerpendicularPID().getkP(),
                RobotConstants.getLowerPerpendicularPID().getkP(),
                RobotConstants.getUpperPerpendicularPID().getkP()
            ));
        }

        if (thetaErrorEnd <= trigger_Theta) {
            answer.setTheta(Range.clip(
                RobotConstants.getMaxRotationalDecceleration() * distEnd_theta + RobotConstants.getLowerRotationalPID().getkP(),
                RobotConstants.getLowerRotationalPID().getkP(),
                RobotConstants.getUpperRotationalPID().getkP()
            ));
        } else {
            answer.setTheta(Range.clip(
                RobotConstants.getMaxRotationalAcceleration() * thetaErrorStart + RobotConstants.getLowerRotationalPID().getkP(),
                RobotConstants.getLowerRotationalPID().getkP(),
                RobotConstants.getUpperRotationalPID().getkP()
            ));
        }

        return answer;
    }

    private double calculateCurrentTheta(Pose currentPose, Vector currentTo_Point, Pose realEnd) {
        switch (thetaInterpolation) {
            case CONSTANT:
                break;

            case TANGENTIAL:
                finalTargetTheta = Math.toDegrees(Math.atan2(
                        currentTo_Point.getY() - currentPose.getY(),
                        currentTo_Point.getX() - currentPose.getX()
                ));
                break;

            case ANALOGICAL:
                // Under construction for now
                break;

            case HYBRID:
                if (Math.hypot(currentTo_Point.getX() - currentPose.getX(),
                              currentTo_Point.getY() - currentPose.getY()) <=
                    RobotConstants.getHybridThetaDistanceThreshold()) {
                    finalTargetTheta = realEnd.getTheta();
                } else {
                    finalTargetTheta = Math.toDegrees(Math.atan2(
                        currentTo_Point.getY() - currentPose.getY(),
                        currentTo_Point.getX() - currentPose.getX()
                    ));
                }
                break;
        }

        if (isReversed) {
            finalTargetTheta = finalTargetTheta - 180;
        }

        return finalTargetTheta;
    }

    /*-- Movement --*/
    public void robotCentricMovement(Pose pose) {
        // Parallel encoder motion Y Axis
        // Perpendicular encoder motion X Axis
        // Rotational IMU motion Theta

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(abs(pose.getY()) + abs(pose.getX()) + abs(pose.getTheta()), 1);
        double frontLeftPower = (pose.getY() + pose.getX() + pose.getTheta()) / denominator;
        double backLeftPower = (pose.getY() - pose.getX() + pose.getTheta()) / denominator;
        double frontRightPower = (pose.getY() - pose.getX() - pose.getTheta()) / denominator;
        double backRightPower = (pose.getY() + pose.getX() - pose.getTheta()) / denominator;

        robotMap.getFrontLeft().setPower(frontLeftPower);
        robotMap.getRearLeftMotor().setPower(backLeftPower);
        robotMap.getFrontRight().setPower(frontRightPower);
        robotMap.getRearRightMotor().setPower(backRightPower);
    }

    public void breakFollowing() {
        robotMap.getFrontLeft().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robotMap.getRearLeftMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robotMap.getFrontRight().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robotMap.getRearRightMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        isFinished = true;
    }

    /*-- Functions --*/
    public boolean isFinished() {
        return isFinished;
    }

    public boolean isNullDetected() {
        return nullDetected;
    }

    public void newPath() {
        isFinished = false;
        firstCycle = true;

        /*-- Default System Settings --*/
        setThetaInterpolation(ThetaInterpolation.HYBRID);
        autoStartingPosition = true;
    }

    public void setAutoStartingPosition(boolean set) {
        autoStartingPosition = set;
    }

    public void setThetaInterpolation(ThetaInterpolation set) {
        if (set == ThetaInterpolation.CONSTANT) {
            RobotLog.e("Connot set Constant Interpolation from this function, use: " +
                           "setConstantThetaInterpolation()");
        } else {
            thetaInterpolation = set;
        }
    }

    public void setConstantThetaInterpolation(double set) {
        thetaInterpolation = ThetaInterpolation.CONSTANT;
        finalTargetTheta = set;
    }

    public Pose getCurrentPose() {
        return currentPose;
    }

    public void setCurrentPose(Pose set) {
        localizer.setPose(set);
        currentPose = set;
    }

    public double getFinalTargetTheta() {
        return finalTargetTheta;
    }

    public double getRealTranslationalStartDistance() {
        return realTranslationalStartDistance;
    }

    public double getRealTranslationalEndDistance() {
        return realTranslationalEndDistance;
    }

    public double getRealThetaStartDistance() {
        return realThetaStartDistance;
    }

    public double getRealThetaEndDistance() {
        return realThetaEndDistance;
    }

    public void setReversed(boolean set) {
        isReversed = set;
    }
}