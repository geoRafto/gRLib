package org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.Localization.GoBildaPinpointDriver;
import org.jetbrains.annotations.NotNull;

public class RobotMap {

    private DcMotorEx frontLeft, rearLeft, frontRight, rearRight;
    private IMU imu;
    private GoBildaPinpointDriver odo;
    private GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection, forwardEncoderDirection;
    private HardwareMap hm;
    private Telemetry telemetry;
    private GoBildaPinpointDriver.GoBildaOdometryPods encoderRes;

    public RobotMap(HardwareMap hm) {
        this(hm, null);
    }

    public RobotMap (HardwareMap hm, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hm = hm;

        /*--Motors--*/
        frontLeft = hm.get(DcMotorEx .class, "frontLeft");
        rearLeft = hm.get(DcMotorEx.class, "rearLeft");
        frontRight = hm.get(DcMotorEx.class, "frontRight");
        rearRight = hm.get(DcMotorEx.class, "rearRight");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        /*--IMU--*/
        imu = hm.get(IMU .class, "external_imu");
        IMU.Parameters imuParameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN
            )
        );
        imu.initialize(imuParameters);

        /*--Encoders--*/
        odo = hm.get(GoBildaPinpointDriver.class, "odometry");
        encoderRes = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        /*--Util--*/
        for (LynxModule module : hm.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    /*--Motors--*/
    public DcMotorEx getFrontLeft () {
        return frontLeft;
    }

    public DcMotorEx getFrontRight () {
        return frontRight;
    }

    public DcMotorEx getRearLeftMotor() {
        return rearLeft;
    }

    public DcMotorEx getRearRightMotor() {
        return rearRight;
    }

    /*--Encoders--*/
    public GoBildaPinpointDriver.GoBildaOdometryPods getEncoderRes() {
        return encoderRes;
    }

    public GoBildaPinpointDriver getOdometry() {
        return odo;
    }

    public GoBildaPinpointDriver.EncoderDirection getStrafeEncoderDirection() {
        return strafeEncoderDirection;
    }

    public GoBildaPinpointDriver.EncoderDirection getForwardEncoderDirection() {
        return forwardEncoderDirection;
    }

    /*--Util--*/
    @NotNull
    public HardwareMap getHardwareMap() {
        return hm;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    /*--IMU--*/
    public IMU getIMU() {
        return imu;
    }
}
