package org.firstinspires.ftc.teamcode.PurePursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.Localization.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotMap;


@TeleOp(name = "Localization Test", group = "Special")
public class LocalizationTest extends LinearOpMode {

    private RobotMap robotMap;
    private PinpointLocalizer localizer;

    private Pose startingPose = new Pose(0,0,0);

    @Override
    public void runOpMode() {
        robotMap = new RobotMap(hardwareMap, telemetry);
        localizer = new PinpointLocalizer(robotMap, startingPose);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            localizer.update();

            telemetry.addData("X: ", localizer.getPose().getX());
            telemetry.addData("Y: ", localizer.getPose().getY());
            telemetry.addData("Theta: ", MathFunction.angleWrap(localizer.getPose().getTheta()));
            telemetry.update();

            FtcDashboard.getInstance().getTelemetry().update();
        }
    }
}
