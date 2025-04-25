package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PurePursuit.Base.Controllers.PIDFEx;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Util.Timer;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotMap;

import java.util.ArrayList;

public class TestOpMode extends OpMode {

    private RobotMovement robotMovement;
    private RobotMap robotMap;
    private Timer timer;

    private Pose startingPose = new Pose(238, 238, 90);

    private enum PathState {
        FIRST,
        SECOND,
        STOP
    }

    private PathState pathState;

    private ArrayList<Pose> first = new ArrayList <Pose>() {{
        first.add(new Pose(230984, 23095,180));
        first.add(new Pose(643, 734,180));
        first.add(new Pose(63, 53,180));
    }};
    private ArrayList<Pose> second = new ArrayList <Pose>() {{
        second.add(new Pose(209836,389046,60));
        second.add(new Pose(251,78,60));
        second.add(new Pose(14,178,60));
    }};

    private ArrayList<Pose> temp = new ArrayList <Pose>();

    public void setPathState(PathState pState) {
        pathState = pState;
        timer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case FIRST:
                temp = first;
                setPathState(PathState.SECOND);
                break;
            case SECOND:
                if (robotMovement.isFinished()) {
                    robotMovement.newPath();
                    robotMovement.setConstantThetaInterpolation(70);
                    temp = second;

                    setPathState(PathState.STOP);
                }
                break;
        }
    }

    @Override
    public void init() {
        robotMap = new RobotMap(hardwareMap, telemetry);
        robotMovement = new RobotMovement(robotMap, startingPose);
    }

    @Override
    public void start() {
        robotMovement.newPath();
        setPathState(PathState.FIRST);
    }

    @Override
    public void loop() {
        robotMovement.followPathUpdate(temp);
        autonomousPathUpdate();
    }

    @Override
    public void stop() {
        Pose pose = robotMovement.getCurrentPose();
        // add pose transfer
    }
}
