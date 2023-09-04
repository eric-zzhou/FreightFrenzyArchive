package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.freightFrenzy.AutonomousMethods;
import org.firstinspires.ftc.teamcode.freightFrenzy.Constants;

@Autonomous(name = "RandomTest", group = "Linear Opmode")
//@Disabled
public class RandomTests extends AutonomousMethods {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    double speed = 1;

    @Override
    public void runOpMode() {
        initializeAutonomousAttachments(hardwareMap, telemetry);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        liftAndExtend(Constants.armTopAuto, Constants.extTop, Constants.extTopEarly, 9);
        sleep(120);
        drop();
        resetDropArm();
    }

}
