package org.firstinspires.ftc.teamcode.testing;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.freightFrenzy.Attachments;
import org.firstinspires.ftc.teamcode.freightFrenzy.Constants;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="NavigationSensorsTest", group="Iterative Opmode")
//@Disabled
public class NavigationSensorsTest extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Attachments myRobot = new Attachments();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        myRobot.initialize(hardwareMap, telemetry);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //Drive motor controls
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double speedMultiplier = 1;
        double rotationMultiplier = .8;

        if(gamepad1.dpad_up){
            ly=1;
            lx=0;
            speedMultiplier = 0.3;
        }
        else if(gamepad1.dpad_down){
            ly=-1;
            lx=0;
            speedMultiplier = 0.3;
        }
        if(gamepad1.dpad_left){
            lx=-1;
            ly=0;
            speedMultiplier = 0.6;
        }
        else if(gamepad1.dpad_right){
            lx=1;
            ly=0;
            speedMultiplier = 0.6;
        }


        double theta = Math.atan2(lx, ly);
        double v_theta = Math.sqrt(lx * lx + ly * ly);
        double v_rotation = gamepad1.right_stick_x;

        myRobot.drive(theta,  speedMultiplier*v_theta, rotationMultiplier*v_rotation);



        telemetry.addData("front distance: ",  myRobot.getFrontDistance());
        telemetry.addData("right distance: ", myRobot.getRightDistance());
        telemetry.addData("left distance: ", myRobot.getLeftDistance());
        telemetry.addData("back distance: ", myRobot.getBackDistance());
        telemetry.addData("rightfront distance: ", myRobot.getRightFrontDistance());
        telemetry.addData("leftfront distance: ", myRobot.getLeftFrontDistance());
        telemetry.addData("frontleft distance: ", myRobot.getFrontLeftDistance());
        telemetry.addData("backleft distance: ", myRobot.getBackLeftDistance());
        telemetry.addData("rf encoder: ",  myRobot.rf.getCurrentPosition() / Constants.TICKS_PER_INCH);
        telemetry.addData("rb encoder: ", myRobot.rb.getCurrentPosition() / Constants.TICKS_PER_INCH);
        telemetry.addData("lf encoder: ", myRobot.lf.getCurrentPosition() / Constants.TICKS_PER_INCH);
        telemetry.addData("lb encoder: ", myRobot.lb.getCurrentPosition() / Constants.TICKS_PER_INCH);
        telemetry.addData("imu: ", myRobot.getAngle());
        //myRobot.readEncoders();
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        Log.d("AHHHHH front: ",  "" + myRobot.getFrontDistance());
        Log.d("AHHHHH right: ", "" + myRobot.getRightDistance());
        Log.d("AHHHHH left: ", "" + myRobot.getLeftDistance());
        Log.d("AHHHHH back: ", "" + myRobot.getBackDistance());
        Log.d("AHHHHH rightfront: ", "" + myRobot.getRightFrontDistance());
        Log.d("AHHHHH leftfront: ", "" + myRobot.getLeftFrontDistance());
        Log.d("AHHHHH frontleft: ", "" + myRobot.getFrontLeftDistance());
        Log.d("AHHHHH backleft: ", "" + myRobot.getBackLeftDistance());
        Log.d("rf encoder: ",  "" + myRobot.rf.getCurrentPosition());
        Log.d("rb encoder: ", "" + myRobot.rb.getCurrentPosition());
        Log.d("lf encoder: ", "" + myRobot.lf.getCurrentPosition());
        Log.d("lb encoder: ", "" + myRobot.lb.getCurrentPosition());
        Log.d("AHHHHH imu: ", "" + myRobot.getAngle());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

