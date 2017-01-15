package org.firstinspires.ftc.team8745;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by darwin on 12/17/16.
 */
@Autonomous(name = "EightkAutoBeaconBlue")
public class EightkAutoBeaconBlue extends LinearOpMode {
    Hardware8745 robot = new Hardware8745();

    int foo;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        waitForStart();

        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //   while (super.opModeIsActive()) {
        //Move to center
        robot.moveToPosition(-48, -.2);

        robot.resetEncoders();
        //Turn to beacon
        robot.turnIMU(-90);

        robot.resetEncoders();
        //Move to beacon and bump
        robot.moveToPosition(-48, -.4);

        robot.resetEncoders();
        //back off beacon
        robot.moveToPosition(1, .1);

        //  robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.waitNSeconds(1);

        int colors = robot.Color_Case();
        telemetry.addData("colors", colors);
        telemetry.addData("CSensorL Red", robot.CSensorL.red());
        telemetry.addData("CSensorL Blue", robot.CSensorL.blue());
        telemetry.addData("CSensorR Red", robot.CSensorR.red());
        telemetry.addData("CSensorR Blue", robot.CSensorR.blue());
        telemetry.update();
        //senses correct colors
        if (colors == 01 || colors == 10 || colors == 11 || colors == 12 || colors == 21) {
            robot.resetEncoders();
            robot.moveToPosition(15, .4);

        } else {

            //senses wrong color and hits again
            robot.waitNSeconds(5);
            robot.resetEncoders();
            robot.moveToPosition(-2, .1);
            //back off
            robot.moveToPosition(15, .4);
        }
        //turns toward 2nd beacon
        robot.turnIMU(90);
        //moves to second beacon
        robot.moveToPosition(-48, -.2);

        robot.resetEncoders();
        //Turn to beacon
        robot.turnIMU(-90);

        robot.resetEncoders();
        //bump beacon
        robot.moveToPosition(-20, -.4);

        robot.resetEncoders();
        //back off beacon
        robot.moveToPosition(1, .1);


        telemetry.addLine("Made it to loop");
        telemetry.update();
        robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.waitNSeconds(1);
        //senses correct colors
        if (colors == 01 || colors == 10 || colors == 11 || colors == 12 || colors == 21) {

            robot.moveToPosition(10, .4);

            return;

        } else {
            //hits beacon again if wrong
            robot.waitNSeconds(5);
            robot.moveToPosition(-2, .1);
            robot.moveToPosition(15, .4);
        }

//                if (robot.CSensorR.blue() > 4) {
//
//                    robot.moveToPosition(15, .4);
//
//                    robot.resetEncoders();
//
//                    robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                    break;
//                } else {
//                    robot.moveToPosition(15, .4);
//                    robot.resetEncoders();
//
//                    robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                    robot.moveToPosition(-16, -.3);
//                    robot.resetEncoders();
//
//                    robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                }


        //     break;
        // }
    }
}