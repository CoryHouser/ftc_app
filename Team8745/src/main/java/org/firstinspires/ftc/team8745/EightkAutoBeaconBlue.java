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

        robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //   while (super.opModeIsActive()) {
        //Move to center
        robot.moveToPosition(-42, -.6);

        robot.resetEncoders();

        //Turn to beacon
        robot.turnIMU(-90);

        telemetry.addLine("first move and bump");
        telemetry.update();
        robot.resetEncoders();
        //Move to beacon and bump
        robot.moveToPosition(-48, -.45);
        telemetry.addLine("back off");
        telemetry.update();
        robot.resetEncoders();
        //back off beacon
        robot.moveToPosition(1, .1);

         robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            telemetry.addLine("we saw blue backing up");
            telemetry.update();
            robot.resetEncoders();
            robot.moveToPosition(15, .5);
            robot.setALLpower(0);
        } else { telemetry.addLine("We saw red waiting");
            telemetry.update();
            //senses wrong color and hits again
            robot.waitNSeconds(5);

            telemetry.addLine("We saw red bump");
            telemetry.update();
            robot.resetEncoders();
            robot.moveToPosition(-2, -.1);
            //back off
            telemetry.addLine("We saw red back up");
            telemetry.update();
            robot.resetEncoders();
            robot.moveToPosition(15, .4);
            robot.setALLpower(0);
        }
        //turns toward 2nd beacon
        telemetry.addLine("Turning to second beacon");
        telemetry.update();
        robot.resetEncoders();
        robot.turnIMU(90);
        //moves to second beacon
        robot.resetEncoders();
        telemetry.addLine("moving to front of second beacon");
        telemetry.update();
        robot.moveToPosition(-48, -.6);

        telemetry.addLine("last turn");
        telemetry.update();
        robot.resetEncoders();
        //Turn to beacon
        robot.turnIMU(-90);

        telemetry.addLine("move and first bump");
        telemetry.update();
        robot.resetEncoders();
        //bump beacon
        robot.moveToPosition(-20, -.45);

        telemetry.addLine("back off");
        telemetry.update();
        robot.resetEncoders();
        //back off beacon
        robot.moveToPosition(1, .1);


        telemetry.addLine("Check colors");
        telemetry.update();


        robot.waitNSeconds(1);
        //senses correct colors
        if (colors == 01 || colors == 10 || colors == 11 || colors == 12 || colors == 21) {
            telemetry.addLine("Color was blue back up and stop");
            telemetry.update();
            robot.resetEncoders();
            robot.moveToPosition(10, .5);
            robot.setALLpower(0);
            return;

        } else {
            telemetry.addLine("We saw red2 wait");
            telemetry.update();
            //hits beacon again if wrong
            robot.waitNSeconds(5);
            robot.resetEncoders();
            telemetry.addLine("We saw red2  bump");
            telemetry.update();
            robot.moveToPosition(-2, -.1);
            robot.resetEncoders();
            telemetry.addLine("We saw red back up");
            telemetry.update();
            robot.moveToPosition(10, .5);
            robot.setALLpower(0);
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