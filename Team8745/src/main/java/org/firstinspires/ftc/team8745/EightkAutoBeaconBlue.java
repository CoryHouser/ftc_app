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
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);


        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (super.opModeIsActive()) {
            //Move to center
            robot.moveToPosition(-48, -.2);

            robot.resetEncoders();
            //Turn to beacon
            robot.turnIMU(-90);

            robot.resetEncoders();
            //Move to beacon
            robot.moveToPosition(-48, -.4);

            robot.resetEncoders();

            robot.moveToPosition(1,.1);



                telemetry.addLine("Made it to loop");
                telemetry.update();
                robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.waitNSeconds(1);

                int colors = robot.Color_Case();
                telemetry .addData("colors",colors);
                telemetry.update();
                if(colors == 01 || colors == 10 || colors == 11 || colors ==21 || colors == 12){

                    robot.moveToPosition(15, .4);

                }
                else {
                    robot.waitNSeconds(5);
                    robot.moveToPosition(-2,.1);
                    robot.moveToPosition(15,.4);
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
        }
    }
}