#!/usr/bin/env python3

# py -3 -m robotpy deploy --skip-tests --no-verify --no-install   #to deploy code

import wpilib

from CoreComponents.CoreFunctions import *
from rev import SparkBase, SparkFlex, SparkMax, SparkLowLevel, SparkFlexConfig, SparkMaxConfig, ClosedLoopConfig
import ntcore
import math


class MyRobot(wpilib.TimedRobot):
    # The channel on the driver station that the joystick is connected to
    joystickChannel = 0

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """

        # If True, "north" in field centric is centered and without offset. This is overwritten in auton init
        self.teleopFirst = True
        
        # Robot specific characteristics
        wheelbaseWidth = 24.75  # in inches
        wheelbaseLength = 24.75  # in inches

        rotationalGearRatio = 12.8  # Gear ratio for MK3 and MK4: 12.8 . Use 1 for testbed. Auto select in code depending on self.testBed boolean
        driveRotPerInch = 0.436

        self.joystickDeadzone = 0.1  # Global value for all sticks
        gyroDeadzone = 1  # was 2   # in degrees # changed and tightened up 2-21-2025
        autoDriveEndpointSlop = 4  # in inches

        # *** Essential to declare driveMotorType
        driveMotorType = 2  # 1 = Neo, 2 = Kraken
        rotationMotorType = 1  # 1 = Neo, 2 = Kraken

        # Initializing plugs, switches, limit/IR sensor
        cancoderPlug = wpilib.DigitalInput(0)  # Shorting plug present in DIO 0 for NOT using cancoder
        cancoderAbsent = not cancoderPlug.get()  # Plug present in DIO 0 means cancoderAbsent is True (reversed with not) and is in NO cancoder present mode.
        # If cancoderAbsent True, no cancoders used in robotInit for initial swerve rotational motor offset on boot or anywhere else.
        # Subsequent rotational position is only determined by rotational motor encoder updates and not the actual cancoders.
        # Swerve will still function if wheels are aligned properly at start up. All 4 wheels aligned straight with gears on right side.
        # This is worst case scenario on the field.

        self.teamColorSwitch = wpilib.DigitalInput(1)
        self.autonPosition1Switch = wpilib.DigitalInput(2)
        self.autonPosition3Switch = wpilib.DigitalInput(3)
        self.autonOption1Switch = wpilib.DigitalInput(4)
        self.autonOption3Switch = wpilib.DigitalInput(5)
        self.coralIntakeIRSensor = wpilib.DigitalInput(8) #was 7
        # self.elbowLimitSwitch = wpilib.DigitalInput(7)

        testbedPlug = wpilib.DigitalInput(9) # Shorting plug present in DIO 9 for Testbed mode.
        testbed = not testbedPlug.get()   #Plug present in DIO 9 means testbed is True (reversed with not) and is in testbed mode.

        # Establishing default limelight pipelines
        limelight1_DefaultPipeline = 0
        limelight2_DefaultPipeline = 7

        # Declaring coreFunctions instances
        self.coreFunctionsInstance = CoreFunctions(driveMotorType, rotationMotorType, testbed, cancoderAbsent,
                                                   autoDriveEndpointSlop,
                                                   wheelbaseWidth, wheelbaseLength, rotationalGearRatio,
                                                   driveRotPerInch,
                                                   self.joystickDeadzone, gyroDeadzone, limelight1_DefaultPipeline, limelight2_DefaultPipeline)

        # Timers
        self.coralIntakeRateOfChange = RateOfChange()
        self.emergencyElbowReset = RateOfChange()
        self.emergencyWristReset = RateOfChange()
        self.coralIntakeTimer = NiftyTimer()
        self.coralTransferTimer = NiftyTimer()
        self.coralTroughDropTimer = NiftyTimer()
        self.coralEjectionTimer = NiftyTimer()
        self.delayTimer = NiftyTimer()

        # Declaring joystick controllers
        self.joystick0 = self.coreFunctionsInstance.joystick0  # First controller - Driver
        self.joystick1 = self.coreFunctionsInstance.joystick1  # Second controller  - Gadget
        self.joystick2 = self.coreFunctionsInstance.joystick2  # Third controller  - Tuning


        # Taz motors
        self.coralIntake = SparkFlex(10, SparkLowLevel.MotorType.kBrushless)
        self.coralIntake_Enc = self.coralIntake.getEncoder()
        # self.coralIntake_Enc.setPosition(0) # Important to reset on code redeploy so encoder not persistent without whole system reboot
        self.coralIntake_PID = self.coralIntake.getClosedLoopController()

        self.coralIntake_config = SparkMaxConfig()
        self.coralIntake_config.inverted(False)
        self.coralIntake_config.closedLoop.pid(0.1, 0.0, 0.0)
        self.coralIntake_config.closedLoop.outputRange(-0.5, 0.3)   # Min was -0.3

        self.coralIntake.configure(self.coralIntake_config, SparkBase.ResetMode.kResetSafeParameters,
                                SparkBase.PersistMode.kPersistParameters)
        # Using kResetSafeParameters and not kNoResetSafeParameters speeds up boot significantly since parameter not checked
        # Should be OK since no SafeParameters are essential or changed to begin with.

        self.coralIntakeWrist = SparkMax(11, SparkLowLevel.MotorType.kBrushless)
        self.coralIntakeWrist_Enc = self.coralIntakeWrist.getEncoder()
        self.coralIntakeWrist_PID = self.coralIntakeWrist.getClosedLoopController()

        self.coralIntakeWrist_config = SparkMaxConfig()
        self.coralIntakeWrist_config.inverted(True)
        self.coralIntakeWrist_config.closedLoop.pid(0.1, 0.0, 0.0)
        self.coralIntakeWrist_config.closedLoop.outputRange(-0.3, 0.4)  # Was Min -0.25 / Max 0.35

        self.coralIntakeWrist.configure(self.coralIntakeWrist_config, SparkBase.ResetMode.kResetSafeParameters,
                                   SparkBase.PersistMode.kPersistParameters)

        self.coralIntakeElbow = SparkFlex(12, SparkLowLevel.MotorType.kBrushless)
        self.coralIntakeElbow_Enc = self.coralIntakeElbow.getEncoder()
        self.coralIntakeElbow_PID = self.coralIntakeElbow.getClosedLoopController()

        self.coralIntakeElbow_config = SparkMaxConfig()
        self.coralIntakeElbow_config.inverted(True)
        self.coralIntakeElbow_config.closedLoop.pid(0.1, 0.0, 0.0)
        self.coralIntakeElbow_config.closedLoop.outputRange(-0.75, 0.85)  # min was -0.7, Max was 0.8. *** OOPS

        self.coralIntakeElbow.configure(self.coralIntakeElbow_config, SparkBase.ResetMode.kResetSafeParameters,
                                        SparkBase.PersistMode.kPersistParameters)

        self.coralTilt = SparkMax(13, SparkLowLevel.MotorType.kBrushless)
        self.coralTilt_Enc = self.coralTilt.getEncoder()
        self.coralTilt_PID = self.coralTilt.getClosedLoopController()

        self.coralTilt_config = SparkMaxConfig()
        self.coralTilt_config.inverted(True)
        self.coralTilt_config.closedLoop.pid(0.1, 0.0, 0.0)
        self.coralTilt_config.closedLoop.outputRange(-0.25, 0.1) #min was -0.15, max was 0.15

        self.coralTilt.configure(self.coralTilt_config, SparkBase.ResetMode.kResetSafeParameters,
                                   SparkBase.PersistMode.kPersistParameters)

        self.cascadeOutputCoralMax = 0.35 #was 0.4
        self.cascadeOutputAlgaeMax = 0.1

        self.cascadeR = SparkFlex(14, SparkLowLevel.MotorType.kBrushless)
        self.cascadeR_Enc = self.cascadeR.getEncoder()
        self.cascadeR_PID = self.cascadeR.getClosedLoopController()

        self.cascadeR_config = SparkFlexConfig()
        self.cascadeR_config.inverted(False)
        self.cascadeR_config.closedLoop.pid(0.1, 0.0, 0.0)
        self.cascadeR_config.closedLoop.outputRange(-0.18, self.cascadeOutputCoralMax) # Min was -0.18

        self.cascadeR.configure(self.cascadeR_config, SparkBase.ResetMode.kResetSafeParameters,
                                SparkBase.PersistMode.kPersistParameters)

        self.cascadeL = SparkFlex(15, SparkLowLevel.MotorType.kBrushless)
        self.cascadeL_Enc = self.cascadeL.getEncoder()
        self.cascadeL_PID = self.cascadeL.getClosedLoopController()

        self.cascadeL_config = SparkFlexConfig()
        self.cascadeL_config.inverted(True)
        self.cascadeL_config.closedLoop.pid(0.1, 0.0, 0.0)
        self.cascadeL_config.closedLoop.outputRange(-0.18, self.cascadeOutputCoralMax)  # Min was -0.18

        self.cascadeL.configure(self.cascadeL_config, SparkBase.ResetMode.kResetSafeParameters,
                                SparkBase.PersistMode.kPersistParameters)

        self.turtleneck = SparkMax(16, SparkLowLevel.MotorType.kBrushless)
        self.turtleneck_Enc = self.turtleneck.getEncoder()
        self.turtleneck_PID = self.turtleneck.getClosedLoopController()

        self.turtleneck_config = SparkMaxConfig()
        self.turtleneck_config.inverted(True)
        self.turtleneck_config.closedLoop.pid(0.1, 0.0, 0.0)
        self.turtleneck_config.closedLoop.outputRange(-0.9, 0.8)    # min was 0.8

        self.turtleneck.configure(self.turtleneck_config, SparkBase.ResetMode.kResetSafeParameters,
                                   SparkBase.PersistMode.kPersistParameters)

        self.algaeBoot = SparkMax(17, SparkLowLevel.MotorType.kBrushless)
        self.algaeBoot_Enc = self.algaeBoot.getEncoder()
        self.algaeBoot_PID = self.algaeBoot.getClosedLoopController()

        self.algaeBoot_config = SparkMaxConfig()
        self.algaeBoot_config.inverted(True)
        self.algaeBoot_config.closedLoop.pid(0.1, 0.0, 0.0)
        self.algaeBoot_config.closedLoop.outputRange(-0.3, 0.3)

        self.algaeBoot.configure(self.algaeBoot_config, SparkBase.ResetMode.kResetSafeParameters,
                                   SparkBase.PersistMode.kPersistParameters)

        self.coralEject = SparkMax(18, SparkLowLevel.MotorType.kBrushless)
        self.coralEject_Enc = self.coralEject.getEncoder()
        self.coralEject_PID = self.coralEject.getClosedLoopController()

        self.coralEject_config = SparkMaxConfig()
        self.coralEject_config.inverted(False)
        self.coralEject_config.closedLoop.pid(0.1, 0.0, 0.0)
        self.coralEject_config.closedLoop.outputRange(-0.2, 0.25)

        self.coralEject.configure(self.coralEject_config, SparkBase.ResetMode.kResetSafeParameters,
                                   SparkBase.PersistMode.kPersistParameters)

        self.coralEject_Enc.setPosition(0)  # Manually set to zero with new code upload, even if robot not power cycled

        self.climb = SparkFlex(19, SparkLowLevel.MotorType.kBrushless)
        self.climb_Enc = self.climb.getEncoder()
        self.climb_PID = self.climb.getClosedLoopController()

        self.climb_config = SparkFlexConfig()
        self.climb_config.inverted(False)
        self.climb_config.closedLoop.pid(0.1, 0.0, 0.0)
        self.climb_config.closedLoop.outputRange(-0.8, 0.8)

        self.climb.configure(self.climb_config, SparkBase.ResetMode.kResetSafeParameters,
                                   SparkBase.PersistMode.kPersistParameters)

        # self.climb_Enc.setPosition(0) # May need to re-activate if not zeroing with robot power cycle.

        self.mouseTrap = SparkMax(20, SparkLowLevel.MotorType.kBrushless)
        self.mouseTrap_Enc = self.mouseTrap.getEncoder()
        self.mouseTrap_PID = self.mouseTrap.getClosedLoopController()

        self.mouseTrap_config = SparkMaxConfig()
        self.mouseTrap_config.inverted(False)
        self.mouseTrap_config.closedLoop.pid(0.1, 0.0, 0.0)
        self.mouseTrap_config.closedLoop.outputRange(-0.3, 0.3)

        self.mouseTrap.configure(self.mouseTrap_config, SparkBase.ResetMode.kResetSafeParameters,
                                 SparkBase.PersistMode.kPersistParameters)

        self.mouseTrap_Enc.setPosition(0)


        #  Essential constants modified downstream / Primers
        # This needs to be after motor declarations since some default are current enc positions
        self.robotInitDefaults()

        # networktable initialization
        self.tazTableInstance = ntcore.NetworkTableInstance.getDefault()
        self.tazTable = self.tazTableInstance.getTable("datatable")

        self.tazMotorsPub()
        self.DIO_Pub()
        self.joystickPub()

    def autonomousInit(self):   # autonCode
        """This function is run once each time the robot enters autonomous mode."""
        # Essential defaults
        self.autonomous = True
        self.teleoperated = False
        self.teleopFirst = False
        self.autonStep = 1

        # repeat configuration of intake
        self.coralIntake_config.inverted(False)
        self.coralIntake_config.closedLoop.pid(0.1, 0.0, 0.0)
        self.coralIntake_config.closedLoop.outputRange(-0.5, 0.3)  # Min was -0.3

        self.coralIntake.configure(self.coralIntake_config, SparkBase.ResetMode.kResetSafeParameters,
                                   SparkBase.PersistMode.kPersistParameters)

        ### gyroCompensation needs to be passed down to teleop if starting with auton in run

        self.autonTeleop_InitDefaults()

        # Reading switches
        self.readingAutonSwitches()

        if self.autonPosition == 1:
            # self.gyroCompensation = 90
            self.gyroCompensation = 0
        elif self.autonPosition == 2:
            self.gyroCompensation = 180
        elif self.autonPosition == 3:
            # self.gyroCompensation = -90
            self.gyroCompensation = 0

        # if self.autonPosition == 1:
        #     # self.gyroCompensation = 90
        #     self.gyroCompensation = 180
        # elif self.autonPosition == 2:
        #     self.gyroCompensation = 180
        # elif self.autonPosition == 3:
        #     # self.gyroCompensation = -90
        #     self.gyroCompensation = 180

        self.autoFaceAngle = self.gyroCompensation  # Default face angle at start


    def autonomousPeriodic(self):   # auton
        """This function is called periodically during autonomous."""
        self.variableResets()  # variables setting to default prior to manipulation belowf

        self.readingSensors()   # Collecting DIO data

        self.coreFunctionsInstance.gyroAngleAdjustments(self.autonomous, self.gyroCompensation)

        # Defaulting variable to 0 and awaiting possible replacement below. Only for auton.
        self.coreFunctionsInstance.zeroingJoystickVariables()
        # activateRumbling = False

        """Start autonSteps below"""

        # In auton, if maxAutoDriveNStopSpeed modified in steps, need reset criteria for entrance e.g right coatRack by = False
        # in order to set speedMultiplier back to OG.

        if self.autonStep == 1:
            if self.autonPosition == 1:
                self.autonStep = 400
            if self.autonPosition == 2:
                self.autonStep = 210
            if self.autonPosition == 3:
                self.autonStep = 400

        # Position 1
        if self.autonStep == 115: # the power is a % of speedLimitMultiplier. Can increase self.speedLimitMultiplier to speed up.
            angle = 180
            power = 0.8 # Was 1
            distance = 70 #was 80
            self.autoFaceAngle = 120
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power, distance) # failoutTime is optional
            if gotoNextAutonStep:
                self.autonCoral4DropOff = True
                self.autonStep += 1

            # if self.thresholdDistanceAutoDrive_Coral1 > distanceLeft > 0: # Lift cascade once close enough
            #     if self.newAutonCascade:
            #         self.autonCoral4DropOff = True
            #         self.newAutonCascade = False    # Restricting self.autonCoral4DropOff to 1 cycle so reverse ejection during lift not repeated
            # if gotoNextAutonStep:
            #     self.newAutonCascade = True # Resetting
            #     self.autonStep += 1

        if self.autonStep == 116:    # Autoseek activated for coral #1 dropoff / right coat rack
            if self.teamBlue:
                self.limelight2Pipeline = self.coral1R_Pos1_Blue
            else:
                self.limelight2Pipeline = self.coral1R_Pos1_Red

            self.autonRightCoatRackSeek = True  # Should activate proper pipeline

            self.autonStep += 1
            # if self.thresholdDistanceAutoSeek > self.autoSeekNStopDistanceLeftInInches > 0: # List cascade once close enough
            #     self.autonCoral4DropOff = True
            #     self.autonStep += 1

        if self.autonStep == 117:   # Eject coral once basket in position.
            if self.coralTilt_Enc.getPosition() > self.coralTilt4 - self.setpointTriggerZoneFine:
                self.autonCoralEjection = True  # Self resetting
                self.autonStep += 1

        if self.autonStep == 118:   # Allow enough time for ejection before moving on to next step.
            timesUp = self.coralEjectionTimer.autoTimer(0.5)
            if timesUp:
                self.coreFunctionsInstance.autoDriveReset()
                self.autonRightCoatRackSeek = False  # Turning off autoseek so variable can be reset.
                self.autonStep = 120

        if self.autonStep == 120:   # cascade back to base and robot moving on to next target.
            self.autonCascadeDownNReset()   # OOPS
            # Venturing to station. Clear coat rack first.
            angle = 217 # Was 215
            power = 1  # Was 0.8
            distance = 50
            self.autoFaceAngle = 120
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
                                                                                   distance)  # failoutTime is optional
            if gotoNextAutonStep:
                self.autonStep += 1

        if self.autonStep == 121:   # cascade back to base and robot venturing to front of station
            self.autonCoralSpoonFed = True  # Get intake into station feed position
            angle = 200 # Was 215
            power = 0.85 #was 0.8
            distance = 100 # was 90
            self.autoFaceAngle = -40
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
                                                                                   distance)  # failoutTime is optional
            if gotoNextAutonStep:
                self.autonStep += 1

        if self.autonStep == 122:  # Autoseek to station for feeding for coral #2
            self.autonCoralSpoonFedSeek = True
            if self.coralIntakeFull:    # Fail out when IR sensor tripped from presence of coral in mouth.
                self.coreFunctionsInstance.autoDriveReset()
                self.autonCoralSpoonFedSeek = False     # Turn off autoseek
                self.autonStep = 125

                # if self.autonOption == 3:   # 3 piece.
                #     self.autonStep = 125
                # elif self.autonOption == 2: # same 3 piece for now.
                #     self.autonStep = 125
                # else:   # same 3 piece for now.
                #     self.autonStep = 125


        if self.autonStep == 125:
            angle = 20  # was 25
            power = 0.9  # Was 0.8
            distance = 103  # was 115
            self.autoFaceAngle = 60
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
                                                                                   distance)  # failoutTime is optional
            if gotoNextAutonStep:
                self.autonCoralSpoonFed = False
                self.autonCoral4DropOff = True  # Self resetting in a one off
                self.autonStep += 1

            # if self.thresholdDistanceAutoDrive_Coral2 > distanceLeft > 0: # Lift cascade once close enough
            #     if self.newAutonCascade:
            #         self.autonCoralSpoonFed = False
            #         self.autonCoral4DropOff = True
            #         self.newAutonCascade = False  # Restricting self.autonCoral4DropOff to 1 cycle so reverse ejection during lift not repeated
            # if gotoNextAutonStep:
            #     self.newAutonCascade = True  # Resetting
            #     self.autonStep += 1

        if self.autonStep == 126:    # Autoseek activated for coral #2 dropoff
            if self.teamBlue:
                self.limelight2Pipeline = self.coral2R_Pos1_Blue
            else:
                self.limelight2Pipeline = self.coral2R_Pos1_Red

            self.newAutonCascade = True  # Resetting
            self.coreFunctionsInstance.autoDriveReset()

            self.autonRightCoatRackSeek = True  # Should activate proper pipeline

            self.autonStep += 1

        if self.autonStep == 127:   # Eject coral once basket in position.
            if self.coralTilt_Enc.getPosition() > self.coralTilt4 - self.setpointTriggerZoneFine:
                self.autonCoralEjection = True  # Self resetting
                self.autonStep += 1

        if self.autonStep == 128:   # Allow enough time for ejection before moving on to next step.
            timesUp = self.coralEjectionTimer.autoTimer(0.5)
            if timesUp:
                self.coreFunctionsInstance.autoDriveReset()
                self.autonRightCoatRackSeek = False  # Turning off autoseek so variable can be reset.
                self.autonStep = 130

        if self.autonStep == 130:   # cascade back to base and robot moving on to next target.
            self.autonCascadeDownNReset()   # Returning cascade to base.
            self.autonCoralSpoonFed = True
            # Venturing to station.
            angle = 210 #was 215
            power = 0.85 # Was 0.8
            distance = 80
            self.autoFaceAngle = -40 # was 310
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
                                                                                   distance)  # failoutTime is optional
            if gotoNextAutonStep:
                self.autonStep += 1

        if self.autonStep == 131:  # Autoseek to station for feeding for coral #3
            self.autonCoralSpoonFedSeek = True
            if self.coralIntakeFull:  # Fail out when IR sensor tripped from presence of coral in mouth.
                self.coreFunctionsInstance.autoDriveReset()
                self.autonCoralSpoonFedSeek = False  # Turn off autoseek
                self.autonStep = 135

        if self.autonStep == 135:   # Start of autodrive to coral 3 drop off
            angle = 20
            power = 0.9  # Was 0.8
            distance = 103
            self.autoFaceAngle = 60
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
                                                                                   distance)  # failoutTime is optional
            if gotoNextAutonStep:
                self.autonCoralSpoonFed = False
                self.autonCoral4DropOff = True  # Self resetting in a one off
                self.autonStep += 1

            # if self.thresholdDistanceAutoDrive_Coral3 > distanceLeft > 0: # Lift cascade once close enough
            #     if self.newAutonCascade:
            #         self.autonCoralSpoonFed = False
            #         self.autonCoral4DropOff = True
            #         self.newAutonCascade = False  # Restricting self.autonCoral4DropOff to 1 cycle so reverse ejection during lift not repeated
            # if gotoNextAutonStep:
            #     self.newAutonCascade = True  # Resetting
            #     self.autonStep += 1

        if self.autonStep == 136:  # Autoseek activated for coral #3 dropoff
            if self.teamBlue:
                self.limelight2Pipeline = self.coral3L_Pos1_Blue
            else:
                self.limelight2Pipeline = self.coral3L_Pos1_Red

            self.newAutonCascade = True  # Resetting
            self.coreFunctionsInstance.autoDriveReset()

            self.autonLeftCoatRackSeek = True  # Should activate proper pipeline

            self.autonStep += 1

        if self.autonStep == 137:   # Eject coral once basket in position.
            if self.coralTilt_Enc.getPosition() > self.coralTilt4 - self.setpointTriggerZoneFine:
                self.autonCoralEjection = True  # Self resetting
                self.autonStep += 1

        if self.autonStep == 138:   # Allow enough time for ejection before moving on to next step.
            timesUp = self.coralEjectionTimer.autoTimer(0.5)
            if timesUp:
                self.coreFunctionsInstance.autoDriveReset()
                self.autonLeftCoatRackSeek = False  # Turning off autoseek so variable can be reset.
                self.autonStep = 140


        if self.autonStep == 140:  # cascade back to base and robot moving on to next target.
            self.autonCascadeDownNReset() # Returning cascade to base.
            self.autonCoralSpoonFed = True
            # Venturing to station.
            angle = 210 #was 215
            power = 0.85  # Was 0.3
            distance = 80
            self.autoFaceAngle = -40  # was 310
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
                                                                                   distance)  # failoutTime is optional
            if gotoNextAutonStep:
                self.autonStep += 1

        if self.autonStep == 141:  # Autoseek to station for feeding for coral #3
            self.autonCoralSpoonFedSeek = True
            if self.coralIntakeFull:  # Fail out when IR sensor tripped from presence of coral in mouth.
                self.coreFunctionsInstance.autoDriveReset()
                self.autonCoralSpoonFedSeek = False  # Turn off autoseek
                self.autonStep += 1

        if self.autonStep == 142:
            self.coreFunctionsInstance.stopDriving()

        # Position 2

        # if self.autonStep == 210:
        #     angle = 180
        #     power = 0.8
        #     distance = 60
        #     self.autoFaceAngle = 180
        #     distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
        #                                                                            distance)  # failoutTime is optional
        #     if gotoNextAutonStep:
        #         self.autonStep += 1
        #
        # if self.autonStep == 211:
        #     self.coreFunctionsInstance.stopDriving()

        if self.autonStep == 210:
            angle = 180
            power = 0.8
            distance = 6
            self.autoFaceAngle = 180
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
                                                                                   distance)  # failoutTime is optional
            if gotoNextAutonStep:
                self.autonStep += 1

        if self.autonStep == 211:    # Autoseek activated for coral #1 dropoff
            if self.teamBlue:
                self.limelight2Pipeline = self.coral1R_Pos2_Blue
            else:
                self.limelight2Pipeline = self.coral1R_Pos2_Red
            self.autonRightCoatRackSeek = True  # Should activate proper pipeline
            # In order to use distance left as a trigger, need to have an autodrive step prior or will crash.
            if self.thresholdDistanceAutoSeekPos2 > self.autoSeekNStopDistanceLeftInInches > 0: # Lift cascade once close enough
                self.autonCoral4DropOff = True
                self.autonStep += 1

        if self.autonStep == 212:   # Eject coral once basket in position.
            # self.coreFunctionsInstance.ledBlink(True, 0.25, 0.25)
            if self.coralTilt_Enc.getPosition() > self.coralTilt4 - self.setpointTriggerZoneFine:
                self.autonCoralEjection = True  # Self resetting
                self.autonStep += 1

        if self.autonStep == 213:   # Allow enough time for ejection before moving on to next step.
            timesUp = self.coralEjectionTimer.autoTimer(0.5)
            if timesUp:
                self.coreFunctionsInstance.autoDriveReset()
                self.autonStep += 1

        if self.autonStep == 214:   # cascade back to base and robot moving on to next target.
            # self.coreFunctionsInstance.ledBlink(False)    # Turning off LED blink
            self.autonRightCoatRackSeek = False  # Turning off autoseek so variable can be reset.
            self.autonCascadeDownNReset() # OOPS
            timesUp = self.delayTimer.autoTimer(1.5)    # was 1
            if timesUp:
                self.autonStep += 1

        if self.autonStep == 215: # strafing left for algae boot
            angle = 90
            power = 0.5  # Was 0.3
            distance = 14
            self.autoFaceAngle = 180
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
                                                                                   distance)  # failoutTime is optional
            if gotoNextAutonStep:
                self.autonStep += 1

        if self.autonStep == 216: # booting algae 1
            self.autonAlgaeBoot23 = True
            if self.cascadeR_Enc.getPosition() > self.cascadeAlgae23Boot - self.setpointTriggerZoneFine:
                self.autonStep += 1

            # if self.autonOption == 1 or self.autonOption == 3:
            #     self.autonAlgaeBoot34 = True
            #     if self.cascadeR_Enc.getPosition() > self.cascadeAlgae34Boot - self.setpointTriggerZoneFine:
            #         self.autonStep += 1
            # elif self.autonOption == 2:
            #     self.autonAlgaeBoot23 = True
            #     if self.cascadeR_Enc.getPosition() > self.cascadeAlgae23Boot - self.setpointTriggerZoneFine:
            #         self.autonStep += 1

        if self.autonStep == 217:
            timesUp = self.delayTimer.autoTimer(2.5) #was 1
            if timesUp:
                self.postAlgaeBootCascadeReset = True   # Lowering and resetting cascade
                self.autonStep += 1

        if self.autonStep == 218:   # Driving back
            self.autonAlgaeBoot23 = False
            self.autonAlgaeBoot34 = False
            angle = 0
            power = 0.5  # Was 0.3
            distance = 12 #was 24, return to 24 if reactivating other options ##OOPS
            self.autoFaceAngle = 180
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
                                                                                   distance)  # failoutTime is optional
            if gotoNextAutonStep:
                self.autonStep += 1

        if self.autonStep == 219:   # Fork to other options
            self.coreFunctionsInstance.stopDriving()

            # if self.autonOption == 1:
            #     self.autonStep = 220
            # elif self.autonOption == 3:
            #     self.autonStep = 230
            # else:
            #     self.coreFunctionsInstance.stopDriving()


        # algae boot from far center and far right
        if self.autonStep == 220:
            angle = 105
            power = 0.5  # Was 0.3
            distance = 80
            self.autoFaceAngle = 240
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
                                                                                   distance)  # failoutTime is optional
            if gotoNextAutonStep:
                self.autonStep += 1

        if self.autonStep == 221:
            self.autonAlgaeBootSeek = True  # Should activate proper pipeline
            self.limelight2Pipeline = self.algaeBootPipeline
            currentVelocity = self.coreFunctionsInstance.currentVelocity()  # This triggering method is good when there is no rush to do what's next.
            timesUp = self.delayTimer.autoTimer(0.5)    # Time delay before sensing for motor stop since at start velocity also 0.
            if timesUp:
                if currentVelocity == 0:
                    self.coreFunctionsInstance.autoDriveReset()
                    self.autonStep += 1

        if self.autonStep == 222:
            self.autonAlgaeBoot34 = True
            if self.cascadeR_Enc.getPosition() > self.cascadeAlgae34Boot - self.setpointTriggerZoneFine:
                self.autonStep += 1

        if self.autonStep == 223:
            self.autonAlgaeBoot34 = False
            angle = 60
            power = 0.5  # Was 0.3
            distance = 24
            self.autoFaceAngle = 240
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
                                                                                   distance)  # failoutTime is optional
            if gotoNextAutonStep:
                self.autonStep += 1

        if self.autonStep == 224:
            self.coreFunctionsInstance.stopDriving()

        # algae boot from far center and far left
        if self.autonStep == 230:
            angle = 255
            power = 0.5  # Was 0.3
            distance = 80
            self.autoFaceAngle = 150
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
                                                                                   distance)  # failoutTime is optional
            if gotoNextAutonStep:
                self.autonStep += 1

        if self.autonStep == 231:
            self.autonAlgaeBootSeek = True  # Should activate proper pipeline
            self.limelight2Pipeline = self.algaeBootPipeline
            currentVelocity = self.coreFunctionsInstance.currentVelocity()  # This triggering method is good when there is no rush to do what's next.
            timesUp = self.delayTimer.autoTimer(0.5)  # Time delay before sensing for motor stop since at start velocity also 0.
            if timesUp:
                if currentVelocity == 0:
                    self.coreFunctionsInstance.autoDriveReset()
                    self.autonStep += 1

        if self.autonStep == 232:
            self.autonAlgaeBoot34 = True
            if self.cascadeR_Enc.getPosition() > self.cascadeAlgae34Boot - self.setpointTriggerZoneFine:
                self.autonStep += 1

        if self.autonStep == 233:
            self.autonAlgaeBoot34 = False
            angle = 320
            power = 0.5  # Was 0.3
            distance = 24
            self.autoFaceAngle = 150
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
                                                                                   distance)  # failoutTime is optional
            if gotoNextAutonStep:
                self.autonStep += 1

        if self.autonStep == 234:
            self.coreFunctionsInstance.stopDriving()

        # Position 3
        if self.autonStep == 315: # the power is a % of speedLimitMultiplier. Can increase self.speedLimitMultiplier to speed up.
            angle = 180
            power = 0.8 # Was 0.85
            distance = 75  # was 80
            self.autoFaceAngle = -120 #was -130
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power, distance) # failoutTime is optional
            if gotoNextAutonStep:
                # self.cascadeConfigOnTheFly(-0.22, 0.30) #min was -0.2, Max was 0.35
                self.autonCoral4DropOff = True
                self.autonStep += 1

        if self.autonStep == 316:    # Autoseek activated for coral #1 dropoff / right coat rack
            if self.teamBlue:
                self.limelight2Pipeline = self.coral1R_Pos3_Blue
            else:
                self.limelight2Pipeline = self.coral1R_Pos3_Red

            self.autonRightCoatRackSeek = True  # Should activate proper pipeline

            self.autonStep += 1

        if self.autonStep == 317:   # Eject coral once basket in position.
            if self.coralTilt_Enc.getPosition() > self.coralTilt4Auton - self.setpointTriggerZoneFine:
                self.autonCoralEjection = True  # Self resetting
                self.autonStep += 1

        if self.autonStep == 318:   # Allow enough time for ejection before moving on to next step.
            timesUp = self.coralEjectionTimer.autoTimer(0.5)
            if timesUp:
                self.coreFunctionsInstance.autoDriveReset()
                self.autonRightCoatRackSeek = False  # Turning off autoseek so variable can be reset.
                self.autonStep = 320

        if self.autonStep == 320:   # cascade back to base and robot moving on to next target.
            self.autonCascadeDownNReset()   # OOPS
            # Venturing to station. Clear coat rack first.
            angle = 143 # was 150
            power = 1 # Was 0.8
            distance = 70 #was 40
            self.autoFaceAngle = -120
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
                                                                                   distance)  # failoutTime is optional
            if gotoNextAutonStep:
                self.autonStep += 1

        if self.autonStep == 321:   # cascade back to base and robot moving on to next target.
            # Venturing to front of station
            self.autonCoralSpoonFed = True  # Get intake into station feed position
            angle = 175 #was 165
            power = 0.95  # Was 0.85
            distance = 70 # was 76
            self.autoFaceAngle = -130
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
                                                                                   distance)  # failoutTime is optional
            if gotoNextAutonStep:
                self.autonStep += 1

        if self.autonStep == 322:  # Autoseek to station for feeding for coral #2
            self.autonCoralSpoonFedSeek = True
            if self.coralIntakeFull:    # Fail out when IR sensor tripped from presence of coral in mouth.
                self.coreFunctionsInstance.autoDriveReset()
                self.autonCoralSpoonFedSeek = False     # Turn off autoseek
                self.autonStep = 325

                # if self.autonOption == 3:   # 3 piece.
                #     self.autonStep = 325
                # elif self.autonOption == 2: # same 3 piece for now.
                #     self.autonStep = 325
                # else:   # same 3 piece for now.
                #     self.autonStep = 325

        if self.autonStep == 325:   # Setup drive for CoatRack
            angle = 335 #was 340
            power = 0.8  # Was 0.9
            distance = 103 #was 105
            self.autoFaceAngle = -60 #was -75
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
                                                                                   distance)  # failoutTime is optional
            if gotoNextAutonStep:
                # self.cascadeConfigOnTheFly(-0.22, 0.26) #min was -0.2 and max was 0.32 # Default is min = 0.18, max = 0.35
                self.autonCoralSpoonFed = False
                self.autonCoral4DropOff = True  # Self resetting in a one off
                self.autonStep += 1

        if self.autonStep == 326:    # Autoseek activated for coral #2 dropoff
            if self.teamBlue:
                self.limelight2Pipeline = self.coral2R_Pos3_Blue
            else:
                self.limelight2Pipeline = self.coral2R_Pos3_Red
            self.coreFunctionsInstance.autoDriveReset()

            self.autonRightCoatRackSeek = True
            self.autonStep += 1

        if self.autonStep == 327:   # Eject coral once basket in position.
            if self.coralTilt_Enc.getPosition() > self.coralTilt4Auton - self.setpointTriggerZoneFine:
                self.autonCoralEjection = True  # Self resetting
                self.autonStep += 1

        if self.autonStep == 328:   # Allow enough time for ejection before moving on to next step.
            timesUp = self.coralEjectionTimer.autoTimer(0.5)
            if timesUp:
                # self.cascadeConfigOnTheFly(-0.18, self.cascadeOutputCoralMax)  # Default is min = 0.18, max = 0.35. Readying for teleop
                self.coreFunctionsInstance.autoDriveReset()
                self.autonRightCoatRackSeek = False  # Turning off autoseek so variable can be reset.
                self.autonStep = 330

        if self.autonStep == 330:
            self.autonCascadeDownNReset()
            self.coreFunctionsInstance.stopDriving()

        # if self.autonStep == 330:   # cascade back to base and robot moving on to next target.
        #     self.autonCascadeDownNReset() # Returning cascade to base.
        #     self.autonCoralSpoonFed = True
        #     # Venturing to station.
        #     angle = 150
        #     power = 0.95  # Was 0.8
        #     distance = 80
        #     self.autoFaceAngle = -130
        #     distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
        #                                                                            distance)  # failoutTime is optional
        #     if gotoNextAutonStep:
        #         self.autonStep += 1

        if self.autonStep == 331:  # Autoseek to station for feeding for coral #3
            self.autonCoralSpoonFedSeek = True
            if self.coralIntakeFull:  # Fail out when IR sensor tripped from presence of coral in mouth.
                self.coreFunctionsInstance.autoDriveReset()
                self.autonCoralSpoonFedSeek = False  # Turn off autoseek
                self.autonStep = 335

        if self.autonStep == 335:  # Start of autodrive to coral 3 drop off
            angle = 340 # Was 330
            power = 0.8  # Was 0.9
            distance = 93 # was 103
            self.autoFaceAngle = -60 #was -65
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
                                                                                   distance)  # failoutTime is optional
            if gotoNextAutonStep:
                # self.cascadeConfigOnTheFly(-0.22, 0.26) #min was 0.14 and max was 0.3  # Default is min = 0.18, max = 0.35
                self.autonCoralSpoonFed = False
                self.autonCoral4DropOff = True  # Self resetting in a one off
                self.autonStep += 1

        if self.autonStep == 336:  # Autoseek activated for coral #3 dropoff
            self.newAutonCascade = True
            if self.teamBlue:
                self.limelight2Pipeline = self.coral3L_Pos3_Blue
            else:
                self.limelight2Pipeline = self.coral3L_Pos3_Red
            self.coreFunctionsInstance.autoDriveReset()

            self.autonLeftCoatRackSeek = True
            self.autonStep += 1

        if self.autonStep == 337:   # Eject coral once basket in position.
            if self.coralTilt_Enc.getPosition() > self.coralTilt4Auton - self.setpointTriggerZoneFine:
                self.autonCoralEjection = True  # Self resetting
                self.autonStep += 1

        if self.autonStep == 338:   # Allow enough time for ejection before moving on to next step.
            timesUp = self.coralEjectionTimer.autoTimer(0.5)
            if timesUp:
                # self.cascadeConfigOnTheFly(-0.18, self.cascadeOutputCoralMax)  # Default is min = 0.18, max = 0.35. Readying for teleop
                self.coreFunctionsInstance.autoDriveReset()
                self.autonLeftCoatRackSeek = False  # Turning off autoseek so variable can be reset.
                self.autonStep = 340

        if self.autonStep == 340:  # cascade back to base and robot moving on to next target.
            self.autonCascadeDownNReset() # Returning cascade to base.
            self.autonCoralSpoonFed = True
            # Venturing to station.
            angle = 150
            power = 0.8  # Was 0.85
            distance = 80
            self.autoFaceAngle = -130
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
                                                                                   distance)  # failoutTime is optional
            if gotoNextAutonStep:
                self.autonStep += 1

        if self.autonStep == 341:  # Autoseek to station for feeding for coral #3
            self.autonCoralSpoonFedSeek = True
            if self.coralIntakeFull:  # Fail out when IR sensor tripped from presence of coral in mouth.
                self.coreFunctionsInstance.autoDriveReset()
                self.autonCoralSpoonFedSeek = False  # Turn off autoseek
                self.autonStep += 1

        if self.autonStep == 342:
            self.coreFunctionsInstance.stopDriving()

        if self.autonStep == 400:
            angle = 180
            power = 0.8
            distance = 60
            self.autoFaceAngle = 0
            distanceLeft, gotoNextAutonStep = self.coreFunctionsInstance.autoDrive(angle, power,
                                                                                   distance)  # failoutTime is optional
            if gotoNextAutonStep:
                self.autonStep += 1

        if self.autonStep == 401:
            self.coreFunctionsInstance.autoDriveReset()
            self.coreFunctionsInstance.stopDriving()


        # Gadget Input **************************************************************
        # Intake activation
        self.intakeControl()

        # Cascade activation
        self.cascadeControl()

        # Coral ejection onto CoatRack
        self.coralEjection()

        # autoSeekNStop activation
        self.autoSeekLimelight()

        # Activate to return robot to default
        self.robotDefault()

        # Actual code for actuating motors
        self.coreFunctionsInstance.swerveMasterControl(self.autonomous,
                                 self.speedLimitMultiplier, self.fieldCentric,
                                 self.autoFaceAngle)

        self.actualActuation()

        #   Disable rumble for auton. Possible conflict?
        self.coreFunctionsInstance.rumbleJoy0()
        self.coreFunctionsInstance.rumbleJoy1()

        #telemetry
        self.coreFunctionsInstance.driveTelemetry()
        self.tazMotorsTelemetry()

    def teleopInit(self):   # teleop
        """This function is run once each time the robot enters teleop mode."""
        # Essential defaults
        self.autonomous = False
        self.teleoperated = True
        self.autonTeleop_InitDefaults()
        
        # # In case not reset at end of auton, the cascade Min Max is reset as default
        # self.cascadeConfigOnTheFly(-0.18,
        #                            self.cascadeOutputCoralMax)  # Default is min = 0.18, max = 0.35. Readying for teleop

        if self.teleopFirst:    # This is the reason why after a new code upload, north is now front of the robot since enter teleop directly.
            self.gyroCompensation = 0
            self.autoFaceAngle = 0

    def teleopPeriodic(self):   # teleCode
        """This function is called periodically during operator control."""
        self.variableResets()   # variables setting to default prior to manipulation below

        self.readingSensors()  # Collecting DIO data

        # Driver Input (needs to be before Gadget input in order to be overwritten downstream) ***********************
        self.driverControl()

        # Gadget Input **************************************************************
        # Intake activation
        self.intakeControl()

        # Cascade activation
        self.cascadeControl()

        # Coral ejection onto CoatRack
        self.coralEjection()

        # autoSeekNStop activation by gadget
        self.autoSeekLimelight()

        # Activate to return robot to default
        self.robotDefault()

        # Put wrist and elbow back into base position if radio disconnects
        # self.emergencyReset()

        # Actual code for actuating motors
        # Swervedrive functional only if Robot not in Lockdown. This prevents a control conflict. Should be at end of TeleopPeriodic
        # Call for wheel lock is removed since not needed in 2025 game.
        # The rest of Driver control is here since part of swerveMasterControl logic

        self.coreFunctionsInstance.swerveMasterControl(self.autonomous,
                    self.speedLimitMultiplier, self.fieldCentric, self.autoFaceAngle)

        self.actualActuation()

        self.DIO_Telemetry()
        # Telemetry
        # self.coreFunctionsInstance.driveTelemetry()
        # self.coreFunctionsInstance.cancoderTelemetry()
        # self.coreFunctionsInstance.navxTelemetry()
        # self.coreFunctionsInstance.amperageTelemetry()
        # self.tazMotorsTelemetry()

        # self.coreFunctionsInstance.RyFromDataArrayPub.set(self.coreFunctionsInstance.dataArrayLimelightSub.get()[4])
        # self.coreFunctionsInstance.limelightTelemetry()  # Crashing
        # self.coreFunctionsInstance.limelight2Telemetry()

    def testInit(self):
        pass

    def testPeriodic(self):
        pass

    def disabledInit(self):
        self.autonomous = False
        self.teleoperated = False

    def disabledPeriodic(self): # disCode
        # Telemetry
        self.coreFunctionsInstance.driveTelemetry()
        # self.coreFunctionsInstance.navxTelemetry()    # This was crashing in disabled
        # self.coreFunctionsInstance.limelightTelemetry()   # Crashing
        # self.coreFunctionsInstance.limelight2Telemetry()
        # self.coreFunctionsInstance.RyFromDataArrayPub.set(self.coreFunctionsInstance.dataArrayLimelightSub.get()[4])
        self.tazMotorsTelemetry()

        self.readingAutonSwitches()
        self.readingSensors()
        self.DIO_Telemetry()
        # self.joystickTelemetry()




    # Functions ****************************************************************************************

    def actualActuation(self):
        self.coralIntake.set(self.coralIntakePower)
        self.algaeBoot.set(self.algaeBootPower)

        self.coralIntakeWrist_PID.setReference(self.coralIntakeWristSetpoint, SparkLowLevel.ControlType.kPosition)
        self.coralIntakeElbow_PID.setReference(self.coralIntakeElbowSetpoint, SparkLowLevel.ControlType.kPosition)
        self.coralTilt_PID.setReference(self.coralTiltSetpoint, SparkLowLevel.ControlType.kPosition)
        self.cascadeR_PID.setReference(self.cascadeSetpoint, SparkLowLevel.ControlType.kPosition)
        self.cascadeL_PID.setReference(self.cascadeSetpoint, SparkLowLevel.ControlType.kPosition)
        self.turtleneck_PID.setReference(self.turtleneckSetpoint, SparkLowLevel.ControlType.kPosition)
        self.coralEject_PID.setReference(self.coralEjectSetpoint, SparkLowLevel.ControlType.kPosition)
        self.climb_PID.setReference(self.climbSetpoint, SparkLowLevel.ControlType.kPosition)
        self.mouseTrap_PID.setReference(self.mouseTrapSetpoint, SparkLowLevel.ControlType.kPosition)

    # Essential robot specific constants and Primers (Prime variables to prevent crashes), some modified downstream
    # and remain as is between modes and not reset so to prevent sudden movements upon robot enabling
    def robotInitDefaults(self):       # tune
        #autoSeekAndStop tuning variables
        self.maxAutoSeekNStopSpeed = 0.63 #was 0.6 # do not touch
        self.virtualStickPower = 0.6   # Secondary power control behind speedLimitMultiplier, which it's a fraction of
        self.virtualStickPower2 = 0.5  # Secondary power control behind speedLimitMultiplier, which it's a fraction of
        self.emergencyBrakeDistance = 38  # in inches from station where emergency stop occurs
        self.emergencyBrakeDistance2 = 0 # in inches from coat rack where emergency stop occurs
        self.decelFudgeFactor = 1   # CANNOT BE 0! decelFudgeFactor range is 1-3. >1 is increasing braking.
        self.faceAngleFudgeFactor = 0   # Range is >= 0 in degrees. Increasing means overcompensating for lag to catch up
        self.faceAngleFudgeFactor2 = 0  # Range is >= 0 in degrees. Increasing means overcompensating for lag to catch up
        self.driveAngleFudgeFactor = 4  # In degrees, positive is to target more to the right.
        self.driveAngleFudgeFactor2 = 0  # In degrees, positive is to target more to the right.
        self.freshEmergency = True
        self.emergencyTransfer = False

        self.newAutonCascade = True
        # Following only for position 1 auton. Position 3 not need since cascade goes up when limelightAutoseek kicks in
        self.thresholdDistanceAutoDrive_Coral1 = 20  # in inches
        self.thresholdDistanceAutoDrive_Coral2 = 25  # in inches
        self.thresholdDistanceAutoDrive_Coral3 = 30  # was 25

        # As of 3-21-25, NO LONGER USED. Controlling Cascade elevation speed more in vogue now.
        self.thresholdDistanceAutoseek_Pos3Coral2 = 16  # in inches
        self.thresholdDistanceAutoseek_Pos3Coral3 = 22   # in inches - was 22
        self.thresholdDistanceAutoSeekPos2= 40  # in inches

        # Limelight2 Pipeline Tuning
        self.coral1R_Pos1_Blue = 7
        self.coral2R_Pos1_Blue = 8  # Was 7. 7 is neutral for right rack. Higher number = move more to right.
        self.coral3L_Pos1_Blue = 1  # Was 2.

        self.coral1R_Pos1_Red = 7
        self.coral2R_Pos1_Red = 7
        self.coral3L_Pos1_Red = 2

        self.coral1R_Pos3_Blue = 8 #was 7
        self.coral2R_Pos3_Blue = 7
        self.coral3L_Pos3_Blue = 2

        self.coral1R_Pos3_Red = 7   # Default for position 2
        self.coral2R_Pos3_Red = 7
        self.coral3L_Pos3_Red = 2

        self.coral1R_Pos2_Blue = 7
        self.coral1R_Pos2_Red = 8

        self.algaeBootPipeline = 4

        self.setpointTriggerZone = 2
        self.setpointTriggerZoneFine = 1
        self.setpointTriggerZoneLarge = 10
        self.newPipeline = True
        self.newPipeline2 = True
        self.incompleteJourney = False
        self.coralTroughDrop = False
        self.coral4Trough = False

        self.coralIntakePower = 0   # Needed as a primer since after coral pickup intake motor allowed have holding power and not 0
        self.coralIntakePowerGrab = 0.85    # Was 0.8
        self.coralIntakePowerTrough = -0.15  # Was 0.5
        self.coralIntakePowerTroughHold = 0.25 # Was 0.4
        self.coralIntakePowerAlgae = -0.7 # was -0.5
        self.coralIntakePowerAlgaeHold = -0.05
        self.coralIntakePowerAlgaeProcessor = 0.6 #was 0.3
        self.coralIntakePowerHold = 0.08
        self.coralIntakePowerTransfer = -0.25
        self.coralSpoonFedHailMaryMode = False

        self.coralIntakeStep = 1    # Should not be in autonTeleop_InitDefaults so variable caries over to teleop after auton
        self.coralIntakeCycleComplete = True    # Should not be in autonTeleop_InitDefaults so variable caries over to teleop after auton
        self.autonCoralSpoonFed = False # positions intake for station coral pickup when True. Should not be in autonTeleop_InitDefaults

        self.coralIntakeWristTransfer = 0   # Starting point
        self.coralIntakeWristSpoonFed = 13.5 #was 12 #was 11.6  # pick up coral at station
        self.coralIntakeWristSpoonFed_HailMary = 11.9
        self.coralIntakeWristPreTrough = 15
        self.coralIntakeWristTrough = 22.5
        self.coralIntakeWristAlgae= 13 #was 15 (too tight), 12.5 # was 11.4
        self.coralIntakeWristPickup = 16 #was 14.5 # was 12.2 # Possible update
        self.coralIntakeWristClimb =  13.5 #was 12.6
        self.coralIntakeWristClimbBalance = 12.8
        self.coralIntakeWristSetpoint = self.coralIntakeWristTransfer

        self.coralIntakeElbowTransfer = 0  # Starting point
        self.coralIntakeElbowSafety = 6  # Possible update # Safe position when coral held and cascade not in base
        self.coralIntakeElbowSpoonFed = 11.05 #was 17.4 # pick up coral at station
        self.coralIntakeElbowSpoonFed_HailMary = 25.2  # was 17.4 # pick up coral at station
        self.coralIntakeElbowTrough = 29.6
        self.coralIntakeElbowAlgae = 65
        self.coralIntakeElbowPickup = 103  # Possible update
        self.coralIntakeElbowClimb = 16.4
        self.coralIntakeElbowClimbBalance = 40.9
        self.coralIntakeElbowSetpoint = self.coralIntakeElbowTransfer

        self.coralTiltBase = 0
        self.coralTilt123 = 6 #was 6
        self.coralTilt123_HailMary = 5.5 #was 5
        self.coralTilt4 = 8 #was 7
        self.coralTilt4Auton = 7.5
        self.coralTilt4_HailMary = 6.5 # was 7
        self.coralTiltClimb = 11.5 #was 11.2
        self.coralTiltSetpoint = self.coralTiltBase
        self.coralTiltHailMaryMode = False

        self.cascadeBase = 0
        self.cascade1 = 6 #was 8
        self.cascade2 = 10.5 # was 11
        self.cascade3 = 19.5 #was 24
        self.cascade4 = 33.5 #34 is absolute max
        self.cascadeMax = 33.5 #was 35    # Need be verified *** OOPS
        self.cascadeAlgae23PreBoot = 5    # Setup for algae 23 Boot
        self.cascadeAlgae23Boot = 8 #was 10   # Algae 23 Boot
        self.cascadeAlgae34PreBoot = 13  # Setup for algae 34 Boot
        self.cascadeAlgae34Boot = 18  # Algae 34 Boot
        self.cascadeSetpoint = self.cascadeBase
        self.cascade2Active = False
        self.cascade3Active = False
        self.cascade4Active = False
        self.postAlgaeBootCascadeReset = False

        self.turtleneckIn = 0
        self.turtleneckOut = 70 # was 77
        self.turtleneckBoot = 96 # was 86
        self.turtleneckClimb = 65
        self.turtleneckMax = self.turtleneckBoot    # For now. May need to extend more.
        self.turtleneckSetpoint = self.turtleneckIn

        # self.coralEjectPowerPreset = 0.25
        self.coralEjectSetpoint = 0 # Possible reset encoder to zero on every reboot of robot init
        self.coralEjectIncrement = 12 #was 10

        self.algaeBootPowerPreset = 0.25
        self.cascadeAlgaePreTarget = self.cascadeBase   # Just a primer/placeholder
        self.cascadeAlgaeTarget = self.cascadeBase  # Just a primer/placeholder
        self.algaeBootStep = 1
        self.algaeBootCycleComplete = True
        self.algaeIntake = False

        self.climbBase = 0
        self.climbUpMax = 100
        self.climbSetpoint = self.climbBase

        self.mouseTrapCoralBlocker = 0
        self.mouseTrapClosed = 7 #was 3.3
        self.mouseTrapOpen = 30
        self.mouseTrapSetpoint = self.mouseTrapCoralBlocker
        self.newClimb = True
    
    def autonTeleop_InitDefaults(self):    # Primers / Universal defaults re-establishing
        self.fieldCentric = True

        self.speedLimitMultiplier = 0.6 # Max power no greater than 0.92. Needs headroom for compensation adjustments.
        self.speedLimitMultiplierOG = self.speedLimitMultiplier
        self.newPipeline = True  # Redundant
        self.newPipeline2 = True    # Redundant
        self.incompleteJourney = False  # Redundant
        self.autoSeekNStopDistanceLeftInInches = 0 # Pure primer. *** OOPS. Was disabled and auton was functioning. However, may be needed to prevent crash?
        self.newAutonCascade = True

        self.algaeBootStep = 1  # Redundant
        self.algaeBootCycleComplete = True  # Redundant
        self.algaeIntake = False    # Redundant
        self.coralTroughDrop = False   # Redundant
        self.coral4Trough = False  # Redundant

        # Variables for activating functions called in autonPeriodic by "= True" in steps
        # Actuation
        self.autonCoral4DropOff = False
        self.autonCoralEjection = False
        self.autonAlgaeBoot23 = False
        self.autonAlgaeBoot34 = False
        self.autonCascadeDefault = False
        self.autonRobotDefault = False

        # Autoseek
        self.autonCoralSpoonFedSeek = False
        self.autonAlgaeBootSeek = False
        self.autonLeftCoatRackSeek = False
        self.autonRightCoatRackSeek = False

        self.cascade2Active = False
        self.cascade3Active = False
        self.cascade4Active = False
        self.coralTiltHailMaryMode = False
        self.coralSpoonFedHailMaryMode = False
        self.postAlgaeBootCascadeReset = False

    def variableResets(self):
        self.coralIntakePower = 0

    def driverControl(self):
        # This will allow driver to manually reset "North" of robot. This is not persistent with code re-deployment
        if self.joystick0.getBackButtonPressed() or self.joystick0.getStartButtonPressed():  # Updating gyroCompensation to re-establish North
            self.fieldCentric = True  # Reactivates field centric incase it is the culprit
            resetNorthActivated = True
        else:
            resetNorthActivated = False
        # Need be early in Periodic since the gyro values are needed downstream.
        self.coreFunctionsInstance.gyroAngleAdjustments(self.autonomous, self.gyroCompensation, resetNorthActivated)

        # Speed control by driver
        if self.joystick0.getYButtonPressed():
            self.speedLimitMultiplier = 0.90
            self.speedLimitMultiplierOG = self.speedLimitMultiplier  # To record current chozen speed
        if self.joystick0.getBButtonPressed():
            self.speedLimitMultiplier = 0.6
            self.speedLimitMultiplierOG = self.speedLimitMultiplier
        if self.joystick0.getAButtonPressed():
            self.speedLimitMultiplier = 0.15
            self.speedLimitMultiplierOG = self.speedLimitMultiplier

        # Driver activating preset positions for climb
        if self.joystick0.getLeftBumper() and self.joystick0.getRightBumper():
            # if self.newClimb:
            #     self.mouseTrap_Enc.setPosition(0)
            #     self.newClimb = False
            self.coralIntakeWristSetpoint = self.coralIntakeWristClimb
            self.coralIntakeElbowSetpoint = self.coralIntakeElbowClimb
            self.coralTiltSetpoint = self.coralTiltClimb
            self.mouseTrapSetpoint = self.mouseTrapOpen
            # self.coreFunctionsInstance.ledLimelightPub2.set(1)
            # self.coreFunctionsInstance.pipelineLimelightPub2.set(3)
            # most effective way is to turn off Green LED since pipelines all used up.

        if self.joystick0.getPOV() == 180 and self.joystick0.getRightBumper():
            self.coralIntakeWristSetpoint = self.coralIntakeWristTransfer
            self.coralIntakeElbowSetpoint = self.coralIntakeElbowTransfer
            self.coralTiltSetpoint = self.coralTiltBase
            self.mouseTrapSetpoint = self.mouseTrapCoralBlocker
            self.newClimb = True

        ## Test Code
        # if self.joystick0.getPOV() == 90:
        #     self.mouseTrapSetpoint = self.mouseTrapOpen
        # elif self.joystick0.getPOV() == 270:
        #     self.mouseTrapSetpoint = self.mouseTrapCoralBlocker

        if (abs(self.joystick0.getLeftX()) > self.joystickDeadzone or abs(self.joystick0.getLeftY()) > self.joystickDeadzone) and self.joystick0.getXButton():
            self.fieldCentric = False   # Driver able to turn off fieldCentric for FPV strafing
            self.speedLimitMultiplier = 0.10 # was 0.15
        elif self.joystick0.getRightTriggerAxis() > 0.5:
            self.speedLimitMultiplier = 0.15 # was 0.20
        else:
            self.fieldCentric = True    # fieldCentric return to default setting of "On"
            self.speedLimitMultiplier = self.speedLimitMultiplierOG # Returns speed back to last recorded

        # Driver controlled climb, allowed only if wrist and tilt in climb preset positions.
        if (((self.coralIntakeWristClimb + self.setpointTriggerZone * 1.5) > self.coralIntakeWrist_Enc.getPosition() > (self.coralIntakeWristClimb - self.setpointTriggerZone * 1.5)) and
                (self.coralTilt_Enc.getPosition() > (self.coralTiltClimb - self.setpointTriggerZone * 1.5))):
            # mouseTrap starts from open to close between trigger 0 to midpoint. Beyond midpoint, it remains closed.
            if self.joystick0.getLeftTriggerAxis() <= 0.5:
                self.mouseTrapSetpoint = self.mouseTrapOpen - ((self.joystick0.getLeftTriggerAxis() * 2) * (self.mouseTrapOpen - self.mouseTrapClosed))
            elif self.joystick0.getLeftTriggerAxis() > 0.5:
                self.mouseTrapSetpoint = self.mouseTrapClosed

            # # Climb happens between trigger midpoint and max and mouseTrap in closed position. Balancing config now baked in here.
            if (self.mouseTrapClosed - self.setpointTriggerZone) < self.mouseTrap_Enc.getPosition() < (self.mouseTrapClosed + self.setpointTriggerZone):
                if self.joystick0.getLeftTriggerAxis() <= 0.5:
                    self.climbSetpoint = self.climbBase
                    self.coralIntakeElbowSetpoint = self.coralIntakeElbowClimb
                    self.coralIntakeWristSetpoint = self.coralIntakeWristClimb
                elif self.joystick0.getLeftTriggerAxis() > 0.5:
                    self.climbSetpoint = (self.joystick0.getLeftTriggerAxis() - 0.5) * 2 * self.climbUpMax
                    self.coralIntakeElbowSetpoint = self.coralIntakeElbowClimbBalance
                    self.coralIntakeWristSetpoint = self.coralIntakeWristClimbBalance

        ## The following is no longer needed since already baked in above.
        # if self.climb_Enc.getPosition() > (self.climbUpMax - self.setpointTriggerZone * 2):
        #     self.coralIntakeElbowSetpoint = self.coralIntakeElbowClimbBalance
        #     self.coralIntakeWristSetpoint = self.coralIntakeWristClimbBalance
        # elif (self.climbUpMax - self.setpointTriggerZone * 2) >= self.climb_Enc.getPosition() > self.setpointTriggerZoneLarge:
        #     # This is included in the main loop and the "> self.setpointTriggerZoneLarge" prevents Elbow and Wrist from going into in climb position at start.
        #     self.coralIntakeElbowSetpoint = self.coralIntakeElbowClimb
        #     self.coralIntakeWristSetpoint = self.coralIntakeWristClimb

        # Reading driver control inputs (Overwritten downstream so automated Gadget actions override Driver actions)
        self.coreFunctionsInstance.readingJoysticks(self.speedLimitMultiplier)

    def intakeControl(self):    # Shared intake control between Auton and Teleop
        # If in auton, need be called continuously for proper function, not just called in step.
        # Coral intake from ground and station

        if self.joystick1.getPOV() == 180:  # Turning on coral hold for trough deposit
            self.coral4Trough = True

        if self.joystick1.getLeftTriggerAxis() >= 0.8 or self.joystick1.getLeftBumper() or self.autonCoralSpoonFed:
            self.cascadeSetpoint = self.cascadeBase # To make sure cascade back in base position.
            self.coralTiltSetpoint = self.coralTiltBase
            self.turtleneckSetpoint = self.turtleneckIn

            if self.coralIntakeStep == 1:   # Getting into position and running intake. This consolidated 2 previous steps so can change mind on intake mode midstream
                self.coralIntakeCycleComplete = False
                if self.joystick1.getLeftTriggerAxis() >= 0.8:
                    self.coralIntakeWristSetpoint = self.coralIntakeWristPickup
                    self.coralIntakeElbowSetpoint = self.coralIntakeElbowPickup
                    if self.coralIntakeElbow_Enc.getPosition() > self.coralIntakeElbowPickup - self.setpointTriggerZoneLarge:  # Intake not start spinning till arm near down endpoint
                        self.coralIntakePower = self.coralIntakePowerGrab   # Start intake motor and wait for IR sensor occlusion
                elif self.joystick1.getLeftBumper() or self.autonCoralSpoonFed:
                    lockedOn = self.coreFunctionsInstance.targetAcquired(limelightNum=1)  # If target present and identified
                    if lockedOn:
                        self.coreFunctionsInstance.ledOn()  # Turn LED on
                    else:
                        self.coreFunctionsInstance.ledOff()  # Turn LED off

                    if self.coralSpoonFedHailMaryMode:
                        self.coralIntakeWristSetpoint = self.coralIntakeWristSpoonFed_HailMary
                        self.coralIntakeElbowSetpoint = self.coralIntakeElbowSpoonFed_HailMary
                    else:
                        self.coralIntakeWristSetpoint = self.coralIntakeWristSpoonFed
                        self.coralIntakeElbowSetpoint = self.coralIntakeElbowSpoonFed

                    if self.coralIntakeElbow_Enc.getPosition() > self.coralIntakeElbowSpoonFed - self.setpointTriggerZoneLarge:  # Intake not start spinning till arm near position
                        self.coralIntakePower = self.coralIntakePowerGrab   # Start intake motor and wait for IR sensor occlusion

                if self.coralIntakeFull:    # IR sensor triggered
                    self.coralIntakeStep += 1

            if self.coralIntakeStep == 2:  # After coral is catched, continue intake motor for additional time.
                self.coralIntakePower = self.coralIntakePowerGrab # Continue to intake for additional set time
                if self.teleoperated:
                    coralCaptureTime = 0.35 # was 0.5
                else:
                    coralCaptureTime = 0.15  # was 0.5
                timesUp = self.coralIntakeTimer.autoTimer(coralCaptureTime)
                if timesUp:
                    self.coreFunctionsInstance.ledOff()  # Turn LED off
                    self.coralIntakeStep += 1

            if self.coralIntakeStep == 3:   # Fork for transfer to basket or for holding for trough deposit
                if self.coral4Trough:
                    self.coralIntakeStep = 10
                else:
                    self.coralIntakeStep += 1

            if self.coralIntakeStep == 4:   # Return to transfer position while holding coral at grab power
                self.coralIntakePower = self.coralIntakePowerGrab
                self.coralIntakeWristSetpoint = self.coralIntakeWristTransfer
                self.coralIntakeElbowSetpoint = self.coralIntakeElbowTransfer
                self.coralIntakeStep += 1

            if self.coralIntakeStep == 5:  # Blink Led when Elbow near or at transfer position
                self.coralIntakePower = self.coralIntakePowerGrab
                if self.coralIntakeElbow_Enc.getPosition() < self.coralIntakeElbowTransfer + self.setpointTriggerZoneFine:
                    self.coralIntakePower = self.coralIntakePowerTransfer  # The actual transfer drop off
                    self.coreFunctionsInstance.rumbleJoy1(rumbleStrength=0.5)
                    # self.coreFunctionsInstance.ledBlink(True,0.25, 0.25)  # Once blinking, Gadget can let go of button or trigger
                    self.coralIntakeCycleComplete = True
                    timesUp = self.coralTransferTimer.autoTimer(1)    # was 1 sec
                    if timesUp:
                        self.coralEjectSetpoint = self.coralEjectSetpoint - self.coralEjectIncrement
                        self.coralIntakeStep += 1

            if self.coralIntakeStep == 6:
                self.coralIntakePower = 0

            if self.coralIntakeStep == 10:  # Holding coral for trough drop.
                self.coralIntakePower = self.coralIntakePowerGrab
                self.coralIntakeWristSetpoint = self.coralIntakeWristPreTrough
                self.coralIntakeElbowSetpoint = self.coralIntakeElbowTrough
                if self.coralIntakeElbow_Enc.getPosition() > self.coralIntakeElbowTrough - self.setpointTriggerZone:
                    self.coralIntakeStep += 1

            if self.coralIntakeStep == 11:
                self.coralIntakePower = self.coralIntakePowerHold
                self.coralIntakeCycleComplete = True
                self.coreFunctionsInstance.rumbleJoy1(rumbleStrength=0.5)
                # self.coreFunctionsInstance.ledBlink(True, 0.25, 0.25)  # Once blinking, Gadget can let go of button or trigger

        # Emergency coral transfer
        elif self.joystick1.getLeftY() < -0.8:
            if self.freshEmergency:
                self.emergencyTransfer = True
                self.freshEmergency = False
            self.coralIntakePower = self.coralIntakePowerGrab
            self.coralIntakeWristSetpoint = self.coralIntakeWristTransfer
            self.coralIntakeElbowSetpoint = self.coralIntakeElbowTransfer
            if self.coralIntakeElbow_Enc.getPosition() < self.coralIntakeElbowTransfer + self.setpointTriggerZoneFine:
                self.coralIntakePower = self.coralIntakePowerTransfer  # The actual transfer drop off
                self.coreFunctionsInstance.rumbleJoy1(rumbleStrength=0.5)   # Gadget controller rumble during drop off
                # Blink Led when Elbow near or at transfer position
                # self.coreFunctionsInstance.ledBlink(True, 0.25, 0.25)  # Once blinking, Gadget can let go of button or trigger
                if self.emergencyTransfer:
                    self.coralEjectSetpoint = self.coralEjectSetpoint - self.coralEjectIncrement * 4
                    self.emergencyTransfer = False
                self.coralIntakeCycleComplete = True

        # Algae pickup
        elif self.joystick1.getLeftY() > 0.8:
            self.algaeIntake = True
            # if self.newPipeline:  # This is so pipeline not continuously set
            #     self.coreFunctionsInstance.pipelineLimelightPub.set(2)  # limelight 2 algae ML vision detection
            #     self.newPipeline = False
            self.coralIntakeWristSetpoint = self.coralIntakeWristAlgae
            self.coralIntakeElbowSetpoint = self.coralIntakeElbowAlgae
            if self.coralIntakeElbow_Enc.getPosition() > self.coralIntakeElbowAlgae - self.setpointTriggerZoneLarge:
                self.coralIntakePower = self.coralIntakePowerAlgae
                # lockedOn = self.coreFunctionsInstance.targetAcquired(limelightNum=1)  # If target present and identified
                # if lockedOn:
                #     self.coreFunctionsInstance.ledOn()  # Turn LED on
                # else:
                #     self.coreFunctionsInstance.ledOff()  # Turn LED off

        # Algae deposit
        elif self.joystick1.getRightBumper() > 0.8 and (((self.coralIntakeElbowAlgae - self.setpointTriggerZone) < self.coralIntakeElbow_Enc.getPosition() < (self.coralIntakeElbowAlgae + self.setpointTriggerZone))
                                        and ((self.coralIntakeWristAlgae - self.setpointTriggerZone) < self.coralIntakeWrist_Enc.getPosition() < (self.coralIntakeWristAlgae + self.setpointTriggerZone))):
            self.coralIntakePower = self.coralIntakePowerAlgaeProcessor
            self.algaeIntake = False
            self.coreFunctionsInstance.ledOff()

        # Regurgitate coral for trough deposit. (2-21-2025 May need to slow down the Elbow min max)
        # This segment of code gets wrist into position and drops coral.
        elif self.joystick1.getRightBumper() > 0.8 and self.coralTilt_Enc.getPosition() < self.coralTiltBase + self.setpointTriggerZone and self.coral4Trough:
            self.coralTroughDrop = True
            self.coralIntakePower = self.coralIntakePowerGrab
            self.coralIntakeWristSetpoint = self.coralIntakeWristTrough
            self.coralIntakeElbowSetpoint = self.coralIntakeElbowTrough

        # LED control during climb. Here so ledBlink off is controlled in one place (else portion)
        elif self.climb_Enc.getVelocity() > 1:  # climbing up
            self.coreFunctionsInstance.ledBlink(True, 0.25, 0.25)

        elif self.climb_Enc.getPosition() > self.climbUpMax - self.setpointTriggerZone:
            self.coreFunctionsInstance.ledBlink(True, 10, 0)

        else:
            if self.coralTroughDrop:
                self.coralIntakePower = self.coralIntakePowerTrough
                timesUp = self.coralTroughDropTimer.autoTimer(0.75) # was 1 sec
                if timesUp:
                    self.coralIntakePower = 0
                    self.coralTroughDrop = False
                    self.coral4Trough = False   # Should not be in following else or will negate the original turning on with POV 180
            elif self.algaeIntake:
                self.coralIntakePower = self.coralIntakePowerAlgaeHold
            else:   #  Due to variable reset at start, coralIntakePower default to 0 if no other value overrides
                if self.coralIntakeFull:    # Hold coral if present in intake
                    self.coralIntakePower = self.coralIntakePowerHold
                self.coralTroughDrop = False
                self.coralTroughDropTimer.autoTimerReset()
                self.coralSpoonFedHailMaryMode = False

            self.coralIntakeTimer.autoTimerReset()  # Here to reset autoTimer instance when no failout occurred
            self.coralTransferTimer.autoTimerReset()
            self.coreFunctionsInstance.rumbleJoy1()
            # self.newPipeline = True # Pipeline lock released when initial button pressed released for a new one.
            self.coreFunctionsInstance.ledBlink(False)  # Turn off blinking without conflict
            # resetting emergency transfer variables
            self.freshEmergency = True
            self.emergencyTransfer = False
            # Another method would be to have a Primer at top of code that gets updated below it.
            if self.coralIntakeCycleComplete:  # If cycle not complete, continue at step where last left of.
                self.coralIntakeStep = 1
                

    def cascadeControl(self):
        # If in auton, need be called continuously for proper function, not just called in step.

        # Coral placement
        if self.joystick1.getXButton() or self.joystick1.getBButton() or self.joystick1.getYButton() or self.autonCoral4DropOff:
            self.algaeIntake = False    # To cancel algae intake mode.

            if self.joystick1.getXButtonPressed():
                self.cascade2Active = True
                self.coralTiltHailMaryMode = False
                self.coralEjectSetpoint = self.coralEjectSetpoint - self.coralEjectIncrement
                self.cascadeSetpoint = self.cascade2
                self.turtleneckSetpoint = self.turtleneckOut

            if self.joystick1.getBButtonPressed():
                self.cascade3Active = True
                self.coralTiltHailMaryMode = False
                self.coralEjectSetpoint = self.coralEjectSetpoint - self.coralEjectIncrement
                self.cascadeSetpoint = self.cascade3
                self.turtleneckSetpoint = self.turtleneckOut

            if self.joystick1.getYButtonPressed() or self.autonCoral4DropOff:   #*** in auton, this has to be a one off and not in a loop ***
                self.cascade4Active = True
                self.coralTiltHailMaryMode = False
                self.coralEjectSetpoint = self.coralEjectSetpoint - self.coralEjectIncrement    # Suck front dangling coral back in
                self.cascadeSetpoint = self.cascade4
                self.turtleneckSetpoint = self.turtleneckOut
                self.autonCoral4DropOff = False  # Auto Reset to prevent repeat. This method not work well since upper level unintended looping
                # continue to revert self.autonCoral4DropOff = True

        # Setting up cascade and turtleneck for a hail Mary coral drop when a coral is jammed between robot and reef
        if self.joystick1.getLeftStickButtonPressed() and (self.cascade1 < self.cascadeR_Enc.getPosition() < self.cascadeMax):
            if not self.cascade4Active:
                self.cascadeSetpoint = self.cascadeR_Enc.getPosition() + self.setpointTriggerZoneFine * 1.25  # elevating the cascade a little for greater reach
            self.turtleneckSetpoint = self.turtleneckMax
            self.coralTiltHailMaryMode = True
        # Setting up wrist and elbow for a hail Mary coral catch when a coral is jammed between robot and station
        elif self.joystick1.getLeftStickButton() and self.cascadeR_Enc.getPosition() < self.cascade1:
            self.coralSpoonFedHailMaryMode = True

        # Cascade back to default
        if self.joystick1.getAButtonPressed() or self.autonCascadeDefault:
            self.cascade2Active = False # Deactivated coralTilt for deposit
            self.cascade3Active = False
            self.cascade4Active = False
            self.coralTiltHailMaryMode = False
            self.cascadeSetpoint = self.cascadeBase
            self.coralTiltSetpoint = self.coralTiltBase
            self.turtleneckSetpoint = self.turtleneckIn
            self.autonCascadeDefault = False # Auto Reset to prevent repeat
            if not self.algaeBootCycleComplete: # Re-config cascade Min Max in case Algae boot not completed
                self.cascadeConfigOnTheFly(-0.18, self.cascadeOutputCoralMax)  # OG setting. Called 1 time only to change motor output

        # General loops to activate coralTilt. Upper trigger zone extended to account for increased cascade height for super reach mode.
        if ((self.cascade4 - self.setpointTriggerZone * 3) < self.cascadeR_Enc.getPosition() < (self.cascade4 + self.setpointTriggerZone * 3.5)) and self.cascade4Active:
            if self.coralTiltHailMaryMode:
                self.coralTiltSetpoint = self.coralTilt4_HailMary
            elif not self.teleoperated:
                self.coralTiltSetpoint = self.coralTilt4Auton
            else:
                self.coralTiltSetpoint = self.coralTilt4
        elif ((self.cascade3 - self.setpointTriggerZone * 3) < self.cascadeR_Enc.getPosition() < (self.cascade3 + self.setpointTriggerZone * 3.5)) and self.cascade3Active:
            if self.coralTiltHailMaryMode:
                self.coralTiltSetpoint = self.coralTilt123_HailMary
            else:
                self.coralTiltSetpoint = self.coralTilt123
        elif ((self.cascade2 - self.setpointTriggerZone * 3) < self.cascadeR_Enc.getPosition() < (self.cascade2 + self.setpointTriggerZone * 3.5)) and self.cascade2Active:
            if self.coralTiltHailMaryMode:
                self.coralTiltSetpoint = self.coralTilt123_HailMary
            else:
                self.coralTiltSetpoint = self.coralTilt123

        # Algae booting
        if self.joystick1.getRightY() > 0.8  or self.joystick1.getRightY() < -0.8 or self.autonAlgaeBoot23 or self.autonAlgaeBoot34:
            self.algaeIntake = False  # To cancel algae intake mode.
            if self.algaeBootStep == 1:
                self.algaeBootCycleComplete = False
                self.turtleneckSetpoint = self.turtleneckBoot  # Setting up position for pre-boot.
                self.coralTiltSetpoint = self.coralTilt123  # Added so can help reach algae
                if self.joystick1.getRightY() < -0.8 or self.autonAlgaeBoot34:   # Top algae
                    self.cascadeAlgaePreTarget = self.cascadeAlgae34PreBoot
                    self.cascadeAlgaeTarget = self.cascadeAlgae34Boot
                    self.algaeBootStep += 1
                if self.joystick1.getRightY() > 0.8 or self.autonAlgaeBoot23:  # Bottom algae
                    self.cascadeAlgaePreTarget = self.cascadeAlgae23PreBoot
                    self.cascadeAlgaeTarget = self.cascadeAlgae23Boot
                    # Right of the bat, cascade motor slowed down for lower algae to allow time for transitions
                    self.cascadeConfigOnTheFly(-0.18, self.cascadeOutputAlgaeMax)  # OG setting. Called 1 time only to change motor output
                    self.algaeBootStep += 3 # Skipping cascade motor MinMax config since already done and also the preBoot position altogether
            if self.algaeBootStep == 2:
                self.cascadeSetpoint = self.cascadeAlgaePreTarget
                if self.cascadeR_Enc.getPosition() > self.cascadeAlgaePreTarget - 1:
                    self.algaeBootStep += 1
            if self.algaeBootStep == 3: # Lowering cascade speed with lower Max
                self.cascadeConfigOnTheFly(-0.18, self.cascadeOutputAlgaeMax)  # Called 1 time only to change motor output
                self.algaeBootStep += 1
            if self.algaeBootStep == 4:
                self.algaeBootPower = self.algaeBootPowerPreset
                self.cascadeSetpoint = self.cascadeAlgaeTarget
                if self.cascadeR_Enc.getPosition() > self.cascadeAlgaeTarget - 1:
                    self.algaeBootStep += 1
            if self.algaeBootStep == 5: # Return cascade power config to default
                self.cascadeConfigOnTheFly(-0.18, self.cascadeOutputCoralMax)  # OG setting. Called 1 time only to change motor output
                self.algaeBootStep += 1
            if self.algaeBootStep == 6:
                self.algaeBootPower = self.algaeBootPowerPreset
                self.algaeBootCycleComplete = True
        else:
            self.algaeBootPower = 0
            if self.algaeBootCycleComplete:
                self.algaeBootStep = 1

        if self.joystick1.getRightStickButtonPressed() or self.postAlgaeBootCascadeReset: # Returning cascade to base after boot
            self.cascadeSetpoint = self.cascadeBase
            self.coralTiltSetpoint = self.coralTiltBase
            self.turtleneckSetpoint = self.turtleneckIn
            self.cascadeConfigOnTheFly(-0.18, self.cascadeOutputCoralMax)  # OG setting. Called 1 time only to change motor output
            self.postAlgaeBootCascadeReset = False

    def coralEjection(self):
        # If in auton, need be called continuously for proper function, not just called in the step.
        if (self.joystick1.getRightBumperPressed() and self.coralTilt_Enc.getPosition() > self.coralTilt123 - self.setpointTriggerZone * 1.5) or self.autonCoralEjection:
            self.algaeIntake = False  # To cancel algae intake mode.
            self.coralEjectSetpoint = self.coralEjectSetpoint + self.coralEjectIncrement
            self.autonCoralEjection = False # Self resetting autonCoralEjection

        if self.joystick1.getStartButtonPressed():
            self.coralEjectSetpoint = self.coralEjectSetpoint - (self.coralEjectIncrement / 2)  # Prairie dog back

    def autoSeekLimelight(self):
        if ((self.joystick1.getLeftBumper() and self.joystick1.getRightTriggerAxis() > 0.8) or self.autonCoralSpoonFedSeek or
                self.joystick1.getPOV() == 0 or self.joystick1.getPOV() == 90 or self.joystick1.getPOV() == 270 or
                self.autonAlgaeBootSeek or self.autonLeftCoatRackSeek or self.autonRightCoatRackSeek):
            self.fieldCentric = True
            if (self.joystick1.getLeftBumper() and self.joystick1.getRightTriggerAxis() > 0.8) or self.autonCoralSpoonFedSeek:
                limelightNumber = 1
                orientationOfAction = "west"
                virtualStickPowerLocal = self.virtualStickPower
                emergencyBrakeDistanceLocal = self.emergencyBrakeDistance
                driveAngleFudgeFactorLocal = self.driveAngleFudgeFactor
                faceAngleFudgeFactorLocal = self.faceAngleFudgeFactor
                self.coreFunctionsInstance.pipelineLimelightPub.set(0)  # Default pipeline for station autoseek
                if self.teleoperated:
                    self.lastInchesLeft = 2 # Effects autoseek in teleop. Additional boot at the end.
                else:
                    self.lastInchesLeft = 1 # Effects autoseek in auton. Additional boot at the end.
                # Limelight1 only need 1 pipeline so default is pipeline 0 for now. No need to set or switch to increase reliability.
                # if self.newPipeline2:  # This is so pipeline not continuously set
                #     self.coreFunctionsInstance.pipelineLimelightPub.set(0)  # limelight 1
                #     # self.coreFunctionsInstance.pipelineLimelightPub2.set(0) # limelight 2 LED off
                #     self.newPipeline2 = False
            elif self.joystick1.getPOV() == 0 or self.autonAlgaeBootSeek:
                limelightNumber = 2
                orientationOfAction = "north"
                virtualStickPowerLocal = self.virtualStickPower2
                emergencyBrakeDistanceLocal = self.emergencyBrakeDistance2
                driveAngleFudgeFactorLocal = self.driveAngleFudgeFactor2
                faceAngleFudgeFactorLocal = self.faceAngleFudgeFactor2
                # Setting pipeline only once may not be enough and the call may have been miss in lost packets. Therefore for
                # increased reliability, the setting is continuous and bombards the limelight with the command.
                if self.teleoperated:
                    self.limelight2Pipeline = self.algaeBootPipeline
                self.coreFunctionsInstance.pipelineLimelightPub2.set(self.limelight2Pipeline)  # limelight aprilTag central bias detection
                if self.teleoperated:
                    self.lastInchesLeft = 3 # Effects autoseek in teleop
                else:
                    self.lastInchesLeft = 2 # Effects autoseek in auton
                # if self.newPipeline2:
                #     self.coreFunctionsInstance.pipelineLimelightPub2.set(1)  # limelight aprilTag central bias detection
                #     # self.coreFunctionsInstance.pipelineLimelightPub.set(1)  # limelight 1 LED Off
                #     self.newPipeline2 = False
            elif self.joystick1.getPOV() == 270 or self.autonLeftCoatRackSeek:
                limelightNumber = 2
                orientationOfAction = "north"
                virtualStickPowerLocal = self.virtualStickPower2
                emergencyBrakeDistanceLocal = self.emergencyBrakeDistance2
                driveAngleFudgeFactorLocal = self.driveAngleFudgeFactor2
                faceAngleFudgeFactorLocal = self.faceAngleFudgeFactor2
                if self.teleoperated:
                    self.limelight2Pipeline = self.coral3L_Pos1_Blue
                self.coreFunctionsInstance.pipelineLimelightPub2.set(self.limelight2Pipeline)  # limelight aprilTag left bias detection
                if self.teleoperated:
                    self.lastInchesLeft = 3 #was 4 # Effects autoseek in teleop
                else:
                    self.lastInchesLeft = 0 #was 1 # Effects autoseek in auton
                # if self.newPipeline2:
                #     self.coreFunctionsInstance.pipelineLimelightPub2.set(2)  # limelight aprilTag left bias detection
                #     # self.coreFunctionsInstance.pipelineLimelightPub.set(1)  # limelight 1 LED Off
                #     self.newPipeline2 = False
            elif self.joystick1.getPOV() == 90 or self.autonRightCoatRackSeek:
                limelightNumber = 2
                orientationOfAction = "north"
                virtualStickPowerLocal = self.virtualStickPower2
                emergencyBrakeDistanceLocal = self.emergencyBrakeDistance2
                driveAngleFudgeFactorLocal = self.driveAngleFudgeFactor2
                faceAngleFudgeFactorLocal = self.faceAngleFudgeFactor2
                if self.teleoperated:
                    self.limelight2Pipeline = self.coral1R_Pos1_Blue    # General pipeline that works for right rack
                self.coreFunctionsInstance.pipelineLimelightPub2.set(self.limelight2Pipeline)  # limelight aprilTag right bias detection
                if self.teleoperated:
                    self.lastInchesLeft = 2 # was 1 # Effects autoseek in teleop
                else:
                    self.lastInchesLeft = 1 #was 0 # Effects autoseek in auton
                # if self.newPipeline2:
                #     self.coreFunctionsInstance.pipelineLimelightPub2.set(3)  # limelight aprilTag right bias detection
                #     # self.coreFunctionsInstance.pipelineLimelightPub.set(1)  # limelight 1 LED Off
                #     self.newPipeline2 = False

            self.speedLimitMultiplier = self.maxAutoSeekNStopSpeed  # Assures autoSeek power not influence by Driver setting
            # speedLimitMultiplier reverts back to original driver setting in driverControl portion, but only in teleop.

            lockedOn = self.coreFunctionsInstance.targetAcquired(limelightNumber)  # If target present and identified
            if lockedOn:
                self.incompleteJourney = True
                self.autonomous = True
                self.coreFunctionsInstance.ledOn()  # Turn LED on # OOPS
                if self.teleoperated:   # OOPS
                    self.coreFunctionsInstance.rumbleJoy0(rumbleStrength=0.5)  # Turn on driver controller rumble

                angleToParallel, self.autoSeekNStopAngle, self.autoSeekNStopPower, self.autoSeekNStopDistanceLeftInInches = self.coreFunctionsInstance.autoSeekNStop(orientationOfAction, virtualStickPowerLocal, emergencyBrakeDistanceLocal, driveAngleFudgeFactorLocal, self.decelFudgeFactor, limelightNumber)
                # decelFudgeFactor range is 1-3. >1 is increasing braking.
                # The 3 returned values are store in case lockedOn fails and robot completes the journey with autoDrive
                if angleToParallel > 1: # in degrees
                    faceAngleFudgeFactor = -faceAngleFudgeFactorLocal
                elif angleToParallel < -1:  # in degrees
                    faceAngleFudgeFactor = faceAngleFudgeFactorLocal
                else:
                    faceAngleFudgeFactor = 0

                faceAngleActual = self.coreFunctionsInstance.gyroAngleFinal
                self.autoFaceAngle = faceAngleActual - angleToParallel + faceAngleFudgeFactor   # The orientationOfAction is NOT important
                self.autoSeekNStopPowerOG = self.autoSeekNStopPower

            elif self.incompleteJourney:    # This portion only entered if lockedOn occurred earlier, or the initial parameters passed would not be relevant
                #   self.autonomous, self.fieldCentric, self.autoFaceAngle should all still be assigned properly elsewhere.
                # Last saved parameters passes down to autoDrive to help complete the journey. No braking power is handled here.
                self.coreFunctionsInstance.ledOff()  # Turn LED off # OOPS
                self.coreFunctionsInstance.autoDrive(self.autoSeekNStopAngle, self.autoSeekNStopPower, self.lastInchesLeft) # failoutTime is optional. Using autodrive route.

        else:
            if self.teleoperated: ## OOPS
                self.autonomous = False # So driver can control rotation again in teleop.
                # DO NOT place following line in main unfiltered loop. Will BORK autoDrive since initialPosition will never be set correctly.
                self.coreFunctionsInstance.newAutoDrive = True  # In teleop, this allows reset of autodrive.
            if self.autonomous:
                self.speedLimitMultiplier = self.speedLimitMultiplierOG # Revert speed back here since reverting back code is in Driver code (which is not present in auton)
            self.coreFunctionsInstance.rumbleJoy0()  # Turn off driver controller rumble
            self.newPipeline = True
            self.newPipeline2 = True  # Pipeline lock released when initial button pressed released for a new one.
            self.incompleteJourney = False
            self.autoSeekNStopDistanceLeftInInches = 0  # Resetting variable since no longer autoseeking. Needed?

            # The following allows for the resetting of essential variables in autoDrive
            self.coreFunctionsInstance.autoDriveFailoutTimer.autoTimerReset()  # Here to reset autoTimer instance when no failout occurred
            # self.coreFunctionsInstance.autoFaceAngleComplete = False  # This is to reset autoFaceAngleTurnOnly since if preceded
            # by autoDrive, in some scenarios, this variable may remain True and prevent turnOnly function.

    def emergencyElbowEncoderReset(self):
        self.coralIntakeElbowSetpoint = -1000
        self.elbowConfigOnTheFly(-0.3, 0.3)
        mysteryElbowEncoderVelocity = self.coralIntakeElbow_Enc.getVelocity()
        endpointReached = self.emergencyElbowReset.speedDeltaDetection(True, 0.5, 0.5, mysteryElbowEncoderVelocity)
        if endpointReached:
            self.coralIntakeElbow_Enc.setPosition(-1)
            self.coralIntakeElbowSetpoint = 0

    def emergencyWristEncoderReset(self):
        mysteryWristEncoderPosition = self.coralIntakeWrist_Enc.getPosition()
        self.coralIntakeWristSetpoint = mysteryWristEncoderPosition - 25
        self.elbowConfigOnTheFly(-0.3, 0.3)
        mysteryWristEncoderVelocity = self.coralIntakeWrist_Enc.getVelocity()
        endpointReached = self.emergencyWristReset.speedDeltaDetection(True, 0.5, 0.5, mysteryWristEncoderVelocity)
        if endpointReached:
            self.coralIntakeWrist_Enc.setPosition(-1)
            self.coralIntakeWristSetpoint = 0

    def emergencyReset(self):
        if self.joystick1.getLeftX() > 0.8 and self.joystick1.getRightX() < -0.8:
            self.emergencyElbowEncoderReset()
            # self.emergencyWristEncoderReset()

    def robotDefault(self): # Resetting essential variables and robot positions
        if self.joystick1.getBackButtonPressed() or self.autonRobotDefault:
            self.coralIntakePower = 0
            self.coralIntakeStep = 1
            self.coralIntakeCycleComplete = True
            self.algaeBootStep = 1
            self.algaeBootCycleComplete = True
            self.coralIntakeWristSetpoint = self.coralIntakeWristTransfer
            self.coralIntakeElbowSetpoint = self.coralIntakeElbowTransfer
            self.coralTiltSetpoint = self.coralTiltBase
            self.cascadeSetpoint = self.cascadeBase
            self.turtleneckSetpoint = self.turtleneckIn
            self.climbSetpoint = self.climbBase
            self.mouseTrapSetpoint = self.mouseTrapCoralBlocker
            self.coreFunctionsInstance.ledOff()
            self.algaeIntake = False  # Redundant
            self.coralTroughDrop = False  # Redundant
            self.coral4Trough = False  # Redundant
            self.newPipeline = True # Redundant
            self.newPipeline2 = True  # Redundant
            self.incompleteJourney = False   # Redundant
            self.cascade2Active = False
            self.cascade3Active = False
            self.cascade4Active = False
            self.coralTiltHailMaryMode = False
            self.coralSpoonFedHailMaryMode = False
            # self.cascadeConfigOnTheFly(-0.18,
            #                            self.cascadeOutputCoralMax)  # OG setting. Called 1 time only to change motor output
            self.autonRobotDefault = False  # Auto Reset to prevent repeat. Ineffective since constantly called in repeated cycles.

    def autonCascadeDownNReset(self):
        self.cascade4Active = False
        self.cascadeSetpoint = self.cascadeBase
        self.coralTiltSetpoint = self.coralTiltBase
        self.turtleneckSetpoint = self.turtleneckIn
        self.incompleteJourney = False  # Redundant


    def cascadeConfigOnTheFly(self, min, max):
        self.cascadeR_config.inverted(False)
        self.cascadeR_config.closedLoop.pid(0.1, 0.0, 0.0)
        self.cascadeR_config.closedLoop.outputRange(min, max)

        self.cascadeL_config.inverted(True)
        self.cascadeL_config.closedLoop.pid(0.1, 0.0, 0.0)
        self.cascadeL_config.closedLoop.outputRange(min, max)

        self.cascadeR.configure(self.cascadeR_config, SparkBase.ResetMode.kResetSafeParameters,
                                SparkBase.PersistMode.kPersistParameters)
        self.cascadeL.configure(self.cascadeL_config, SparkBase.ResetMode.kResetSafeParameters,
                                SparkBase.PersistMode.kPersistParameters)

    def elbowConfigOnTheFly(self, min, max):
        self.coralIntakeElbow_config.inverted(False)
        self.coralIntakeElbow_config.closedLoop.pid(0.1, 0.0, 0.0)
        self.coralIntakeElbow_config.closedLoop.outputRange(min, max)

        self.coralIntakeElbow.configure(self.coralIntakeElbow_config, SparkBase.ResetMode.kResetSafeParameters,
                                SparkBase.PersistMode.kPersistParameters)

    def wristConfigOnTheFly(self, min, max):
        self.coralIntakeWrist_config.inverted(False)
        self.coralIntakeWrist_config.closedLoop.pid(0.1, 0.0, 0.0)
        self.coralIntakeWrist_config.closedLoop.outputRange(min, max)

        self.coralIntakeWrist.configure(self.coralIntakeWrist_config, SparkBase.ResetMode.kResetSafeParameters,
                                        SparkBase.PersistMode.kPersistParameters)

    # def mouseTrapConfigOnTheFly(self, min, max):
    #     self.mouseTrap_config.inverted(False)
    #     self.mouseTrap_config.closedLoop.pid(0.1, 0.0, 0.0)
    #     self.mouseTrap_config.closedLoop.outputRange(min, max)
    #
    #     self.mouseTrap.configure(self.mouseTrap_config, SparkBase.ResetMode.kResetSafeParameters,
    #                              SparkBase.PersistMode.kPersistParameters)

    def readingAutonSwitches(self):    # Auton DIO
        self.teamBlue = not self.teamColorSwitch.get()

        if not self.autonPosition1Switch.get():
            self.autonPosition = 1
        elif not self.autonPosition3Switch.get():
            self.autonPosition = 3
        else:
            self.autonPosition = 2

        if not self.autonOption1Switch.get():
            self.autonOption = 1
        elif not self.autonOption3Switch.get():
            self.autonOption = 3
        else:
            self.autonOption = 2

    def readingSensors(self):
        self.coralIntakeFull = not self.coralIntakeIRSensor.get()
        # self.elbowLimitReached = not self.elbowLimitSwitch.get()

    # Telemetry
    # Generic part 1 code for declaring a telemetry publishing (this declaration is placed in robotInit)
    # self.xxxPub = self.coreTable.getDoubleTopic("xxx").publish()
    # self.xxxPub = self.coreTable.getBooleanTopic("xxx").publish()

    def tazMotorsPub(self): # For robotInit
        self.coralIntakePub = self.tazTable.getDoubleTopic("coralIntake").publish()
        self.coralIntakeVelocityPub = self.tazTable.getDoubleTopic("coralIntakeVelocity").publish()
        self.coralIntakeImpactPub = self.tazTable.getBooleanTopic("coralIntakeImpact").publish()
        self.coralIntakeWristPub = self.tazTable.getDoubleTopic("coralIntakeWrist").publish()
        self.coralIntakeElbowPub = self.tazTable.getDoubleTopic("coralIntakeElbow").publish()
        self.coralTiltPub = self.tazTable.getDoubleTopic("coralTilt").publish()
        self.cascadeRPub = self.tazTable.getDoubleTopic("cascadeR").publish()
        self.cascadeLPub = self.tazTable.getDoubleTopic("cascadeL").publish()
        self.algaeBootPub = self.tazTable.getDoubleTopic("algaeBoot").publish()
        self.coralEjectPub = self.tazTable.getDoubleTopic("coralEject").publish()
        self.turtleneckPub = self.tazTable.getDoubleTopic("turtleneck").publish()
        self.climbPub = self.tazTable.getDoubleTopic("climb").publish()
        self.mouseTrapPub = self.tazTable.getDoubleTopic("mouseTrap").publish()

    def tazMotorsTelemetry(self): # For periodic
        self.coralIntakePub.set(self.coralIntake_Enc.getPosition())
        if self.teleoperated:
            self.coralIntakeVelocityPub.set(self.coralIntake_Enc.getVelocity())
        self.coralIntakeWristPub.set(self.coralIntakeWrist_Enc.getPosition())
        self.coralIntakeElbowPub.set(self.coralIntakeElbow_Enc.getPosition())
        self.coralTiltPub.set(self.coralTilt_Enc.getPosition())
        self.cascadeRPub.set(self.cascadeR_Enc.getPosition())
        self.cascadeLPub.set(self.cascadeL_Enc.getPosition())
        self.algaeBootPub.set(self.algaeBoot_Enc.getPosition())
        self.coralEjectPub.set(self.coralEject_Enc.getPosition())
        self.turtleneckPub.set(self.turtleneck_Enc.getPosition())
        self.climbPub.set(self.climb_Enc.getPosition())
        self.mouseTrapPub.set(self.mouseTrap_Enc.getPosition())

    def DIO_Pub(self):
        self.teamBluePub = self.tazTable.getBooleanTopic("teamBlue").publish()
        self.autonPositionPub = self.tazTable.getDoubleTopic("autonPosition").publish()
        self.autonOptionPub = self.tazTable.getDoubleTopic("autonOption").publish()
        self.coralIntakeFullPub = self.tazTable.getBooleanTopic("coralIntakeFull").publish()
        # self.elbowLimitPub = self.tazTable.getBooleanTopic("elbowLimit").publish()

    def DIO_Telemetry(self):
        self.teamBluePub.set(self.teamBlue)
        self.autonPositionPub.set(self.autonPosition)
        self.autonOptionPub.set(self.autonOption)
        self.coralIntakeFullPub.set(self.coralIntakeFull)
        # self.elbowLimitPub.set(self.elbowLimitReached)

    def joystickPub(self):
        self.leftXPub = self.tazTable.getDoubleTopic("leftX").publish()
        self.leftYPub = self.tazTable.getDoubleTopic("leftY").publish()
        self.rightXPub = self.tazTable.getDoubleTopic("rightX").publish()
        self.rightYPub = self.tazTable.getDoubleTopic("rightY").publish()
        self.buttonA_Pub = self.tazTable.getBooleanTopic("buttonA").publish()
        self.buttonB_Pub = self.tazTable.getBooleanTopic("buttonB").publish()
        self.buttonX_Pub = self.tazTable.getBooleanTopic("buttonX").publish()
        self.buttonY_Pub = self.tazTable.getBooleanTopic("buttonY").publish()
        self.leftBumperPub = self.tazTable.getBooleanTopic("leftBumper").publish()
        self.rightBumperPub = self.tazTable.getBooleanTopic("rightBumper").publish()
        self.leftTriggerPub = self.tazTable.getDoubleTopic("leftTrigger").publish()
        self.rightTriggerPub = self.tazTable.getDoubleTopic("rightTrigger").publish()
        self.backButtonPub = self.tazTable.getBooleanTopic("backButton").publish()
        self.startButtonPub = self.tazTable.getBooleanTopic("startButton").publish()
        self.povPub = self.tazTable.getDoubleTopic("pov").publish()

    def joystickTelemetry(self):
        self.leftXPub.set(self.coreFunctionsInstance.joystick0.getLeftX())
        self.leftYPub.set(self.coreFunctionsInstance.joystick0.getLeftY())
        self.rightXPub.set(self.coreFunctionsInstance.joystick0.getRightX())
        self.rightYPub.set(self.coreFunctionsInstance.joystick0.getRightY())
        self.buttonA_Pub.set(self.coreFunctionsInstance.joystick0.getAButton())
        self.buttonB_Pub.set(self.coreFunctionsInstance.joystick0.getBButton())
        self.buttonX_Pub.set(self.coreFunctionsInstance.joystick0.getXButton())
        self.buttonY_Pub.set(self.coreFunctionsInstance.joystick0.getYButton())
        self.leftBumperPub.set(self.coreFunctionsInstance.joystick0.getLeftBumper())
        self.rightBumperPub.set(self.coreFunctionsInstance.joystick0.getRightBumper())
        self.leftTriggerPub.set(self.coreFunctionsInstance.joystick0.getLeftTriggerAxis())
        self.rightTriggerPub.set(self.coreFunctionsInstance.joystick0.getRightTriggerAxis())
        self.backButtonPub.set(self.coreFunctionsInstance.joystick0.getBackButton())
        self.startButtonPub.set(self.coreFunctionsInstance.joystick0.getStartButton())
        self.povPub.set(self.coreFunctionsInstance.joystick0.getPOV())



if __name__ == "__main__":
    wpilib.run(MyRobot)
