import wpilib
from rev import SparkBase, SparkFlex, SparkMax, SparkLowLevel, SparkFlexConfig, ClosedLoopConfig
from phoenix6 import hardware, controls, configs, signals
from CoreComponents.SwerveDrive import SwerveDrive
from navx import AHRS
import ntcore
import math

class CoreFunctions:
    joystickChannel = 0
    def __init__(self, driveMotorType, rotationMotorType, testbed, cancoderAbsent, autoDriveEndpointSlop,
                 wheelbaseWidth, wheelbaseLength, rotationalGearRatio, driveRotPerInch,
                 joystickDeadzone, gyroDeadzone, limelight1_DefaultPipeline = 0, limelight2_DefaultPipeline = 0):
        # Essential defaults
        self.driveMotorType = driveMotorType
        self.rotationMotorType = rotationMotorType
        self.testbed = testbed
        self.cancoderAbsent = cancoderAbsent
        self.autoDriveEndpointSlop = autoDriveEndpointSlop
        self.gyroNorthReset = False  # This is needed for proper function of Swerve code.
        self.joystickDeadzone = joystickDeadzone
        self.ledBlinkReset = True
        self.newAutoDrive = True

        self.autoFaceAngleTurnOnlyMode = False
        self.autoFaceAngleComplete = False  # Will prevent a crash if 1st step in auton is turn only.

        # Default robot orientation.
        # self.gyroCompensation not needed here since already established and passed on in Main code.
        self.gyroResetConstant = 0

        self.navxAttached = True  # Was False  # If navX Gyro is onboard or not on a testbed (testbed can run either way)
        if not self.testbed:  # This will assure when actual robot running and not on testbed, the gyro will always be on (assuming navx will definitely be on a running robot)
            self.navxAttached = True

        if self.navxAttached:
            # Initialize navX
            self.navx = AHRS(AHRS.NavXComType.kUSB1) # use kUSB1 or kUSB for outer USB
            self.navx.reset()

        # Declaring controllers
        self.joystick0 = wpilib.XboxController(0)  # First controller - Driver
        self.joystick1 = wpilib.XboxController(1)  # Second controller  - Gadget
        self.joystick2 = wpilib.XboxController(2)  # Third controller  - Tuning

        # Initialize Rev PDP so can be used to turn "custom relay switch" on or off
        self.powerDistributionHub = wpilib.PowerDistribution(9, wpilib.PowerDistribution.ModuleType.kRev)
        self.powerDistributionHub.setSwitchableChannel(False) # LED off
        self.ledBlinkStep = 1

        # Initializing cancoders
        # Phoenix6 syntax
        self.cancoderFR = hardware.CANcoder(21)
        self.cancoderFL = hardware.CANcoder(22)
        self.cancoderBL = hardware.CANcoder(23)
        self.cancoderBR = hardware.CANcoder(24)

        # Declaring motors
        if self.driveMotorType == 1:
            self.driveFR = SparkFlex(5, SparkLowLevel.MotorType.kBrushless)
            self.driveFR.setInverted(False) # Despite new 2025 Rev configure mode, this still works.
            self.driveFR_Enc = self.driveFR.getEncoder()

            self.driveFL = SparkFlex(6, SparkLowLevel.MotorType.kBrushless)
            self.driveFL.setInverted(False)
            self.driveFL_Enc = self.driveFL.getEncoder()

            self.driveBL = SparkFlex(7, SparkLowLevel.MotorType.kBrushless)
            self.driveBL.setInverted(False)
            self.driveBL_Enc = self.driveBL.getEncoder()

            self.driveBR = SparkFlex(8, SparkLowLevel.MotorType.kBrushless)
            self.driveBR.setInverted(False)
            self.driveBR_Enc = self.driveBR.getEncoder()

        if self.driveMotorType == 2:    # Kraken
            # This inversion actually changes setting in motor controller. Can change without cycling power and
            # Will have to change CLOCKWISE_POSITIVE to COUNTER_CLOCKWISE_POSITIVE once to revert back to default

            # # General reversal of motor call
            # driveMotor_configs = configs.MotorOutputConfigs()
            # driveMotor_configs.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
            #
            # driveMotorReversed_configs = configs.MotorOutputConfigs()
            # driveMotorReversed_configs.inverted = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

            self.driveFR = hardware.TalonFX(5)
            self.driveFL = hardware.TalonFX(6)
            self.driveBL = hardware.TalonFX(7)
            self.driveBR = hardware.TalonFX(8)

            # self.driveFR.configurator.apply(driveMotorReversed_configs)       # Applying standard (driveMotor_configs) vs. reversal (driveMotorReversed_configs)
            # self.driveFL.configurator.apply(driveMotorReversed_configs)
            # self.driveBL.configurator.apply(driveMotorReversed_configs)
            # self.driveBR.configurator.apply(driveMotorReversed_configs)

        if self.rotationMotorType == 1:  # Neo
            self.rotationFR = SparkFlex(1, SparkLowLevel.MotorType.kBrushless)
            self.rotationFR_Enc = self.rotationFR.getEncoder()
            self.rotationFR_PID = self.rotationFR.getClosedLoopController()

            self.rotationFL = SparkFlex(2, SparkLowLevel.MotorType.kBrushless)
            self.rotationFL_Enc = self.rotationFL.getEncoder()
            self.rotationFL_PID = self.rotationFL.getClosedLoopController()

            self.rotationBL = SparkFlex(3, SparkLowLevel.MotorType.kBrushless)
            self.rotationBL_Enc = self.rotationBL.getEncoder()
            self.rotationBL_PID = self.rotationBL.getClosedLoopController()

            self.rotationBR = SparkFlex(4, SparkLowLevel.MotorType.kBrushless)
            self.rotationBR_Enc = self.rotationBR.getEncoder()
            self.rotationBR_PID = self.rotationBR.getClosedLoopController()

            # PID Gain and MaxOutput constants for rotational motors.
            rotationAll_P = 0.5
            rotationAll_I = 0.01
            rotationAll_D = 0
            rotationAll_MaxOutput = 0.5

            # Configuring rotational motor Constants
            self.rotation_config = SparkFlexConfig()
            self.rotation_config.inverted(False)
            self.rotation_config.closedLoop.pid(rotationAll_P, rotationAll_I, rotationAll_D)
            self.rotation_config.closedLoop.outputRange(-rotationAll_MaxOutput, rotationAll_MaxOutput)

            self.rotationFR.configure(self.rotation_config, SparkBase.ResetMode.kResetSafeParameters,
                                   SparkBase.PersistMode.kPersistParameters)
            self.rotationFL.configure(self.rotation_config, SparkBase.ResetMode.kResetSafeParameters,
                                      SparkBase.PersistMode.kPersistParameters)
            self.rotationBL.configure(self.rotation_config, SparkBase.ResetMode.kResetSafeParameters,
                                      SparkBase.PersistMode.kPersistParameters)
            self.rotationBR.configure(self.rotation_config, SparkBase.ResetMode.kResetSafeParameters,
                                      SparkBase.PersistMode.kPersistParameters)


        if self.rotationMotorType == 2: # Kraken
            # This inversion actually changes setting in motor controller. Can change without cycling power and
            # Will have to change CLOCKWISE_POSITIVE to COUNTER_CLOCKWISE_POSITIVE once to revert back to default

            # General reversal of motor call
            motor_configs = configs.MotorOutputConfigs()
            motor_configs.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE

            # Slot 0 PID preset for Motion Magic (basically a fancy PID system with
            # smoother velocity control and not mere MinMaxPower. Better than PositionVoltage/PositionDutyCycle)
            rotation_configs = configs.TalonFXConfiguration()

            slot0_configs = rotation_configs.slot0
            slot0_configs.k_s = 0  # Was 0.25 - Add 0.25 V output to overcome static friction. Caused rotational drift.
            slot0_configs.k_v = 0  # Was 0.12 - A velocity target of 1 rps results in 0.12 V output. Caused rotational drift.
            slot0_configs.k_a = 0  # An acceleration of 1 rps/s requires 0.01 V output. Causes end twitch.
            slot0_configs.k_p = 0.7 #was 0.6 #waswas 3
            slot0_configs.k_i = 0.0 #was 0.5
            slot0_configs.k_d = 0.3
            # set Motion Magic refinement settings
            motion_magic_configs = rotation_configs.motion_magic
            motion_magic_configs.motion_magic_cruise_velocity = 80  # Target cruise velocity of 80 rps
            motion_magic_configs.motion_magic_acceleration = 160 #was 160  # Target acceleration of 160 rps/s (0.5seconds)
            motion_magic_configs.motion_magic_jerk = 1600  # Target jerk of 1600 rps/s/s (0.1 seconds)

            self.rotationFR = hardware.TalonFX(1)
            self.rotationFL = hardware.TalonFX(2)
            self.rotationBL = hardware.TalonFX(3)
            self.rotationBR = hardware.TalonFX(4)

            self.rotationFR.configurator.apply(rotation_configs)    # Applying PID / Motion Magic
            self.rotationFL.configurator.apply(rotation_configs)
            self.rotationBL.configurator.apply(rotation_configs)
            self.rotationBR.configurator.apply(rotation_configs)

            #  Motor reversal must be after Motion Magic application or will be cancelled
            self.rotationFR.configurator.apply(motor_configs)       # Applying reversal
            self.rotationFL.configurator.apply(motor_configs)
            self.rotationBL.configurator.apply(motor_configs)
            self.rotationBR.configurator.apply(motor_configs)

        # Regarding REV SparkMax
        # The restoreFactoryDefaults() method can be used to reset the
        # configuration parameters in the SPARK MAX to their factory default
        # state. If no argument is passed, these parameters will not persist
        # between power cycles. ? what are the arguments --> something to set the parameters into memory?

        # self.rotationFR.restoreFactoryDefaults()   # do not use since cancels motor reversal.
        # Reversal is maintained between robot restarts even if reversal is removed.
        # The factory restore does wipe out the residual motor reversal. Rev neo by default turns counterclockwise.

        self.driveRotPerInch = driveRotPerInch  # Self variable needed since will be modified by a function in this class
        # pending testbed status

        rotationalGearRatio_LocalValue = self.resettingRotationalMotorEncoder(rotationalGearRatio)  # Reset swerve wheel orientation encoder setup. This has to be before declaring SwerveDrive instance
        # since that requires the rotationalGearRatio value

        # Declaring SwerveDrive instance. This has to be after above changing of rotationalGearRatio to 1 for testbed
        self.swerve = SwerveDrive(wheelbaseWidth, wheelbaseLength, rotationalGearRatio_LocalValue,
                                  joystickDeadzone, gyroDeadzone)

        # Declaring timer instance
        self.autoDriveFailoutTimer = NiftyTimer()
        self.ledTimer = NiftyTimer()

        # NetworkTable initialization
        self.coreTableInstance = ntcore.NetworkTableInstance.getDefault()

        # Limelight subscribing and publishing setup
        self.limelightTable = self.coreTableInstance.getTable("limelight")  # IP's not work here
        self.limelightTable2 = self.coreTableInstance.getTable("limelight-two")

        self.txLimelightSub = self.limelightTable.getDoubleTopic("tx").subscribe(
            0.0)  # via limelight to limelightTable, for direct on robot retrieval
        self.tyLimelightSub = self.limelightTable.getDoubleTopic("ty").subscribe(0.0)
        self.taLimelightSub = self.limelightTable.getDoubleTopic("ta").subscribe(0.0)
        self.tvLimelightSub = self.limelightTable.getDoubleTopic("tv").subscribe(0)
        self.tidLimelightSub = self.limelightTable.getDoubleTopic("tid").subscribe(0.0)

        self.txLimelightSub2 = self.limelightTable2.getDoubleTopic("tx").subscribe(
            0.0)  # via limelight2 to limelightTable2
        self.tyLimelightSub2 = self.limelightTable2.getDoubleTopic("ty").subscribe(0.0)
        self.taLimelightSub2 = self.limelightTable2.getDoubleTopic("ta").subscribe(0.0)
        self.tvLimelightSub2 = self.limelightTable2.getDoubleTopic("tv").subscribe(0)
        self.tidLimelightSub2 = self.limelightTable2.getDoubleTopic("tid").subscribe(0.0)

        # dataArray from 3D AprilTag
        self.dataArrayLimelightSub = self.limelightTable.getDoubleArrayTopic("camerapose_targetspace").subscribe([6])
        self.dataArrayLimelightSub2 = self.limelightTable2.getDoubleArrayTopic("camerapose_targetspace").subscribe(
            [6])  # OG

        # Changing settings on Limelight
        self.pipelineLimelightPub = self.limelightTable.getDoubleTopic(
            "pipeline").publish()  # via limelightTable to limelight
        self.pipelineLimelightPub.set(limelight1_DefaultPipeline)  # Default pipeline
        self.streamLimelightPub = self.limelightTable.getDoubleTopic(
            "stream").publish()
        self.streamLimelightPub.set(2)  # 2 is PIP with webcam as main. 1 is opposite. 0 is side-by-side

        # self.ledLimelightPub = self.limelightTable.getDoubleTopic("ledMode").publish()
        # self.ledLimelightPub.set(1) # Default LED. 1 is off. 3 is on.

        self.pipelineLimelightPub2 = self.limelightTable2.getDoubleTopic(
            "pipeline").publish()  # via limelightTable2 to limelight2
        self.pipelineLimelightPub2.set(limelight2_DefaultPipeline)  # Default pipeline
        # self.streamLimelightPub2 = self.limelightTable.getDoubleTopic(
        #     "stream").publish()
        # self.streamLimelightPub2.set(2)
        # self.ledLimelightPub2 = self.limelightTable2.getDoubleTopic("ledMode").publish()
        # self.ledLimelightPub.set(1)  # Default LED. 1 is off. 3 is on.

        self.coreTable = self.coreTableInstance.getTable("datatable")

        # General subscribing and publishing setup
        # Generic part 1 code for declaring a telemetry publishing ( this declaration is placed in robotInit)
        # self.xxxPub = self.coreTable.getDoubleTopic("xxx").publish()
        # self.xxxPub = self.coreTable.getBooleanTopic("xxx").publish()

        self.txPub = self.coreTable.getDoubleTopic("tx").publish()  # via coreTable to shuffleboard
        self.tyPub = self.coreTable.getDoubleTopic("ty").publish()
        self.taPub = self.coreTable.getDoubleTopic("ta").publish()
        self.tvPub = self.coreTable.getDoubleTopic("tv").publish()
        self.tidPub = self.coreTable.getDoubleTopic("tid").publish()
        self.pipelinePub = self.coreTable.getDoubleTopic("pipeline").publish()

        self.txPub2 = self.coreTable.getDoubleTopic("tx2").publish()  # via coreTable to shuffleboard
        self.tyPub2 = self.coreTable.getDoubleTopic("ty2").publish()
        self.taPub2 = self.coreTable.getDoubleTopic("ta2").publish()
        self.tvPub2 = self.coreTable.getDoubleTopic("tv2").publish()
        self.tidPub2 = self.coreTable.getDoubleTopic("tid2").publish()
        self.pipelinePub2 = self.coreTable.getDoubleTopic("pipeline2").publish()

        # Publishing elements form dataArray
        self.txFromDataArrayPub = self.coreTable.getDoubleTopic("txFromDataArray").publish()
        self.tzFromDataArrayPub = self.coreTable.getDoubleTopic("tzFromDataArray").publish()
        self.RyFromDataArrayPub = self.coreTable.getDoubleTopic("RyFromDataArray").publish()
        self.targetDistancePub = self.coreTable.getDoubleTopic("targetDistance").publish()

        self.txFromDataArrayPub2 = self.coreTable.getDoubleTopic("txFromDataArray2").publish()
        self.tzFromDataArrayPub2 = self.coreTable.getDoubleTopic("tzFromDataArray2").publish()
        self.RyFromDataArrayPub2 = self.coreTable.getDoubleTopic("RyFromDataArray2").publish()
        self.targetDistancePub2 = self.coreTable.getDoubleTopic("targetDistance2").publish()

        # Drive system telemetry
        self.driveFR_EncPub = self.coreTable.getDoubleTopic("driveFR_Enc").publish()
        self.driveFL_EncPub = self.coreTable.getDoubleTopic("driveFL_Enc").publish()
        self.driveBL_EncPub = self.coreTable.getDoubleTopic("driveBL_Enc").publish()
        self.driveBR_EncPub = self.coreTable.getDoubleTopic("driveBR_Enc").publish()

        self.rotationFR_EncPub = self.coreTable.getDoubleTopic("rotationFR_Enc").publish()
        self.rotationFL_EncPub = self.coreTable.getDoubleTopic("rotationFL_Enc").publish()
        self.rotationBR_EncPub = self.coreTable.getDoubleTopic("rotationBR_Enc").publish()
        self.rotationBL_EncPub = self.coreTable.getDoubleTopic("rotationBL_Enc").publish()

        # Cancoder coreTable publication
        self.cancoderFRPub = self.coreTable.getDoubleTopic("cancoderFR").publish()
        self.cancoderFLPub = self.coreTable.getDoubleTopic("cancoderFL").publish()
        self.cancoderBRPub = self.coreTable.getDoubleTopic("cancoderBR").publish()
        self.cancoderBLPub = self.coreTable.getDoubleTopic("cancoderBL").publish()

        self.gyroAngleFinalPub = self.coreTable.getDoubleTopic("gyroAngleFinal").publish()

        self.autonStepPub = self.coreTable.getDoubleTopic("autonStep").publish()

        self.autonPositionPub = self.coreTable.getDoubleTopic("autonPosition").publish()

        self.robotCodePub = self.coreTable.getBooleanTopic("robotCode").publish()

        # Rev PDH amperage publication
        self.ampCh0Pub = self.coreTable.getDoubleTopic("ampCh0").publish() # channel 0
        self.ampCh1Pub = self.coreTable.getDoubleTopic("ampCh1").publish() # channel 1
        self.ampCh2Pub = self.coreTable.getDoubleTopic("ampCh2").publish() # channel 2
        self.ampCh3Pub = self.coreTable.getDoubleTopic("ampCh3").publish() # channel 3
        self.ampCh4Pub = self.coreTable.getDoubleTopic("ampCh4").publish() # channel 4
        self.ampCh5Pub = self.coreTable.getDoubleTopic("ampCh5").publish() # channel 5
        self.ampCh6Pub = self.coreTable.getDoubleTopic("ampCh6").publish() # channel 6
        self.ampCh7Pub = self.coreTable.getDoubleTopic("ampCh7").publish() # channel 7
        self.ampCh8Pub = self.coreTable.getDoubleTopic("ampCh8").publish() # channel 8
        self.ampCh9Pub = self.coreTable.getDoubleTopic("ampCh9").publish() # channel 9
        self.ampCh10Pub = self.coreTable.getDoubleTopic("ampCh10").publish()  # channel 10
        self.ampCh11Pub = self.coreTable.getDoubleTopic("ampCh11").publish()  # channel 11
        self.ampCh12Pub = self.coreTable.getDoubleTopic("ampCh12").publish()  # channel 12
        self.ampCh13Pub = self.coreTable.getDoubleTopic("ampCh13").publish()  # channel 13
        self.ampCh14Pub = self.coreTable.getDoubleTopic("ampCh14").publish()  # channel 14
        self.ampCh15Pub = self.coreTable.getDoubleTopic("ampCh15").publish()  # channel 15
        self.ampCh16Pub = self.coreTable.getDoubleTopic("ampCh16").publish()  # channel 16
        self.ampCh17Pub = self.coreTable.getDoubleTopic("ampCh17").publish()  # channel 17
        self.ampCh18Pub = self.coreTable.getDoubleTopic("ampCh18").publish()  # channel 18
        self.ampCh19Pub = self.coreTable.getDoubleTopic("ampCh19").publish()  # channel 19
        self.ampCh20Pub = self.coreTable.getDoubleTopic("ampCh20").publish()  # channel 20
        self.ampCh21Pub = self.coreTable.getDoubleTopic("ampCh21").publish()  # channel 21
        self.ampCh22Pub = self.coreTable.getDoubleTopic("ampCh22").publish()  # channel 22
        self.ampCh23Pub = self.coreTable.getDoubleTopic("ampCh23").publish()  # channel 23

        # Generic part 2 code for the actual uploading of telemetry (placed in anyPeriodic)
        # self.xxxPub.set(THE ACTUAL VALUE or BOOLEAN TO PUBLISH)

    def resettingRotationalMotorEncoder(self, rotationalGearRatio_LocalValue):   # Injecting corrected CanCoder value to motor controller
        if self.testbed:    # If on actual robot, the following variable are set by default in robot init above
            rotationalGearRatio_LocalValue = 1  # Gear ratio for MK3: 12.8 . Use 1 for testbed. This overrides prior values for later use.
            self.driveRotPerInch = 1  #  1 for testbed for ease of following rotation

        if self.cancoderAbsent: # This variable is a fudge factor to force following encOffset to 0 when cancoder is absent
            cancoderBinary = 0
        else:
            cancoderBinary = 1

        # Reading CanCoder orientation and injecting gear ratio corrected value to motor controller encoder
        # Phoenix6 syntax. Cancoder now has values 0-1 and not 0-360. Needs .value since API returns StatusSignal and not a float
        self.encOffsetFR = self.cancoderFR.get_absolute_position().value * rotationalGearRatio_LocalValue * cancoderBinary
        self.encOffsetFL = self.cancoderFL.get_absolute_position().value * rotationalGearRatio_LocalValue * cancoderBinary
        self.encOffsetBL = self.cancoderBL.get_absolute_position().value * rotationalGearRatio_LocalValue * cancoderBinary
        self.encOffsetBR = self.cancoderBR.get_absolute_position().value * rotationalGearRatio_LocalValue * cancoderBinary

        # Because the rotation encoders are re-injected and set at each code reboot/re-deployment, in cancoderAbsent mode,
        # wheel rotation orientation need be redone on every boot and code re-deployment!!!
        if self.rotationMotorType == 1:
            self.rotationFR_Enc.setPosition(self.encOffsetFR)
            self.rotationFL_Enc.setPosition(self.encOffsetFL)
            self.rotationBL_Enc.setPosition(self.encOffsetBL)
            self.rotationBR_Enc.setPosition(self.encOffsetBR)

        if self.rotationMotorType == 2:
            self.rotationFR.set_position(self.encOffsetFR)
            self.rotationFL.set_position(self.encOffsetFL)
            self.rotationBL.set_position(self.encOffsetBL)
            self.rotationBR.set_position(self.encOffsetBR)

        return rotationalGearRatio_LocalValue

    # Dirty function that centralizes calls to swerve class and runs the motors to facilitate use in
    # all other run modes, including teleop and autonomous.
    def swerveMasterControl(self, autonomous, speedLimitMultiplier, fieldCentric, autoFaceAngle = 0):
        # autoFaceAngle = 0 in argument allows for 0 to be passed for a non-use scenario in teleop.
        # autoFaceAngle only used in Auton.

        # Passing joystick input to invKinematics for initial processing
        self.swerve.invKinematics(self.joyL_X, self.joyL_Y, self.joyR_X, autonomous, self.gyroNorthReset, self.gyroAngleFinal, fieldCentric, self.autoFaceAngleTurnOnlyMode, autoFaceAngle)

        if self.rotationMotorType == 1:  # REV Neo
            # Running each wheel through the outputAngleNPower function of the SwerveDrive class
            drivePowerFR, targetAngleFR, self.autoFaceAngleComplete = self.swerve.outputAngleNPower("FR", self.rotationFR_Enc.getPosition(), self.cancoderAbsent, self.cancoderFR.get_absolute_position().value)
            self.rotationFR_PID.setReference(targetAngleFR, SparkLowLevel.ControlType.kPosition)  # Just goto desired angle

            drivePowerFL, targetAngleFL, self.autoFaceAngleComplete = self.swerve.outputAngleNPower("FL", self.rotationFL_Enc.getPosition(), self.cancoderAbsent, self.cancoderFL.get_absolute_position().value)
            self.rotationFL_PID.setReference(targetAngleFL, SparkLowLevel.ControlType.kPosition)  # Just goto desired angle

            drivePowerBL, targetAngleBL, self.autoFaceAngleComplete = self.swerve.outputAngleNPower("BL", self.rotationBL_Enc.getPosition(), self.cancoderAbsent, self.cancoderBL.get_absolute_position().value)
            self.rotationBL_PID.setReference(targetAngleBL, SparkLowLevel.ControlType.kPosition)  # Just goto desired angle

            drivePowerBR, targetAngleBR, self.autoFaceAngleComplete = self.swerve.outputAngleNPower("BR", self.rotationBR_Enc.getPosition(), self.cancoderAbsent, self.cancoderBR.get_absolute_position().value)
            self.rotationBR_PID.setReference(targetAngleBR, SparkLowLevel.ControlType.kPosition)  # Just goto desired angle

        if self.rotationMotorType == 2:    # Kraken
            # Running each wheel through the outputAngleNPower function of the SwerveDrive class
            drivePowerFR, targetAngleFR, self.autoFaceAngleComplete = self.swerve.outputAngleNPower("FR",
                                                                                                    self.rotationFR.get_position().value, self.cancoderAbsent, self.cancoderFR.get_absolute_position().value)
            self.rotationFR.set_control(controls.MotionMagicVoltage(targetAngleFR))  # Just goto desired angle

            drivePowerFL, targetAngleFL, self.autoFaceAngleComplete = self.swerve.outputAngleNPower("FL",
                                                                                                    self.rotationFL.get_position().value, self.cancoderAbsent, self.cancoderFL.get_absolute_position().value)
            self.rotationFL.set_control(controls.MotionMagicVoltage(targetAngleFL))  # Just goto desired angle

            drivePowerBL, targetAngleBL, self.autoFaceAngleComplete = self.swerve.outputAngleNPower("BL",
                                                                                                    self.rotationBL.get_position().value, self.cancoderAbsent, self.cancoderBL.get_absolute_position().value)
            self.rotationBL.set_control(controls.MotionMagicVoltage(targetAngleBL))  # Just goto desired angle

            drivePowerBR, targetAngleBR, self.autoFaceAngleComplete = self.swerve.outputAngleNPower("BR",
                                                                                                    self.rotationBR.get_position().value, self.cancoderAbsent, self.cancoderBR.get_absolute_position().value)
            self.rotationBR.set_control(controls.MotionMagicVoltage(targetAngleBR))  # Just goto desired angle

        if self.driveMotorType == 1:
            self.driveFR.set(drivePowerFR * speedLimitMultiplier)
            self.driveFL.set(drivePowerFL * speedLimitMultiplier)
            self.driveBL.set(drivePowerBL * speedLimitMultiplier)
            self.driveBR.set(drivePowerBR * speedLimitMultiplier)

        if self.driveMotorType == 2:
            self.driveFR.set_control(controls.DutyCycleOut(drivePowerFR * speedLimitMultiplier))
            self.driveFL.set_control(controls.DutyCycleOut(drivePowerFL * speedLimitMultiplier))
            self.driveBL.set_control(controls.DutyCycleOut(drivePowerBL * speedLimitMultiplier))
            self.driveBR.set_control(controls.DutyCycleOut(drivePowerBR * speedLimitMultiplier))

    def positionLockMasterControl(self, positionLockOrientation = "universal"):
        self.stopDriving()
        if self.rotationMotorType == 1:  # REV Neo
            # Running each wheel through the outputAngleOnly function of the SwerveDrive class
            targetAngleFR = self.swerve.outputAngleOnly("FR", positionLockOrientation, self.rotationFR_Enc.getPosition(), self.cancoderAbsent, self.cancoderFR.get_absolute_position().value)
            self.rotationFR_PID.setReference(targetAngleFR,
                                             SparkLowLevel.ControlType.kPosition)  # Just goto desired angle

            targetAngleFL = self.swerve.outputAngleOnly("FL", positionLockOrientation, self.rotationFL_Enc.getPosition(), self.cancoderAbsent, self.cancoderFL.get_absolute_position().value)
            self.rotationFL_PID.setReference(targetAngleFL,
                                             SparkLowLevel.ControlType.kPosition)  # Just goto desired angle

            targetAngleBL = self.swerve.outputAngleOnly("BL", positionLockOrientation, self.rotationBL_Enc.getPosition(), self.cancoderAbsent, self.cancoderBL.get_absolute_position().value)
            self.rotationBL_PID.setReference(targetAngleBL,
                                             SparkLowLevel.ControlType.kPosition)  # Just goto desired angle

            targetAngleBR = self.swerve.outputAngleOnly("BR", positionLockOrientation, self.rotationBR_Enc.getPosition(), self.cancoderAbsent, self.cancoderBR.get_absolute_position().value)
            self.rotationBR_PID.setReference(targetAngleBR,
                                             SparkLowLevel.ControlType.kPosition)  # Just goto desired angle

        if self.rotationMotorType == 2:  # Kraken
        # Running each wheel through the outputAngleOnly function of the SwerveDrive class
            targetAngleFR = self.swerve.outputAngleOnly("FR", positionLockOrientation,
                                                        self.rotationFR.get_position().value, self.cancoderAbsent, self.cancoderFR.get_absolute_position().value)
            self.rotationFR.set_control(controls.MotionMagicVoltage(targetAngleFR))  # Just goto desired angle

            targetAngleFL = self.swerve.outputAngleOnly("FL", positionLockOrientation,
                                                        self.rotationFL.get_position().value, self.cancoderAbsent, self.cancoderFL.get_absolute_position().value)
            self.rotationFL.set_control(controls.MotionMagicVoltage(targetAngleFL))  # Just goto desired angle

            targetAngleBL = self.swerve.outputAngleOnly("BL", positionLockOrientation,
                                                        self.rotationBL.get_position().value, self.cancoderAbsent, self.cancoderBL.get_absolute_position().value)
            self.rotationBL.set_control(controls.MotionMagicVoltage(targetAngleBL))  # Just goto desired angle

            targetAngleBR = self.swerve.outputAngleOnly("BR", positionLockOrientation,
                                                        self.rotationBR.get_position().value, self.cancoderAbsent, self.cancoderBR.get_absolute_position().value)
            self.rotationBR.set_control(controls.MotionMagicVoltage(targetAngleBR))  # Just goto desired angle

    def currentVelocity(self, teamBlue = True):
        if self.driveMotorType == 1:
            if teamBlue:
                currentVelocity = self.driveFR_Enc.getVelocity()
            else:
                currentVelocity = self.driveFL_Enc.getVelocity()

        if self.driveMotorType == 2:
            if teamBlue:
                currentVelocity = self.driveFR.get_velocity().value
            else:
                currentVelocity = self.driveFL.get_velocity().value

        return currentVelocity

    def autoDrive(self, angle, power, distance, failoutTime = 20, teamBlue = True):  # Swerve in autonomous with distance traveled controlled by encoder.
        # This includes faceAngle turn if driving is concurrent. For faceTurn only, use autoFaceAngleTurnOnly function
        # Faceangle is actually managed concurrently in the background via global variable
        timesUp = self.autoDriveFailoutTimer.autoTimer(failoutTime)

        self.joyL_X = power * math.sin(math.radians(angle))
        self.joyL_Y = power * math.cos(math.radians(angle))
        # This selection of encoder is important since Blue and Red sides usually have mirrored fields.
        # However in games like 2025 when there no mirroring, just use 1 same encoder.
        # the parameter teamBlue is the default and uses the FR encoder.
        if self.driveMotorType == 1:
            if teamBlue:
                driveEncoder = self.driveFR_Enc.getPosition()
            else:
                driveEncoder = self.driveFL_Enc.getPosition()

        if self.driveMotorType == 2:
            if teamBlue:
                driveEncoder = self.driveFR.get_position().value
            else:
                driveEncoder = self.driveFL.get_position().value

        currentPosition = driveEncoder / self.driveRotPerInch

        if self.newAutoDrive:
            self.initialPosition = currentPosition
            self.newAutoDrive = False

        error = distance - abs(currentPosition - self.initialPosition)
        if error > self.autoDriveEndpointSlop: # timesUp is the "returned boolean" of the failoutTimer
            self.joyR_X = 0  # The 0 for joyR_X will allow autoFaceAngle in inverse kinematics. joyL_X and joyL_Y unchanged from above.
            gotoNextAutonStep = False
            if timesUp:
                self.autoDriveFailoutTimer.autoTimerReset()
                self.stopDriving()
                self.newAutoDrive = True
                self.autoFaceAngleComplete = False
                self.zeroingJoystickVariables()
                gotoNextAutonStep = True
        else:
            self.stopDriving()
            self.newAutoDrive = True
            self.autoDriveFailoutTimer.autoTimerReset() # Here to reset autoTimer instance when no failout occurred
            self.autoFaceAngleComplete = False  # This is to reset autoFaceAngleTurnOnly since if preceded
            # by autoDrive, in some scenarios, this variable may remain True and prevent turnOnly function.
            self.zeroingJoystickVariables() # Needed?
            gotoNextAutonStep = True
        return error, gotoNextAutonStep # next autonStep signal. Error is the distance left in inches

    def autoDriveReset(self):
        self.autoDriveFailoutTimer.autoTimerReset()
        self.newAutoDrive = True
        self.autoFaceAngleComplete = False

    def autoFaceAngleTurnOnly(self, failoutTime = 20):  # **** will need to modify speedmultiplier for the autofaceturn.
        timesUp = self.autoDriveFailoutTimer.autoTimer(failoutTime)

        self.autoFaceAngleTurnOnlyMode = True
        #   This function is hollow and used mainly to reset variables and set self.speedLimitMultiplier. The swerve class simply completes autofaceangle specified
        if self.autoFaceAngleComplete or timesUp:
            self.speedLimitMultiplier = self.speedLimitMultiplierOG
            self.autoFaceAngleTurnOnlyMode = False  # This frees invKinematics for other jobs
            self.autoFaceAngleComplete = False  # This resets the function
            self.autoDriveFailoutTimer.autoTimerReset() # Here to reset autoTimer instance when no failout occurred
            self.zeroingJoystickVariables() # The 0 for joyR_X didn't allow autofaceangleonly in inverse kinematics since left stick was 0.
            gotoNextAutonStep = True
        return gotoNextAutonStep  # next autonStep signal

    def stopDriving(self):
        if self.driveMotorType == 1:
            self.driveFR.set(0)
            self.driveFL.set(0)
            self.driveBL.set(0)
            self.driveBR.set(0)

        if self.driveMotorType == 2:
            self.driveFR.set_control(controls.DutyCycleOut(0))
            self.driveFL.set_control(controls.DutyCycleOut(0))
            self.driveBL.set_control(controls.DutyCycleOut(0))
            self.driveBR.set_control(controls.DutyCycleOut(0))

    # For Limelight tv variable, the output is 0 / 1 and not a boolean. This converts it to one.
    def targetAcquired(self, limelightNum = 1):
        if limelightNum == 1:
            if self.tvLimelightSub.get() == 1:
                lockedOn = True
            else:
                lockedOn = False
        if limelightNum == 2:
            if self.tvLimelightSub2.get() == 1:
                lockedOn = True
            else:
                lockedOn = False

        return lockedOn

    def aprilTagID(self, limelightNum = 1):
        if limelightNum == 1:
            tagID = self.tidLimelightSub.get()
        if limelightNum == 2:
            tagID = self.tidLimelightSub2.get()

        return tagID

    def currentGyroAngleFinal(self):    # Needed? TBD
        return self.gyroAngleFinalPub

    def autoSeek(self, orientationOfAction, virtualStickPower, x_compensation, limelightNum = 1):
        # This is handling the drive direction and not the faceAngle. Autofaceangle is controlled elsewhere.
        # Since temporarily in auton mode and self.joyR_X = 0.
        if limelightNum == 1:
            targetingXaxis =  self.txLimelightSub.get()
        if limelightNum == 2:
            targetingXaxis = self.txLimelightSub2.get()

        # actionAngleCompensation is the side of the robot the seeking is directed from.
        if orientationOfAction == "north":
            actionAngleCompensation = 0
        elif orientationOfAction == "west":
            actionAngleCompensation = -90
        elif orientationOfAction == "south":
            actionAngleCompensation = 180
        elif orientationOfAction == "south":
            actionAngleCompensation = 90

        targetAngle = self.gyroAngleFinal + actionAngleCompensation + (targetingXaxis * x_compensation)

        # Vector calculation derived from swerve code
        self.joyL_X = virtualStickPower * math.sin(math.radians(targetAngle))
        self.joyL_Y = virtualStickPower * math.cos(math.radians(targetAngle))
        self.joyR_X = 0

        return targetAngle  # This is the value used for autoFaceAngle

    def autoSeekNStop(self, orientationOfAction, virtualStickPower, emergencyBrakeDistance, driveAngleFudgeFactor, decelFudgeFactor, limelightNum = 1):
        # x_compensation >1 is compensating to the right.  y_compensation >1 is increasing traveling speed (assumed power). Power is fraction of speedLimitMultiplier.
        if limelightNum == 1:
            targetingXaxis = self.txLimelightSub.get()
            tx3D = self.dataArrayLimelightSub.get()[0]  # Side distance to line perpendicular to plane of target
            tz3D = self.dataArrayLimelightSub.get()[2]  # Distance to plane of target
            Ry3D = self.dataArrayLimelightSub.get()[4]  # Rotation around y-axis (vertical) in plane of target
        if limelightNum == 2:
            targetingXaxis = self.txLimelightSub2.get()
            tx3D = self.dataArrayLimelightSub2.get()[0]
            tz3D = self.dataArrayLimelightSub2.get()[2]
            Ry3D = self.dataArrayLimelightSub2.get()[4]

        # actionAngleCompensation is the side of the robot the seeking is directed from.
        if orientationOfAction == "north":
            actionAngleCompensation = 0
        elif orientationOfAction == "west":
            actionAngleCompensation = -90
        elif orientationOfAction == "south":
            actionAngleCompensation = 180
        elif orientationOfAction == "south":
            actionAngleCompensation = 90

        targetAngle = self.gyroAngleFinal + actionAngleCompensation + targetingXaxis + driveAngleFudgeFactor

        distanceLeft = math.sqrt(tx3D ** 2 + tz3D ** 2)     # in meters
        if distanceLeft > 1:    # 1 meter
            powerMultiplier = 1
        else:
            powerMultiplier = math.sqrt(distanceLeft / decelFudgeFactor)    #   decelFudgeFactor range (1-3)

        totalPower = virtualStickPower * powerMultiplier
        distanceLeftInInches = distanceLeft * 39.3701 # converting meters to inches

        if distanceLeftInInches > emergencyBrakeDistance:
            self.joyL_X = totalPower * math.sin(math.radians(targetAngle))  # totalPower is actually a fraction of powerLimitMultiplier
            self.joyL_Y = totalPower * math.cos(math.radians(targetAngle))
            self.joyR_X = 0
        else:
            self.joyL_X = 0
            self.joyL_Y = 0
            self.joyR_X = 0
            self.ledBlink(True, 0.2)

        return Ry3D, targetAngle, totalPower, distanceLeftInInches

    def readingJoysticks(self, speedLimitMultiplier):  # This is only used in teleop. Right joystick is inversely proportional to net speed.
        self.joyL_X = self.joystick0.getLeftX()
        self.joyL_Y = -self.joystick0.getLeftY()
        maxLeftStickDeflection = max(abs(self.joyL_X), abs(self.joyL_Y))
        self.joyR_X = self.joystick0.getRightX() * (0.4 + (0.6 - (0.6 * speedLimitMultiplier * maxLeftStickDeflection)))

        # New location for this filter. Was in invKinematics. Possibly move back if rotation swerve wheels remain wonky
        if abs(self.joyL_X) <= self.joystickDeadzone:
            self.joyL_X = 0
        if abs(self.joyL_Y) <= self.joystickDeadzone:
            self.joyL_Y = 0
        if abs(self.joyR_X) <= self.joystickDeadzone:  # Essential to control right joystick input at this stage
            self.joyR_X = 0

    def zeroingJoystickVariables(self):
        self.joyL_X = 0
        self.joyL_Y = 0
        self.joyR_X = 0

    def rumbleJoy0(self, rumbleStrength = 0):
        self.joystick0.setRumble(self.joystick0.RumbleType.kLeftRumble, rumbleStrength)
        self.joystick0.setRumble(self.joystick0.RumbleType.kRightRumble, rumbleStrength)

    def rumbleJoy1(self, rumbleStrength = 0):
        self.joystick1.setRumble(self.joystick1.RumbleType.kLeftRumble, rumbleStrength)
        self.joystick1.setRumble(self.joystick1.RumbleType.kRightRumble, rumbleStrength)

    def ledOff(self):
        self.powerDistributionHub.setSwitchableChannel(False)  # LED off

    def ledOn(self):
        self.powerDistributionHub.setSwitchableChannel(True)  # LED on

    def ledBlink(self, blink, durationOn = 1, durationOff = 1):  # in Main robot.py when calling this, in else statement need to reset timer in case no proper failout
        # So would add in that portion of Main code "self.coreFunctionsInstance.ledTimer.autoTimerReset()"
        # In same section, will also need to add "self.coreFunctionsInstance.ledBlinkStepReset()" or will start at where last step left of.
        if blink:
            self.ledBlinkReset = True
            if self.ledBlinkStep == 1:
                self.ledOn()
                timesUp = self.ledTimer.autoTimer(durationOn)
                if timesUp:
                    self.ledBlinkStep += 1
            if self.ledBlinkStep == 2:
                self.ledOff()
                timesUp = self.ledTimer.autoTimer(durationOff)
                if timesUp:
                    self.ledBlinkStep += 1
            if self.ledBlinkStep == 3:
                self.ledBlinkStep = 1
        else:
            if self.ledBlinkReset:  # This single reset prevent ledOff conflict with other LED calls
                self.ledBlinkReset = False
                self.ledOff()
                self.ledBlinkStep = 1
                self.ledTimer.autoTimerReset()
            else:
                pass

    def gyroAngleAdjustments(self, autonomous, gyroCompensation, resetNorthActivated = False):
        if self.navxAttached:
            gyroAngle = self.navx.getAngle()
        else:
            gyroAngle = 0
        gyroAngleCompensated = gyroAngle + gyroCompensation

        if not autonomous:
            self.resetNorth(resetNorthActivated, gyroAngleCompensated)

        self.gyroAngleFinal = gyroAngleCompensated + self.gyroResetConstant


    def resetNorth(self, resetNorthActivated, gyroAngleCompensated):
        if resetNorthActivated:
            self.gyroNorthReset = True  # This is needed for resetting the initialFaceAngle after gyro reset
            fractionGyroAngleCompensated, wholeGyroAngleCompensated = math.modf(gyroAngleCompensated / 360)
            if gyroAngleCompensated >= 0:
                midCircle = 180
                circleCompensation1 = 0
                circleCompensation2 = 1
            else:
                midCircle = -180
                circleCompensation1 = -1
                circleCompensation2 = 0
            if gyroAngleCompensated <= wholeGyroAngleCompensated * 360 + midCircle:
                self.gyroResetConstant = (wholeGyroAngleCompensated + circleCompensation1) * 360 - gyroAngleCompensated
            else:
                self.gyroResetConstant = (wholeGyroAngleCompensated + circleCompensation2) * 360 - gyroAngleCompensated

        else:
            self.gyroNorthReset = False

    def driveTelemetry(self):
        if self.driveMotorType == 1:
            self.driveFR_EncPub.set(self.driveFR_Enc.getPosition())
            self.driveFL_EncPub.set(self.driveFL_Enc.getPosition())
            self.driveBL_EncPub.set(self.driveBL_Enc.getPosition())
            self.driveBR_EncPub.set(self.driveBR_Enc.getPosition())
        if self.driveMotorType == 2:
            self.driveFR_EncPub.set(self.driveFR.get_position().value)
            self.driveFL_EncPub.set(self.driveFL.get_position().value)
            self.driveBL_EncPub.set(self.driveBL.get_position().value)
            self.driveBR_EncPub.set(self.driveFR.get_position().value)

        if self.rotationMotorType == 1:
            self.rotationFR_EncPub.set(self.rotationFR_Enc.getPosition())
            self.rotationFL_EncPub.set(self.rotationFL_Enc.getPosition())
            self.rotationBL_EncPub.set(self.rotationBL_Enc.getPosition())
            self.rotationBR_EncPub.set(self.rotationBR_Enc.getPosition())
        if self.rotationMotorType == 2:
            self.rotationFR_EncPub.set(self.rotationFR.get_position().value)
            self.rotationFL_EncPub.set(self.rotationFL.get_position().value)
            self.rotationBL_EncPub.set(self.rotationBL.get_position().value)
            self.rotationBR_EncPub.set(self.rotationBR.get_position().value)

    def cancoderTelemetry(self):
        self.cancoderFRPub.set(self.cancoderFR.get_absolute_position().value)
        self.cancoderFLPub.set(self.cancoderFL.get_absolute_position().value)
        self.cancoderBLPub.set(self.cancoderBL.get_absolute_position().value)
        self.cancoderBRPub.set(self.cancoderBR.get_absolute_position().value)

    def navxTelemetry(self):
        self.gyroAngleFinalPub.set(self.gyroAngleFinal)

    def limelightTelemetry(self):
        self.txPub.set(self.txLimelightSub.get())
        self.tyPub.set(self.tyLimelightSub.get())
        self.taPub.set(self.taLimelightSub.get())
        self.tvPub.set(self.tvLimelightSub.get())
        self.tidPub.set(self.tidLimelightSub.get())

        self.txFromDataArrayPub.set(self.dataArrayLimelightSub.get()[0])
        self.tzFromDataArrayPub.set(self.dataArrayLimelightSub.get()[2])
        self.RyFromDataArrayPub.set(self.dataArrayLimelightSub.get()[4])
        self.targetDistancePub.set(math.sqrt(self.dataArrayLimelightSub.get()[0] ** 2 + self.dataArrayLimelightSub.get()[2] ** 2))

    def limelight2Telemetry(self):
        self.txPub2.set(self.txLimelightSub2.get())
        self.tyPub2.set(self.tyLimelightSub2.get())
        self.taPub2.set(self.taLimelightSub2.get())
        self.tvPub2.set(self.tvLimelightSub2.get())
        self.tidPub2.set(self.tidLimelightSub2.get())

        self.txFromDataArrayPub2.set(self.dataArrayLimelightSub2.get()[0])
        self.tzFromDataArrayPub2.set(self.dataArrayLimelightSub2.get()[2])
        self.RyFromDataArrayPub2.set(self.dataArrayLimelightSub2.get()[4])
        self.targetDistancePub2.set(math.sqrt(self.dataArrayLimelightSub2.get()[0] ** 2 + self.dataArrayLimelightSub2.get()[2] ** 2))

    def amperageTelemetry(self):
        self.ampCh0Pub.set(self.powerDistributionHub.getCurrent(0))
        self.ampCh1Pub.set(self.powerDistributionHub.getCurrent(1))
        self.ampCh2Pub.set(self.powerDistributionHub.getCurrent(2))
        self.ampCh3Pub.set(self.powerDistributionHub.getCurrent(3))
        self.ampCh4Pub.set(self.powerDistributionHub.getCurrent(4))
        self.ampCh5Pub.set(self.powerDistributionHub.getCurrent(5))
        self.ampCh6Pub.set(self.powerDistributionHub.getCurrent(6))
        self.ampCh7Pub.set(self.powerDistributionHub.getCurrent(7))
        self.ampCh8Pub.set(self.powerDistributionHub.getCurrent(8))
        self.ampCh9Pub.set(self.powerDistributionHub.getCurrent(9))
        self.ampCh10Pub.set(self.powerDistributionHub.getCurrent(10))
        self.ampCh11Pub.set(self.powerDistributionHub.getCurrent(11))
        self.ampCh12Pub.set(self.powerDistributionHub.getCurrent(12))
        self.ampCh13Pub.set(self.powerDistributionHub.getCurrent(13))
        self.ampCh14Pub.set(self.powerDistributionHub.getCurrent(14))
        self.ampCh15Pub.set(self.powerDistributionHub.getCurrent(15))
        self.ampCh16Pub.set(self.powerDistributionHub.getCurrent(16))
        self.ampCh17Pub.set(self.powerDistributionHub.getCurrent(17))
        self.ampCh18Pub.set(self.powerDistributionHub.getCurrent(18))
        self.ampCh19Pub.set(self.powerDistributionHub.getCurrent(19))
        self.ampCh20Pub.set(self.powerDistributionHub.getCurrent(20))
        self.ampCh21Pub.set(self.powerDistributionHub.getCurrent(21))
        self.ampCh22Pub.set(self.powerDistributionHub.getCurrent(22))
        self.ampCh23Pub.set(self.powerDistributionHub.getCurrent(23))

class NiftyTimer:
    def __init__(self):
        self.newTimer = wpilib.Timer()
        self.newTimer.start()
        self.newTiming = True
        self.timesUp = False

    def autoTimer(self, desiredTime):
        if self.newTiming:
            self.newTimer.reset()
            self.newTiming = False
            self.timesUp = False
        if self.newTimer.get() > desiredTime:
            self.newTiming = True
            self.timesUp = True

        return self.timesUp

    def autoTimerReset(self):
        self.newTiming = True

class RateOfChange:
    def __init__(self):
        self.speedDeltaDetectionTimer = NiftyTimer()
        self.speedDeltaDetectionStep = 1
        self.currentSpeed = 0   # A priming variable to prevent a crash

    def speedDeltaDetection(self, decelMode, tolerance, rampUpAllowanceTime, currentSpeed):
        # decelMode is Boolean for whether detecting deceleration or acceleration
        # tolerance range is > 0 and in %. It is the % change allowed in 1/50 of a sec before triggering a detection
        # rampUpAllowanceTime is time allowed in seconds to ignore velocity readings during motor rampUp. Cannot be 0
        # This function should be called at the start of motor activation

        impact = False  # Default primer to be updated downstream
        if self.speedDeltaDetectionStep == 1:
            timesUp = self.speedDeltaDetectionTimer.autoTimer(rampUpAllowanceTime)
            if timesUp:
                self.speedDeltaDetectionStep += 1
        if self.speedDeltaDetectionStep == 2:
            self.lastSpeed = self.currentSpeed
            self.currentSpeed = currentSpeed  # This will keep self.currentSpeed fresh for later transfer to self.lastSpeed
            if self.lastSpeed != 0:  # This prevents division by 0 for self.speedDeltaPercentage calculation
                self.speedDeltaPercentage = (self.currentSpeed - self.lastSpeed) / self.lastSpeed
                if decelMode:
                    if (self.speedDeltaPercentage < - tolerance / 100):  # Only if detectionActivated means sensing
                        # only in forward direction and also prevents sensing of natural decel when button released.
                        impact = True
                else:  # Accel mode
                    if (self.speedDeltaPercentage > tolerance / 100):  # Only if detectionActivated means sensing
                        # only in forward direction and also prevents sensing of natural decel when button released.
                        impact = True

        return impact

    def speedDeltaDetectionReset(self):
        self.speedDeltaDetectionStep = 1
        self.speedDeltaDetectionTimer.autoTimerReset()



