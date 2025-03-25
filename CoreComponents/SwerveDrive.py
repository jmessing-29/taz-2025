#warning: NO TOUCHIE TOUCHIE. THIS IS BLACK MAGIC AND DANGEROUS TO TOUCH!!!

import math
class SwerveDrive:
    """to use this class, essential to define robotWidth, robotLength, and rotationalGearRatio in Main code below imports.
    They are needed to be passed as global variables into this class in the most non-Pythonic way! """

    def __init__(self, robotWidth, robotLength, rotationalGearRatio, joystickDeadzone, gyroDeadzone):
        self.robotWidth = robotWidth
        self.robotLength = robotLength
        self.robotRadius = math.sqrt(self.robotWidth ** 2 + self.robotLength ** 2)  # hypotenuse
        self.rotationalGearRatio = rotationalGearRatio
        self.joystickDeadzone = joystickDeadzone
        self.gyroDeadzone = gyroDeadzone   # in degrees
        self.newFaceAngle = True
        self.autoFaceAngleComplete = False
        self.gyroProportion = 0.0135  # was 0.02  # a constant to multiply auton rotational error to achieve proper joyX response for correction
        self.paralysis = 0.00175  # Antiquated constant. The analogue of joystickdeadzone but in auton. (2-21-2025 May need to change to tighten up Auton)

    def invKinematics(self, joyL_X, joyL_Y, joyR_X, autonomous, gyroNorthReset, gyroAngleFinal, fieldCentric, autoFaceAngleTurnOnlyMode, autoFaceAngle):  # function to calculate the rotation and drive power
        # Filtering out the deadzone for manual driving input.
        # vectorThreshold is point when drive system is made inactive so not jump to unpredicted position at deadzone

        #*** this was heavily modified in 1/25 and may need be reverted since manualDriving was depricated.
        # ManualDriving was also removed from essential defaults. Logic prompted this since autonomous = not manualDriving and
        # vice Versa. This means only the variable autonomous was needed and manualDriving was redundant.

        #   The following simply defines the angle-lock angle in both teleop and auton.
        if autonomous:   # includes auton face rotation/angle lock and limelight targeting
            self.vectorThreshold = self.paralysis
            self.initialFaceAngle = autoFaceAngle # the set autoFaceAngle overrides the initialFaceAngle
        else:   # includes face angle lock while manually driving
            self.vectorThreshold = self.joystickDeadzone
            if self.newFaceAngle:  # to update the angle the robot is facing
                self.initialFaceAngle = gyroAngleFinal
                self.newFaceAngle = False

        # For field-centric commands. The gyro angle is injected at this stage for compensation. Rotational element
        # not included and therefore not effected. Otherwise pure rotation cannot be maintained when gyro angle is changed.
        # Important to convert the gyroAngleFinal in degrees to radians by * it by "math.pi / 180" or use math.radians(degrees). Python math trig in radians.
        if fieldCentric:
            joyL_Y_orig = joyL_Y
            joyL_Y = joyL_Y * math.cos(math.radians(gyroAngleFinal)) + joyL_X * math.sin(math.radians(gyroAngleFinal))
            joyL_X = -joyL_Y_orig * math.sin(math.radians(gyroAngleFinal)) + joyL_X * math.cos(math.radians(gyroAngleFinal))

        # This filter assures robot face direction lock only active when manual right stick rotation not performed.
        # Sensor guided turns (e.g. Limelight) is not effected since right stick rotation will be greater than 0.
        # Limelight initiated rotation will manipulate the virtual joyR_X separately.
        # This following turn control will also work for autonomous with manualDriving set to False,
        # since manualDriving == True would enter loop above to filter for deadzone and therefore
        # right stick speed less than deadzone will not be face direction corrected. Finer control is therefore allowed.

        #   Previously, autonomous argument prevented seeking rotation from functioning in auton since target angle locked and not refreshable.
        #   This is good for autoFaceAngleTurnOnly. Argument change to autoFaceAngleTurnOnlyMode to allow for rotational movement for other jobs in auton.
        if (joyR_X == 0 and (joyL_X != 0 or joyL_Y != 0) and not gyroNorthReset) or autoFaceAngleTurnOnlyMode:
            # to maintain the angle the robot it's facing if right joystick not moved or for auton
            # gyroNorthReset later added so after gyro reset, a new faceangle will be set so robot wont act on last saved faceangle!!!!!
            # Was "abs(joyR_X) <= self.joystickDeadzone" before. Now == 0 since up top any right stick less than deadzone is 0 anyways.
            # Now, auton and virtual right stick entered of less than deadzone will not enter loop and be negated if manualDriving is True
            # This joyR_X variable constantly jumps from 0 and what it defines below to be fed further down
            # This may be confusing but this is also the way for auton to enter this loop with joyR_X = 0.

            error = gyroAngleFinal - self.initialFaceAngle
            # The new resulting joyR_X can be greater than the original and still enter this argument since the new joyR_X
            # is passed down to do it's deed while the new incoming loop joyR_X value is fresh and raw from the joystick.
            if abs(error) > self.gyroDeadzone:
                joyR_X = -(error * self.gyroProportion)
                if abs(joyR_X) > 1:   # So joyR_X remains between -1 and 1.
                    joyR_X = joyR_X / abs(joyR_X)

                self.autoFaceAngleComplete = False  # Only important for autoFaceAngleTurnOnly and not autoDrive
                # since in autoDrive faceAngle is constantly adjusted and maintained
            else:
                joyR_X = 0  # This is all teleop and manual driving uses for constant adjustments
                self.autoFaceAngleComplete = True   # Target face angle reached
        else:
            self.newFaceAngle = True # to continue renewal of the angle the robot is facing in teleop
            # This is also refreshed after gyro reset button pressed so a new faceangle and not old one is acted upon

        # X and Y component of straight and rotational vectors combined for calculating inverse kinematics
        self.A_Xnet1 = joyL_X - joyR_X * (self.robotLength / self.robotRadius)
        self.B_Xnet2 = joyL_X + joyR_X * (self.robotLength / self.robotRadius)
        self.C_Ynet1 = joyL_Y - joyR_X * (self.robotWidth / self.robotRadius)
        self.D_Ynet2 = joyL_Y + joyR_X * (self.robotWidth / self.robotRadius)

        # Calculating wheel speed
        self.wheelSpeedFR = math.sqrt(self.B_Xnet2 ** 2 + self.C_Ynet1 ** 2)
        self.wheelSpeedFL = math.sqrt(self.B_Xnet2 ** 2 + self.D_Ynet2 ** 2)
        self.wheelSpeedBL = math.sqrt(self.A_Xnet1 ** 2 + self.D_Ynet2 ** 2)
        self.wheelSpeedBR = math.sqrt(self.A_Xnet1 ** 2 + self.C_Ynet1 ** 2)

        # Finding Max wheel speed and normalizing so not surpass 1.0
        self.wheelSpeedMax = max(self.wheelSpeedFR, self.wheelSpeedFL, self.wheelSpeedBL, self.wheelSpeedBR)
        if self.wheelSpeedMax > 1:
            self.wheelSpeedFR = self.wheelSpeedFR / self.wheelSpeedMax
            self.wheelSpeedFL = self.wheelSpeedFL / self.wheelSpeedMax
            self.wheelSpeedBL = self.wheelSpeedBL / self.wheelSpeedMax
            self.wheelSpeedBR = self.wheelSpeedBR / self.wheelSpeedMax

        # Calculating wheel angles- modified atan2 function to account for 0 deg at north and CW being positive
        #  math.atan2(-X,-Y)*(180/math.pi) + 180
        self.wheelAngleFR = math.atan2(-self.B_Xnet2, -self.C_Ynet1) * (180 / math.pi) + 180
        self.wheelAngleFL = math.atan2(-self.B_Xnet2, -self.D_Ynet2) * (180 / math.pi) + 180
        self.wheelAngleBL = math.atan2(-self.A_Xnet1, -self.D_Ynet2) * (180 / math.pi) + 180
        self.wheelAngleBR = math.atan2(-self.A_Xnet1, -self.C_Ynet1) * (180 / math.pi) + 180

    def outputAngleNPower(self, wheelLocation, rotationEnc, cancoderAbsent, cancoderAngle):
        # to ensure netDirAngle position always from 0 to 0.99 and never negative
        # net angle is post compensating for gyro heading. wheelAngle is in degrees. wheelAngle0to1 in 0-1(post conversion)
        if wheelLocation == "FR":
            wheelAngle = self.wheelAngleFR
            wheelSpeed = self.wheelSpeedFR
        elif wheelLocation == "FL":
            wheelAngle = self.wheelAngleFL
            wheelSpeed = self.wheelSpeedFL
        elif wheelLocation == "BL":
            wheelAngle = self.wheelAngleBL
            wheelSpeed = self.wheelSpeedBL
        elif wheelLocation == "BR":
            wheelAngle = self.wheelAngleBR
            wheelSpeed = self.wheelSpeedBR

        wheelAngle0to1 = wheelAngle/360  # to convert 360 degree format into decimal format
                                        # This angle is where the wheel rotation should be

        # to ensure desired position always from 0 to 0.99 and never negative
        fractionNetDirAngle, wholeNetDirAngle = math.modf(wheelAngle0to1)

        if wheelAngle0to1 >= 0:
            wheelAngle0to1 = fractionNetDirAngle #retrieving the fraction portion of self.wheelAngle0to1 only
        else:
            wheelAngle0to1 = 1 + fractionNetDirAngle

        # This is the current angle of the wheel rotation
        if cancoderAbsent:
            # to ensure encoder position always from 0 to 0.99 and never negative
            fractionEnc, wholeEnc = math.modf(rotationEnc / self.rotationalGearRatio)

            if rotationEnc >= 0:
                encPosition = fractionEnc  #retrieving the fraction portion of encoder position only
            else:
                encPosition = 1 + fractionEnc
        else:   # Wheel angle position puled directly from cancoder
            encPosition = cancoderAngle

        # modifying the 2 angle sets so can be compared
        # New simplified code for comparison. For encoder positions more or less than 0.5, each are potentially
        # difficult to compare zones near 0 representing the destination angles. This simply processes those zones
        # to make them now comparable.
        if encPosition < 0.5:
            if wheelAngle0to1 >= encPosition + 0.5: # Not need "and wheelAngle0to1 < 1" since it's always the case
                wheelAngle0to1 = wheelAngle0to1 - 1
        else:
            if wheelAngle0to1 < encPosition - 0.5:  #  Not need "and wheelAngle0to1 >= 0" since it's always the case
                wheelAngle0to1 = wheelAngle0to1 + 1

        # New simplified code for determining most efficient (least) angle to reach needed wheelAngle and resulting power direction
        # includes wheelAngle turning commands (via PID) and wheel power direction
        if wheelSpeed >= self.vectorThreshold:  # swerve unit functions only when input threshold (joystickDeadzone) reached
            if abs(encPosition - wheelAngle0to1) <= 0.25:  # only when desired rotation within +/- 90 deg
                drivePower = wheelSpeed  # real drive is forward
                targetAngle = rotationEnc - (
                            self.rotationalGearRatio * (encPosition - wheelAngle0to1))  # Accounting for gear ratio
            else:
                drivePower = -wheelSpeed  # real drive is backwards
                if encPosition < wheelAngle0to1:
                    indirectAngle = 0.5 + (encPosition - wheelAngle0to1)
                elif encPosition > wheelAngle0to1:
                    indirectAngle = (encPosition - wheelAngle0to1) - 0.5
                else:
                    indirectAngle = 0

                targetAngle = rotationEnc - (self.rotationalGearRatio * indirectAngle)  # Accounting for gear ratio
        else:
            drivePower = 0
            targetAngle = rotationEnc

        return drivePower, targetAngle, self.autoFaceAngleComplete  # self.autoFaceAngleComplete of swerveClass is ejected out

    def outputAngleOnly(self, wheelLocation, positionLockOrientation, rotationEnc,  cancoderAbsent, cancoderAngle):
        # to ensure netDirAngle position always from 0 to 0.99 and never negative
        if wheelLocation == "FR":
            if positionLockOrientation == "north":
                wheelAngle = 45
            elif positionLockOrientation == "east":
                wheelAngle = 45
            elif positionLockOrientation == "south":
                wheelAngle = 90
            elif positionLockOrientation == "west":
                wheelAngle = 0
            else:
                wheelAngle = 45
        elif wheelLocation == "FL":
            if positionLockOrientation == "north":
                wheelAngle = 135
            elif positionLockOrientation == "east":
                wheelAngle = 0
            elif positionLockOrientation == "south":
                wheelAngle = 90
            elif positionLockOrientation == "west":
                wheelAngle = 135
            else:
                wheelAngle = 135
        elif wheelLocation == "BL":
            if positionLockOrientation == "north":
                wheelAngle = 90
            elif positionLockOrientation == "east":
                wheelAngle = 0
            elif positionLockOrientation == "south":
                wheelAngle = 45
            elif positionLockOrientation == "west":
                wheelAngle = 45
            else:
                wheelAngle = 45
        elif wheelLocation == "BR":
            if positionLockOrientation == "north":
                wheelAngle = 90
            elif positionLockOrientation == "east":
                wheelAngle = 135
            elif positionLockOrientation == "south":
                wheelAngle = 135
            elif positionLockOrientation == "west":
                wheelAngle = 0
            else:
                wheelAngle = 135

        wheelAngle0to1 = wheelAngle/360  # to convert 360 degree format into decimal format

        # to ensure desired position always from 0 to 0.99 and never negative
        fractionNetDirAngle, wholeNetDirAngle = math.modf(wheelAngle0to1)

        if wheelAngle0to1 >= 0:
            wheelAngle0to1 = fractionNetDirAngle #retrieving the fraction portion of self.wheelAngle0to1 only
        else:
            wheelAngle0to1 = 1 + fractionNetDirAngle

        # This is the current angle of the wheel rotation
        if cancoderAbsent:
            # to ensure encoder position always from 0 to 0.99 and never negative
            fractionEnc, wholeEnc = math.modf(rotationEnc / self.rotationalGearRatio)

            if rotationEnc >= 0:
                encPosition = fractionEnc  #retrieving the fraction portion of encoder position only
            else:
                encPosition = 1 + fractionEnc
        else:  # Wheel angle position puled directly from cancoder
            encPosition = cancoderAngle

        # modifying the 2 angle sets so can be compared
        # New simplified code for comparison. For encoder positions more or less than 0.5, each have a potentially
        # difficult to compare zones near 0 representing the destination angles. This simply processes those zones
        # to make them now comparable.
        if encPosition < 0.5:
            if wheelAngle0to1 >= encPosition + 0.5: # Not need "and wheelAngle0to1 < 1" since it's always the case
                wheelAngle0to1 = wheelAngle0to1 - 1
        else:
            if wheelAngle0to1 < encPosition - 0.5:  # Not need "and wheelAngle0to1 >= 0" since it's always the case:
                wheelAngle0to1 = wheelAngle0to1 + 1

        # New simplified code for determining most efficient (least) angle to reach needed wheelAngle

        if abs(encPosition - wheelAngle0to1) <= 0.25:  # only when desired rotation within +/- 90 deg
            targetAngle = rotationEnc - (
                        self.rotationalGearRatio * (encPosition - wheelAngle0to1))  # Accounting for gear ratio
        else:
            if encPosition < wheelAngle0to1:
                indirectAngle = 0.5 + (encPosition - wheelAngle0to1)
            elif encPosition > wheelAngle0to1:
                indirectAngle = (encPosition - wheelAngle0to1) - 0.5
            else:
                indirectAngle = 0

            targetAngle = rotationEnc - (self.rotationalGearRatio * indirectAngle)  # Accounting for gear ratio

        return targetAngle