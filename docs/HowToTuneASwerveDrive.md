# Steps to Tune a Swerve Drive
- Follow CTRe Swerve drive creation
    - this tunes CANcoder offsets
- tune drive/steer PIDs on carpet with weight
- switch to closed-loop voltage control (if not using it already)
    - youc an start with open-loop voltage control for initial testing
    - switch to closed-loop after basic PIDs are tuned
- Test wheel slippage current threshold using [this guide](https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html).
    - add stator current limit to our TunerConstants for drive motors.
- test drive odometry distance calculations. slowly drive various distances and compare measurements, real-world vs drive odometry. debug/adjust as needed. possible reasons for deviation:
    - wheel diameter
    - gear ratio
- Perform system identification (sysId) on the drive base
    - CTRe swerve generator creates sysId functions automatically
    - sysId docs:
        - CTRe: https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/wpilib-integration/sysid-integration/index.html
        - FRC: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html
- things that need tuning:
    - Tuner Constants
        - drive/steer PID (gains), this may have already been done above
        - kSpeedAt12Volts: max robot speed
            - emperically test to determine correct value
        - kSlipCurrent: current at which wheels slip
            - Test wheel slippage current threshold using [this guide](https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html).
    - FieldCentricAutoPoint
        - headingPID: PID loop for auto-point feature
    - AutoPilot (AP)
        - acceleration constraint: max amount robot can acceleration in AP
        - AP heading PID
        - theta controller PID
        - theta end controller PID
    - P2P
        - similar to AP
    - 


### Questions
- is there a way to test/measure gyro drift over time?