# TODO

## Priority A
  [x] Add subsystems/controls for `Intake` (will be similar to `Cageclimb` and `Runcage`)
  [x] Spark max, brushed motor, on/off
[x] Position control for `Armnudge`/`Arm`
  [x] Tune min/max position values in `Arm`
  [x] Tune min/max power in `Arm`
  [x] Tune PID in `Arm`
  [x] Tune nudge multiplier in `Armnudge`
  [x] Tune nudge multiplier in `Liftnudge`
  [x] Validate driving works
  [ ] Decide if using field relative
    [x] If yes, add NavX to robot 
    [x] If yes, verify `AHRS` in `DriveTrain` is correct port (probably should be using SPI instead of USB)
    [ ] If yes, validate using shuffleboard field that orientation works correctly
  [x] Adjust lift positions in `Lift` to reach proper spots on net/reef

## Priority B

[x] Version control - Do another git commit
[x] Version control - Push to github
[x] Shuffleboard - Add more troubleshooting diagnostics for each subsystem/component
[ ] Path Planner Autons
  [x] DriveTrain - Add `AutoBuilder` / `RobotConfig`
  [x] Re-enable `autoChooser` in `RobotContainer`
  [x] Register commands - https://pathplanner.dev/pplib-named-commands.html
  [x] Build autons in path planner