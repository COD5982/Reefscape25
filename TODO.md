# TODO

## Priority A
  [x] Add subsystems/controls for `Intake` (will be similar to `Cageclimb` and `Runcage`)
  [x] Spark max, brushed motor, on/off
[X] Position control for `Armnudge`/`Arm`
  [x] Tune min/max position values in `Arm`
  [x] Tune min/max power in `Arm`
  [X] Tune PID in `Arm`
  [X] Tune nudge multiplier in `Armnudge`
  [X] Tune nudge multiplier in `Liftnudge`
  [X] Validate driving works
  [ ] Decide if using field relative
    [ ] If yes, add NavX to robot 
    [ ] If yes, verify `AHRS` in `DriveTrain` is correct port (probably should be using SPI instead of USB)
    [ ] If yes, validate using shuffleboard field that orientation works correctly
  [ ] Adjust lift positions in `Lift` to reach proper spots on net/reef

## Priority B

[ ] Version control - Do another git commit
[ ] Version control - Push to github
[ ] Shuffleboard - Add more troubleshooting diagnostics for each subsystem/component
[ ] Path Planner Autons
  [x] DriveTrain - Add `AutoBuilder` / `RobotConfig`
  [ ] Re-enable `autoChooser` in `RobotContainer`
  [ ] Register commands - https://pathplanner.dev/pplib-named-commands.html
  [ ] Build autons in path planner