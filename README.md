TODO:

Alpha bot:
- [x] Test swerve drive
- [x] Test turret function
- [x] Tune slippage current
- [x] Test cameras & filters
- [x] Test intake command
- [x] Test wrist CANCoder offset
- [x] Tune elevator
- [x] Test pivot gravity gains
- [x] Remove voltage constraints if everything works
- [x] Test manual controls
- [x] Test presets
- [x] Swerve automation pt. 1 - rotational alignment

Friday, 2/14/2025 to test:
- [x] Retune wrist feedforward + feedback
- [x] Wrist presets + tuning
- [x] Pivot cancoder issues
- [x] Decide on L1 position
- [x] Tune Canandcolor for intake (coral)
- [x] Elevator limit switch

Friday, 2/14/2025 software only:
- [x] Driver only - work on OnFalse triggers instead of OnTrue because of debounce hold to pathfind/direct move to position then OnFalse will subsequently deploy the state properly and effectively
- [x] Double click debouncer
- [x] Move to algae vs coral (drive to object overhaul)
- [x] Swerve automation pt. 2 - pathfinding/translation alignment

Saturday, 2/15/2025 to test:
- [x] Test robot state chooser

Saturday, 2/15/2025 software only:
- [x] Auto - java structure (chooser), paths (pathmanager), commands

Monday, 2/17/2025 to test:
- [ ] Fix pivot cancoder issues
- [ ] Faster wrist
- [ ] Redo L4 preset as needed
- [ ] Test localization trust filter
- [ ] Retest pathfinding to reef, taking into account tolerances and pids
- [ ] Test multiple chooser
- [ ] Preliminary auto things
- [ ] Decide on starting heading of robot

Monday, 2/17/2025 software only:
- [ ] finish paths
- [ ] check for correct event markers & tune locations of each marker
- [ ] perfect/tune the control points (consistent in transitioning between paths)
- [ ] coral station optimization

Comp bot/later:
- [ ] Tune Canandcolor for intake (both coral & algae)
- [ ] Implement climbing
- [ ] SysID for elevator, pivot, wrist
- [ ] Fine tune state transitions for efficiency
- [ ] Test ground intake/object detection