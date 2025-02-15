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
- [ ] Move to algae vs coral (drive to object overhaul)
- [ ] Swerve automation pt. 2 - pathfinding/translation alignment
- [ ] Auto - java structure first, then paths, then test with other mechanisms

Saturday, 2/15/2025 to test:
- [ ] Test robot state chooser
- [ ] Test localization trust filter

Comp bot/later:
- [ ] Tune Canandcolor for intake (both coral & algae)
- [ ] Implement climbing
- [ ] SysID for elevator, pivot, wrist
- [ ] Fine tune state transitions for efficiency