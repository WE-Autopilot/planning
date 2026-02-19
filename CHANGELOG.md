# Changelog

## v0.1 | Initial Dev

### v0.1.0 - Unknown
- [x] Plans hard coded path (no signs)
- [x] Constant speed around the path

### v0.1.1 - Feb 18, 2026
"Open navigation"
- [x] Remove unnecessary "turn angle" and "vehicle speed" subscriptions
- [x] Subscribe to /mapping/lanes instead of hard coding
- [x] Automatically navigate curves while staying on target
- [x] Removed support for "location" waypoint navigation

### v0.1.2 - WIP
"Stop Sign navigation"
- [ ] Stop at stop signs and continue successfully

### v0.1.3 - WIP
"Controlled navigation"
- [ ] Navigate to a determined position safely.
- [ ] Cleanly handle end-of-path situations (see loop path @ end)