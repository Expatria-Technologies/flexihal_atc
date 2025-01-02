# flexihal_atc

Automatic tool change plugin for [grblHal](https://www.grbl.org/what-is-grblhal) based on the plugin code from rcp1 and rvlotta.

## Usage

Thus far these commands are implemented:

"$DRBO" opens the drawbar.
"$DRBC" closes the drawbar.

Plugin does not allow the drawbar to operate outside of IDLE or TOOL_CHANGE states.  Plugin does not allow the spindle to start when the drawbar is open.  Plugin does not allow the drawbar to open when the spindle is running.

## Todo

- [X] Add commands to clamp and unclamp the drawbar.
- [ ] Tool setter via built-in probing functions, not new ones.
- [ ] Ensuring correct interaction with persistent TLO
- [ ] Tool Table support
- [ ] Chip cover
- [ ] Allow other orientations / axis of magazine than Z axis to load / unload
