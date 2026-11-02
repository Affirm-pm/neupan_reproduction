# External control signal for a pipeline node (ControlLayer).
# START activates the node for the next episode; STOP halts it.
# New control signals can be added by extending this enum.
uint8 START = 0
uint8 STOP  = 1

uint8 cmd
