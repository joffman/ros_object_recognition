# Types.
uint8 PRESS	= 0
uint8 MOVE	= 1
uint8 RELEASE	= 2

# Buttons.
uint8 NO_BUTTON		= 0
uint8 LEFT_BUTTON	= 1
uint8 MIDDLE_BUTTON	= 2
uint8 RIGHT_BUTTON	= 3
uint8 UNKNOWN_BUTTON	= 4

uint8 type
uint8 button	# button that caused the event (== NO_BUTTON for MOVE)

# Mouse position when event was generated.
object_detection_2d/Point2D position
