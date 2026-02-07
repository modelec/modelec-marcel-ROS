string DELIMITER = ";"

# Action
string UP = "UP"
string DOWN = "DOWN"
string TOGGLE_ARM = "TOGGLE_ARM"
string ROTATE_ARM = "ROTATE_ARM"
string TAKE = "TAKE"
string FREE = "FREE"
string TOGGLE_SERVO = "TOGGLE_SERVO"
string THERMO = "THERMO"
string LOOK_ON = "LOOK_ON"
string MAX_DEPLOY = "MAX_DEPLOY"

# Step
int32 DONE_STEP = 0
int32 UP_STEP = 1
int32 DOWN_STEP = 2
int32 TAKE_STEP = 3
int32 FREE_STEP = 4
int32 THERMO_STEP = 5
int32 TOGGLE_SERVO_STEP = 7
int32 TOGGLE_ARM_STEP = 8
int32 ROTATE_ARM_STEP = 9
int32 LOOK_ON_STEP = 10
int32 MAX_DEPLOY_STEP = 99

int32[] steps
string action
