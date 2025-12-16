string DELIMITER = ";"

# Action
string UP = "UP"
string DOWN = "DOWN"
string TAKE = "TAKE"
string FREE = "FREE"
string THERMO_DEPLOY = "THERMO_DEPLOY"
string THERMO_UNDEPLOY = "THERMO_UNDEPLOY"
string MAX_DEPLOY = "MAX_DEPLOY"

# Step
int32 DONE_STEP = 0
int32 UP_STEP = 1
int32 DOWN_STEP = 2
int32 TAKE_STEP = 3
int32 FREE_STEP = 4
int32 THERMO_DEPLOY_STEP = 5
int32 THERMO_UNDEPLOY_STEP = 6
int32 MAX_DEPLOY_STEP = 99

int32[] steps
string action
