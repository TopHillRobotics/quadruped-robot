uint8 levelFlag
uint16 commVersion                  # Old version Aliengo does not have
uint16 robotID                      # Old version Aliengo does not have
uint32 SN                           # Old version Aliengo does not have
uint8 bandWidth                     # Old version Aliengo does not have
uint8 mode
float32 progress                    # new on Go1, reserve
IMU imu
uint8 gaitType                      # new on Go1, 0.idle  1.trot  2.trot running  3.climb stair
float32 footRaiseHeight             # (unit: m, default: 0.08m), foot up height while walking
float32[3] position                 # (unit: m), from own odometry in inertial frame, usually drift
float32 bodyHeight                  # (unit: m, default: 0.28m)
float32[3] velocity                 # (unit: m/s), forwardSpeed, sideSpeed, rotateSpeed in body frame
float32 yawSpeed                    # (unit: rad/s), rotateSpeed in body frame        
Cartesian[4] footPosition2Body      # foot position relative to body
Cartesian[4] footSpeed2Body         # foot speed relative to body
int8[20] temperature
BmsState bms
int16[4] footForce                  # Old version Aliengo is different
int16[4] footForceEst               # Old version Aliengo does not have
uint8[40] wirelessRemote
uint32 reserve                      # Old version Aliengo does not have
uint32 crc