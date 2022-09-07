uint8 levelFlag
uint16 commVersion              # Old version Aliengo does not have
uint16 robotID                  # Old version Aliengo does not have
uint32 SN                       # Old version Aliengo does not have
uint8 bandWidth                 # Old version Aliengo does not have
uint8 mode                      # 0. idle, default stand  
                                # 1. force stand (controlled by dBodyHeight + ypr)
                                # 2. target velocity walking (controlled by velocity + yawSpeed)
                                # 3. target position walking (controlled by position + ypr[0])
                                # 4. path mode walking (reserve for future release)
                                # 5. position stand down. 
                                # 6. position stand up 
                                # 7. damping mode 
                                # 8. recovery stand
                                # 9. backflip
                                # 10. jumpYaw
                                # 11. straightHand
                                # 12. dance1
                                # 13. dance2
                                # 14. two leg stand
uint8 gaitType                  # 0.idle  1.trot  2.trot running  3.climb stair
uint8 speedLevel                # 0. default low speed. 1. medium speed 2. high speed. during walking, only respond MODE 3
float32 footRaiseHeight         # (unit: m, default: 0.08m), foot up height while walking
float32 bodyHeight              # (unit: m, default: 0.28m),
float32[2] postion              # (unit: m), desired position in inertial frame
float32[3] euler                # (unit: rad), roll pitch yaw in stand mode
float32[2] velocity             # (unit: m/s), forwardSpeed, sideSpeed in body frame
float32 yawSpeed                # (unit: rad/s), rotateSpeed in body frame
BmsCmd bms
LED[4] led
uint8[40] wirelessRemote
uint32 reserve                  # Old version Aliengo does not have
int32 crc