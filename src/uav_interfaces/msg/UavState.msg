# 飞控连接和飞行状态
std_msgs/Header header

bool connected      # 是否与飞控建立MAVLink连接
bool armed          # 是否已解锁（电机可转动）
bool guided         # 是否接受外部控制指令

string mode         # 当前飞行模式，如：
                    # "MANUAL", "STABILIZED", "POSCTL", 
                    # "OFFBOARD", "AUTO.TAKEOFF", "AUTO.LAND", 
                    # "AUTO.LOITER", "AUTO.MISSION", "AUTO.RTL"

string landed_state # 着陆状态：
                    # "UNKNOWN", "ON_GROUND", "IN_AIR", 
                    # "TAKING_OFF", "LANDING"