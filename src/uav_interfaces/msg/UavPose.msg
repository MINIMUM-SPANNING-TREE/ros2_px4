# 无人机位姿（ENU坐标系，单位：米、弧度）
std_msgs/Header header

float64 x           # 东向位置（ENU: East）
float64 y           # 北向位置（ENU: North）  
float64 z           # 高度（ENU: Up，向上为正）

float64 yaw         # 偏航角（绕Z轴，从X轴正方向逆时针为正，单位：弧度）
                    # 范围：-π ~ +π
                    # 0 = 指向正东（X轴正方向）
                    # π/2 = 指向正北（Y轴正方向）