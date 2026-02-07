# 系统健康状态综合诊断
std_msgs/Header header

int8 level          # 健康等级：
int8 OK = 0         # 正常
int8 WARNING = 1    # 警告（需注意）
int8 ERROR = 2      # 错误（建议降落）

string message      # 详细状态描述，如：
                    # "NOMINAL", "DISCONNECTED", "LOW_BATTERY", 
                    # "POSITION_LOST", "HIGH_VIBRATION"

bool connected      # 通信连接状态（冗余，方便快速判断）
