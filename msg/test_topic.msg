uint64 timestamp
# 每个uORB主题必须包含uint64类型的timestamp成员，用于日志模块记录数据，发布消息时需赋值。注意：.msg非源码文件，变量间无需分号；类型为uint64（区别于 C/C++ 的 uint64_t），变量名不可大写开头
uint64 data
# 主题实际传递的数据，支持多类型/数组（如 "float32[8] data" 表示容量为 8 的 32 位浮点数组）
uint8 TEST_CONST_DATA = 0
# 赋值表示该数据为常量（程序运行中不可修改），常用于主题内变量的枚举值（本例中常量值为0）

# 以 "# TOPICS" 开头，声明同数据类型的多个独立主题（空格分隔主题名）。规则：①省略则默认生成与文件名（test_topic.msg）同名的主题；② 若声明，第一个主题必须与.msg 文件名（test_topic）一致，否则编译报错
# TOPICS test_topic test_topic_x
