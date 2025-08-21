px4_add_module(
    MODULE examples__sky_app  # 定义模块名称
    MAIN sky_app              # 指定模块的主入口标识
    COMPILE_FLAGS               # 预留段：用于添加编译选项（如 -Wall、-O2 等，此处未填具体值）
    SRCS                        # 开始声明模块的源文件列表
        sky_app_main.cpp     # 程序所需源文件
    DEPENDS                     # 程序依赖项：本示例没有依赖项，所以此处为空
)
