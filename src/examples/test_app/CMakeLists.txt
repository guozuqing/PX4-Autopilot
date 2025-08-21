px4_add_module(
    MODULE examples__test_app     # 模块名称，实际路径为 examples/test_app
                                  # 在编译生成后，这个模块可以通过 PX4 的 nsh 命令行执行：test_app start

    MAIN test_app                 # 模块的主函数入口名称，对应 C++ 文件中的函数：
                                  # int test_app_main(int argc, char *argv[]);

    COMPILE_FLAGS                 # 这里可以添加模块独有的编译参数（比如 -O2、-Wall），当前为空，表示不添加额外编译选项

    SRCS
        test_app_main.cpp        # 模块使用的源文件列表，支持添加多个 .cpp 文件，这里只有一个主文件

    DEPENDS                      # 当前模块依赖的其他模块或库，例如：px4_platform、libairspeed 等
                                  # 为空表示无外部依赖；如果使用 PX4 提供的通用库或中间件，请在此处列出
)
