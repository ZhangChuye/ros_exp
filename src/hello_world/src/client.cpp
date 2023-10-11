#include "ros/ros.h"
#include "hello_world/sumup.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    // 调用时动态传值,如果通过 launch 的 args 传参，需要传递的参数个数 +3
    if (argc != 3)
    // if (argc != 5)//launch 传参(0-文件路径 1传入的参数 2传入的参数 3节点名称 4日志路径)
    {
        ROS_ERROR("请提交两个整数");
        return 1;
    }


    // 2.初始化 ROS 节点
    ros::init(argc,argv,"AddInts_Client");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    // 4.创建 客户端 对象
    // This name should be the same as the service
    ros::ServiceClient client = nh.serviceClient<hello_world::sumup>("SumUp");
    //等待服务启动成功
    //方式1
    ros::service::waitForService("SumUp");
    //方式2
    // client.waitForExistence();
    // 5.组织请求数据
    hello_world::sumup ai;
    ai.request.num1 = atoi(argv[1]);
    // ai.request.num1 = 100;
    ai.request.num2 = atoi(argv[2]);
    // ai.request.num2 = 200;
    // 6.发送请求,返回 bool 值，标记是否成功
    bool flag = client.call(ai);
    // 7.处理响应
    if (flag)
    {
        ROS_INFO("请求正常处理,响应结果:%d",ai.response.sum);
    }
    else
    {
        ROS_INFO("%d",ai.request.num1);
        ROS_INFO("%d",ai.request.num2);
        ROS_ERROR("请求处理失败....");
        return 1;
    }

    return 0;
}