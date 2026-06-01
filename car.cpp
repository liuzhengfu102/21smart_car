#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
// ================== 串口通信需要的头文件 ==================
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
// ================== 共享内存需要的头文件 ==================
#include <sys/mman.h>
#include <sys/stat.h>
#include <cstdint>
#include <vector>
#include <csignal>
// ==========================================================

using namespace std;
using namespace cv;

// 全局变量用于信号处理时关闭串口
int g_serial_fd = -1;

// 捕获 Ctrl+C 信号，清空串口发送数据并停车
void handle_sigint(int sig) {
    if (g_serial_fd != -1) {
        uint8_t buffer[8];
        buffer[0] = 0xAA; // 帧头一
        buffer[1] = 0x55; // 帧头二
        buffer[2] = 0;    // 目标速度置0 (停车)
        buffer[3] = (uint8_t)((320 >> 8) & 0xFF); // 中位线复位
        buffer[4] = (uint8_t)(320 & 0xFF);         
        buffer[5] = (uint8_t)((320 >> 8) & 0xFF); // 目标中线复位
        buffer[6] = (uint8_t)(320 & 0xFF);        
        buffer[7] = 0xFF; // 帧尾
        
        write(g_serial_fd, buffer, sizeof(buffer));
        tcdrain(g_serial_fd);
        close(g_serial_fd);
        cout << "\n[程序终止] 已发送停车及状态清空指令并关闭串口。" << endl;
    }
    exit(0);
}

// 初始化串口函数
int init_serial(const char* port) {
    // 增加 O_SYNC 标志尝试阻止立刻返回，取消 O_NDELAY
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd == -1) {
        perror("Unable to open serial port");
        return -1;
    }
    struct termios options;
    tcgetattr(fd, &options);
    
    // 设置波特率为 115200 
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    
    // 开启接收并允许本地连接
    options.c_cflag |= (CLOCAL | CREAD);
    
    // 8位数据位，无校验，1个停止位 (8N1)
    options.c_cflag &= ~PARENB; 
    options.c_cflag &= ~CSTOPB; 
    options.c_cflag &= ~CSIZE; 
    options.c_cflag |= CS8;     

    // 禁用硬件流控（如果不禁用，可能会卡死发送）
    options.c_cflag &= ~CRTSCTS;

    // 禁用软件流控
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // 纯原始输入模式
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
    
    // 纯原始输出模式
    options.c_oflag &= ~OPOST;  
    
    // 刷新串口缓冲
    tcflush(fd, TCIFLUSH);
    
    // 立刻应用设置
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("tcsetattr error");
        return -1;
    }
    return fd;
}

// ================== 共享内存结构体 ==================
struct RoadPoint {
    int x;
    int y;
};

struct DetectObj {
    int c;
    int x1, y1, x2, y2;
};

// ================== 共享内存读取函数 ==================
bool read_shm_data_full(vector<RoadPoint>& road_points, vector<DetectObj>& objects) {
    int shm_fd = shm_open("/shm_road_coords", O_RDONLY, 0666);
    if (shm_fd < 0) {
        return false;
    }

    void* ptr = mmap(0, 4096, PROT_READ, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED) {
        close(shm_fd);
        return false;
    }

    uint8_t* data = static_cast<uint8_t*>(ptr);
    int offset = 0;

    // 1. ===================== 读道路中线 =====================
    uint32_t road_num = *reinterpret_cast<uint32_t*>(data + offset);
    offset += 4;
    road_points.clear();
    // 添加越界保护机制
    for (uint32_t i = 0; i < road_num && offset + 8 <= 4096; ++i) {
        float x = *reinterpret_cast<float*>(data + offset);
        float y = *reinterpret_cast<float*>(data + offset + 4);
        
        int int_x = static_cast<int>(x);
        int int_y = static_cast<int>(y);
        
        // 过滤：有效的中线坐标其y轴范围为280到480，无效的予以舍去
        if (int_y >= 350&& int_y <= 480) {
            road_points.push_back({int_x, int_y});
        }
        offset += 8;
    }

    // 2. ===================== 读检测目标 =====================
    objects.clear();
    if (offset + 4 <= 4096) { 
        uint32_t obj_num = *reinterpret_cast<uint32_t*>(data + offset);
        offset += 4;

        for (uint32_t i = 0; i < obj_num && offset + 20 <= 4096; ++i) {
            uint32_t c = *reinterpret_cast<uint32_t*>(data + offset);
            float x1 = *reinterpret_cast<float*>(data + offset + 4);
            float y1 = *reinterpret_cast<float*>(data + offset + 8);
            float x2 = *reinterpret_cast<float*>(data + offset + 12);
            float y2 = *reinterpret_cast<float*>(data + offset + 16);
            
            int int_c = static_cast<int>(c);
            int int_x1 = static_cast<int>(x1);
            int int_y1 = static_cast<int>(y1);
            int int_x2 = static_cast<int>(x2);
            int int_y2 = static_cast<int>(y2);
            
            // 过滤：检测目标边界框的任意一点 y 坐标在 200 到 480 之间为有效
            int min_y = std::min(int_y1, int_y2);
            int max_y = std::max(int_y1, int_y2);
            if (max_y >= 480 && min_y <= 480) {
                objects.push_back({int_c, int_x1, int_y1, int_x2, int_y2});
            }
            offset += 20;
        }
    }

    munmap(ptr, 4096);
    close(shm_fd);
    return true;
}
// ==========================================================

int main(void){
    signal(SIGINT, handle_sigint); // 注册 Ctrl+C 退出信号处理
    int special_flag = -1; // 特殊标志位：-1（无元素）0（金币代表0）1（出现车）2（出现行人）
    int target_speed = 1; // 目标速度
    int image_median_x = 320; // 图像中位线位置
    int target_median_x = 320; // 目标图像中位线位置
    int64 last_car_tick = 0; // 记录最后一次看到车的时间(防止识别闪烁)
    int last_car_target_x = 320; // 记录最后一次车的避障方向
    float smoothed_coin_target_x = 320.0f; // 用于金币平滑追踪的低通滤波变量
    int base_road_x = 320; // 记录道路稳定的中线基准，防止偏置累加
    float final_smoothed_x = 320.0f; // 用于最终输出中线的低通滤波变量（防抖动）
    
    // ================== 文件读取模块 ==================
    // 稍后在循环中动态读取
    // ==================================================

    // 获取摄像头图形
    // 使用 GStreamer 管道明确指定底层格式和转换，适配 Orange Pi 的硬件特性

    // ================= 初始化串口模块 =================
    // 根据您的香橙派外设设置串口名，例如 "/dev/ttyS0", "/dev/ttyS3" 或 "/dev/ttyS9"  
    g_serial_fd = init_serial("/dev/ttyS3");
    if (g_serial_fd != -1) {
        cout << "串口打开成功！" << endl;
    } else {
        cout << "串口打开失败！请检查权限或串口设备名。" << endl;
    }
    // ==================================================

while(1){
    /* 1. 通过共享内存从 AI 视觉程序实时读取所有坐标和类别！*/
    vector<RoadPoint> road_points;
    vector<DetectObj> objects;
    
    bool shm_ok = read_shm_data_full(road_points, objects);

    if (shm_ok && (!road_points.empty() || !objects.empty())) {
        string print_obj_str = "";
        string print_action_str = "";

        // 修改为：计算所有中线点横坐标的平均值
        if (!road_points.empty()) {
            long long sum_x = 0;
            for (size_t i = 0; i < road_points.size(); ++i) {
                sum_x += road_points[i].x;
            }
            base_road_x = static_cast<int>(sum_x / road_points.size());
        }
        
        // 每次重新基于道路中线底准开始计算，防止上一帧的避障偏移量被无限累加导致溢出 (-4634)
        image_median_x = base_road_x;
        
        target_median_x = 320; // 默认复位目标中位线为直行状态(320)
        bool current_has_car = false; // 标记本帧是否有效识别到车
        bool current_has_coin = false; // 标记本帧是否有效识别到金币
        
        if (!objects.empty()) {
            // 首先判断是否同时存在其他优先元素（如车1，人2），加入对其面积的判断
            bool has_priority_obstacle = false;
            for (size_t i = 0; i < objects.size(); ++i) {
                if (objects[i].c == 1 || objects[i].c == 2) {
                    int w = std::abs(objects[i].x2 - objects[i].x1);
                    int h = std::abs(objects[i].y2 - objects[i].y1);
                    int area = w * h;
                    // 对障碍车进行面积限制，当面积大于一定值时才为有效元素
                    if (area > 3000) { // 【调参点】设定障碍车面积阈值，过滤掉过小或太远的障碍车
                        has_priority_obstacle = true;
                        break;
                    }
                }
            }

            // 若同时识别到多个有效元素，则寻找 y 坐标最大（最靠近车身）的那个元素作为唯一有效元素
            int valid_idx = -1; // 修改为 -1 避免没找到有效目标时引发数组越界或错误拾取 
            int max_y_val = -1;
            for (size_t i = 0; i < objects.size(); ++i) {
                int w = std::abs(objects[i].x2 - objects[i].x1);
                int h = std::abs(objects[i].y2 - objects[i].y1);
                int area = w * h;

                // --- 新增：对金币和障碍车分别加入面积计算过滤 --- 
                if (objects[i].c != 1 && objects[i].c != 2) { // 对金币等非障碍物
                    if (area < 1000) { // 【调参点】设定金币面积阈值
                        continue;
                    }
                } else { // 对障碍车、人等优先障碍物
                    if (area <= 3000) { // 【调参点】相同的障碍车面积限制
                        continue;
                    }
                }

                // 如果画面中存在有效的车或行人，则金币等将其作无效处理，优先处理其他元素
                if (has_priority_obstacle && objects[i].c != 1 && objects[i].c != 2) {
                    continue;
                }
                
                int obj_max_y = std::max(objects[i].y1, objects[i].y2);
                if (obj_max_y > max_y_val) {
                    max_y_val = obj_max_y;
                    valid_idx = i;
                }
            }

            if (valid_idx != -1) {
                special_flag = objects[valid_idx].c;
                
                ostringstream obj_oss;
                obj_oss << "类别=" << objects[valid_idx].c 
                        << " 坐标[" << objects[valid_idx].x1 << "," << objects[valid_idx].y1 
                        << " " << objects[valid_idx].x2 << "," << objects[valid_idx].y2 << "]";
                print_obj_str = obj_oss.str();

                // 如果检测到的是车（特别标志位 1 代表车）
                if (special_flag == 1 || special_flag == 2) {
                    current_has_car = true;
                    // 计算汽车中心 x 坐标
                    int obj_center_x = (objects[valid_idx].x1 + objects[valid_idx].x2) / 2;
                    // 判断汽车在赛道中线的左侧还是右侧
                    if (obj_center_x < 320) {
                        image_median_x = image_median_x - 160; // 车在左侧，减小 image_median_x 使其向右转（躲避）
                        print_action_str = "识别到障碍(人或车)在左，正在向右规避";
                    } else {
                        image_median_x = image_median_x + 160; // 车在右侧，增加 image_median_x 使其向左转（躲避）
                        print_action_str = "识别到障碍(人或车)在右，正在向左规避";
                    }
                    // 每当看到车，刷新时间和方向记录
                    last_car_tick = getTickCount();
                    last_car_target_x = image_median_x; // 现在我们用它记录 image_median_x 的状态
                } else if (special_flag == 0) {
                    current_has_coin = true;
                    // 如果检测到的是金币（特别标志位 0 代表金币）
                    int obj_center_x = (objects[valid_idx].x1 + objects[valid_idx].x2) / 2;
                    
                    if (obj_center_x < 320) {
                        print_action_str = "识别到金币在左，向左追踪获取";
                    } else {
                        print_action_str = "识别到金币在右，向右追踪获取";
                    }

                    // 逻辑：对获取的 image_median_x 做处理，减小等于向右转，增加等于向左转
                    // 金币在左侧 (obj_center_x < 320) 时要去吃，要向左转，所以加一个正数(diff_x > 0)
                    // 金币在右侧 (obj_center_x > 320) 时要去吃，要向右转，所以减一个数(diff_x < 0)
                    int diff_x = 320 - obj_center_x;  
                    int raw_image_x = image_median_x + diff_x;  // 基于前面AI视觉读取计算出的实际道路 image_median_x 进行加减修改
                    
                    // 引入一阶低通滤波 (Exponential Moving Average) 平滑金币轨迹
                    float alpha = 0.15f; 
                    smoothed_coin_target_x = alpha * raw_image_x + (1.0f - alpha) * smoothed_coin_target_x;
                    image_median_x = static_cast<int>(smoothed_coin_target_x);
                }
            }
        } 
        
        // 若当前未在追踪金币，将平滑变量与当前实际输出同步，防止下次出现金币时发生跳变折跃
        if (!current_has_coin) {
            smoothed_coin_target_x = image_median_x;
        }
        
        // --- 容错防丢机制：如果短时间内曾检测到过车，维持之前的避障动作 ---
        if (!current_has_car) {
            double elapsed_time = static_cast<double>(getTickCount() - last_car_tick) / getTickFrequency();
            if (elapsed_time < 0.5) { // 强制保持状态防止车头乱晃
                image_median_x = last_car_target_x;
                special_flag = 1; 
            } else {
                if (!current_has_coin) {
                    special_flag = -1; // 超出容错时间，且本帧既无车又无金币，回归无元素状态(-1)
                }
            }
        }
        
        // --- 最终输出平滑（防抖动算法） ---
        // 使用一阶低通滤波 (Exponential Moving Average) 使 image_median_x 变化更加平滑柔和
        float alpha_final = 0.3f; // 【调参点】最终平滑系数，0.1=极度平滑但响应慢，0.8=抖动大但响应快
        final_smoothed_x = alpha_final * image_median_x + (1.0f - alpha_final) * final_smoothed_x;
        image_median_x = static_cast<int>(final_smoothed_x);

        // 构建综合信息面板，使用 ANSI 清理当前行并在终端上方固定位置更新 (这里用 \r 覆写)
        stringstream panel;
        panel << "\r\033[K" // 回到行首并清空整行
              << " 最终中线值: " << image_median_x 
              << " | ";
        if (print_obj_str.empty()) {
            panel << "未检测到有效目标       ";
        } else {
            panel << "当前有效目标: " << print_obj_str << " -> " << print_action_str;
        }
        
        cout << panel.str() << flush;
    }
   
    // ================= 发送串口数据模块 ===============
    if (g_serial_fd != -1) {
        // 定义具有针头和针尾的数据包：帧头 0xAA 0x55，包含速度与中位线，帧尾 0xFF，总长8字节
        uint8_t buffer[8];
        
        buffer[0] = 0xAA; // 帧头一
        buffer[1] = 0x55; // 帧头二
        
        // 数据部分装填
        buffer[2] = (uint8_t)target_speed;   // 第1个变量 (速度)
        // 应对像宽度这种超出0~255的值的情况，拆分为高8位和低8位
        buffer[3] = (uint8_t)((image_median_x >> 8) & 0xFF);  // 第2个变量高位 (如果没有超过255这一项是0)
        buffer[4] = (uint8_t)(image_median_x & 0xFF);         // 第2个变量低位
        
        buffer[5] = (uint8_t)((target_median_x >> 8) & 0xFF); // 第3个变量高位 target_median_x
        buffer[6] = (uint8_t)(target_median_x & 0xFF);        // 第3个变量低位 target_median_x
        
        buffer[7] = 0xFF; // 帧尾
        
        // 发送数据
        int n = write(g_serial_fd, buffer, sizeof(buffer));
        if (n < 0) {
            perror("\n串口发送失败");
        } else {
            // 确保数据强制刷新推送到物理引脚上，而不是堆积在系统内核缓冲区
            tcdrain(g_serial_fd);
        }
    }
    // 为了防止死循环过载并且让串口有个响应时间，加入延时（50毫秒）
    //usleep(50000);

}

    if(g_serial_fd != -1) {
        close(g_serial_fd);
    }
    return 0;
}

