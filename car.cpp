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

// ========================== 常量定义区 ==========================
constexpr int IMAGE_CENTER_X = 320;
constexpr int OBSTACLE_AREA_THRESHOLD = 3000;     // 【调参点】障碍车面积阈值
constexpr int COIN_AREA_THRESHOLD = 1000;         // 【调参点】金币面积阈值
constexpr int OBSTACLE_STEERING_OFFSET = 160;     // 障碍物避障偏移量
constexpr float COIN_SMOOTH_ALPHA = 0.15f;        // 【调参点】金币追踪平滑系数
constexpr float OUTPUT_SMOOTH_ALPHA = 0.3f;       // 【调参点】最终输出平滑系数
constexpr float CAR_PERSIST_TIME = 0.5f;          // 障碍物消失后保持避障时间(秒)
constexpr int ROAD_Y_MIN = 350;
constexpr int ROAD_Y_MAX = 480;
constexpr int OBJ_Y_MIN = 200;
constexpr int OBJ_Y_MAX = 480;
constexpr int SHM_SIZE = 4096;
constexpr int SERIAL_BAUDRATE = B115200;
constexpr const char* SHM_NAME = "/shm_road_coords";
constexpr const char* SERIAL_DEVICE = "/dev/ttyS3";
constexpr int DEFAULT_SPEED = 1;
constexpr uint8_t FRAME_HEADER_1 = 0xAA;
constexpr uint8_t FRAME_HEADER_2 = 0x55;
constexpr uint8_t FRAME_TAIL = 0xFF;
// ===============================================================

using namespace std;

// ========================== 数据结构区 ==========================
struct RoadPoint {
    int x;
    int y;
};

struct DetectObj {
    int c;
    int x1, y1, x2, y2;
};
// ===============================================================

// ========================== SerialPort 类 =======================
class SerialPort {
public:
    SerialPort() : fd_(-1) {}

    ~SerialPort() {
        close();
    }

    bool init(const char* device) {
        fd_ = open(device, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ == -1) {
            perror("Unable to open serial port");
            return false;
        }
        struct termios options;
        tcgetattr(fd_, &options);

        cfsetispeed(&options, SERIAL_BAUDRATE);
        cfsetospeed(&options, SERIAL_BAUDRATE);

        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~CRTSCTS;
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;

        tcflush(fd_, TCIFLUSH);
        if (tcsetattr(fd_, TCSANOW, &options) != 0) {
            perror("tcsetattr error");
            close();
            return false;
        }
        return true;
    }

    void close() {
        if (fd_ != -1) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    bool is_open() const {
        return fd_ != -1;
    }

    bool send_frame(uint8_t speed, int steering, int target) {
        if (fd_ == -1) return false;
        uint8_t buffer[8];
        buffer[0] = FRAME_HEADER_1;
        buffer[1] = FRAME_HEADER_2;
        buffer[2] = speed;
        buffer[3] = static_cast<uint8_t>((steering >> 8) & 0xFF);
        buffer[4] = static_cast<uint8_t>(steering & 0xFF);
        buffer[5] = static_cast<uint8_t>((target >> 8) & 0xFF);
        buffer[6] = static_cast<uint8_t>(target & 0xFF);
        buffer[7] = FRAME_TAIL;

        int n = write(fd_, buffer, sizeof(buffer));
        if (n < 0) {
            perror("\n串口发送失败");
            return false;
        }
        tcdrain(fd_);
        return true;
    }

    void send_stop() {
        send_frame(0, IMAGE_CENTER_X, IMAGE_CENTER_X);
    }

private:
    int fd_;
};

// ========================== 共享内存读取 ========================
bool read_shm_data(vector<RoadPoint>& road_points, vector<DetectObj>& objects) {
    int shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);
    if (shm_fd < 0) {
        return false;
    }

    void* ptr = mmap(0, SHM_SIZE, PROT_READ, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED) {
        close(shm_fd);
        return false;
    }

    uint8_t* data = static_cast<uint8_t*>(ptr);
    int offset = 0;

    uint32_t road_num = *reinterpret_cast<uint32_t*>(data + offset);
    offset += 4;
    road_points.clear();
    for (uint32_t i = 0; i < road_num && offset + 8 <= SHM_SIZE; ++i) {
        float x = *reinterpret_cast<float*>(data + offset);
        float y = *reinterpret_cast<float*>(data + offset + 4);
        int int_x = static_cast<int>(x);
        int int_y = static_cast<int>(y);
        if (int_y >= ROAD_Y_MIN && int_y <= ROAD_Y_MAX) {
            road_points.push_back({int_x, int_y});
        }
        offset += 8;
    }

    objects.clear();
    if (offset + 4 <= SHM_SIZE) {
        uint32_t obj_num = *reinterpret_cast<uint32_t*>(data + offset);
        offset += 4;
        for (uint32_t i = 0; i < obj_num && offset + 20 <= SHM_SIZE; ++i) {
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

            int min_y = std::min(int_y1, int_y2);
            int max_y = std::max(int_y1, int_y2);
            if (max_y >= OBJ_Y_MAX && min_y <= OBJ_Y_MAX) {
                objects.push_back({int_c, int_x1, int_y1, int_x2, int_y2});
            }
            offset += 20;
        }
    }

    munmap(ptr, SHM_SIZE);
    close(shm_fd);
    return true;
}
// ===============================================================

// ========================== CarController 类 ====================
class CarController {
public:
    CarController()
        : special_flag_(-1), target_speed_(DEFAULT_SPEED),
          image_median_x_(IMAGE_CENTER_X), target_median_x_(IMAGE_CENTER_X),
          last_car_tick_(0), last_car_target_x_(IMAGE_CENTER_X),
          smoothed_coin_target_x_(static_cast<float>(IMAGE_CENTER_X)),
          base_road_x_(IMAGE_CENTER_X),
          final_smoothed_x_(static_cast<float>(IMAGE_CENTER_X)) {}

    void process(const vector<RoadPoint>& road_points, const vector<DetectObj>& objects) {
        print_obj_str_.clear();
        print_action_str_.clear();

        compute_road_center(road_points);

        image_median_x_ = base_road_x_;
        target_median_x_ = IMAGE_CENTER_X;
        bool current_has_car = false;
        bool current_has_coin = false;

        int valid_idx = select_best_object(objects);
        if (valid_idx != -1) {
            const DetectObj& obj = objects[valid_idx];
            special_flag_ = obj.c;

            ostringstream oss;
            oss << "类别=" << obj.c << " 坐标[" << obj.x1 << "," << obj.y1
                << " " << obj.x2 << "," << obj.y2 << "]";
            print_obj_str_ = oss.str();

            if (special_flag_ == 1 || special_flag_ == 2) {
                current_has_car = true;
                compute_obstacle_steering(obj);
            } else if (special_flag_ == 0) {
                current_has_coin = true;
                compute_coin_steering(obj);
            }
        }

        if (!current_has_coin) {
            smoothed_coin_target_x_ = static_cast<float>(image_median_x_);
        }
        apply_persistence(current_has_car, current_has_coin);
        smooth_output();
    }

    int get_steering() const { return image_median_x_; }
    int get_target() const { return target_median_x_; }
    int get_speed() const { return target_speed_; }
    int get_flag() const { return special_flag_; }
    const string& get_obj_str() const { return print_obj_str_; }
    const string& get_action_str() const { return print_action_str_; }

private:
    int special_flag_;
    int target_speed_;
    int image_median_x_;
    int target_median_x_;
    int64 last_car_tick_;
    int last_car_target_x_;
    float smoothed_coin_target_x_;
    int base_road_x_;
    float final_smoothed_x_;
    string print_obj_str_;
    string print_action_str_;

    void compute_road_center(const vector<RoadPoint>& road_points) {
        if (road_points.empty()) return;
        long long sum_x = 0;
        for (size_t i = 0; i < road_points.size(); ++i) {
            sum_x += road_points[i].x;
        }
        base_road_x_ = static_cast<int>(sum_x / road_points.size());
    }

    int select_best_object(const vector<DetectObj>& objects) {
        if (objects.empty()) return -1;

        bool has_priority_obstacle = false;
        for (size_t i = 0; i < objects.size(); ++i) {
            if (objects[i].c == 1 || objects[i].c == 2) {
                int area = std::abs(objects[i].x2 - objects[i].x1)
                         * std::abs(objects[i].y2 - objects[i].y1);
                if (area > OBSTACLE_AREA_THRESHOLD) {
                    has_priority_obstacle = true;
                    break;
                }
            }
        }

        int valid_idx = -1;
        int max_y_val = -1;
        for (size_t i = 0; i < objects.size(); ++i) {
            int area = std::abs(objects[i].x2 - objects[i].x1)
                     * std::abs(objects[i].y2 - objects[i].y1);

            if (objects[i].c != 1 && objects[i].c != 2) {
                if (area < COIN_AREA_THRESHOLD) continue;
            } else {
                if (area <= OBSTACLE_AREA_THRESHOLD) continue;
            }

            if (has_priority_obstacle && objects[i].c != 1 && objects[i].c != 2) {
                continue;
            }

            int obj_max_y = std::max(objects[i].y1, objects[i].y2);
            if (obj_max_y > max_y_val) {
                max_y_val = obj_max_y;
                valid_idx = static_cast<int>(i);
            }
        }
        return valid_idx;
    }

    void compute_obstacle_steering(const DetectObj& obj) {
        int obj_center_x = (obj.x1 + obj.x2) / 2;
        if (obj_center_x < IMAGE_CENTER_X) {
            image_median_x_ -= OBSTACLE_STEERING_OFFSET;
            print_action_str_ = "识别到障碍(人或车)在左，正在向右规避";
        } else {
            image_median_x_ += OBSTACLE_STEERING_OFFSET;
            print_action_str_ = "识别到障碍(人或车)在右，正在向左规避";
        }
        last_car_tick_ = cv::getTickCount();
        last_car_target_x_ = image_median_x_;
    }

    void compute_coin_steering(const DetectObj& obj) {
        int obj_center_x = (obj.x1 + obj.x2) / 2;
        if (obj_center_x < IMAGE_CENTER_X) {
            print_action_str_ = "识别到金币在左，向左追踪获取";
        } else {
            print_action_str_ = "识别到金币在右，向右追踪获取";
        }

        int diff_x = IMAGE_CENTER_X - obj_center_x;
        int raw_image_x = image_median_x_ + diff_x;
        smoothed_coin_target_x_ = COIN_SMOOTH_ALPHA * raw_image_x
                                + (1.0f - COIN_SMOOTH_ALPHA) * smoothed_coin_target_x_;
        image_median_x_ = static_cast<int>(smoothed_coin_target_x_);
    }

    void apply_persistence(bool current_has_car, bool current_has_coin) {
        if (!current_has_car) {
            double elapsed = static_cast<double>(cv::getTickCount() - last_car_tick_)
                           / cv::getTickFrequency();
            if (elapsed < CAR_PERSIST_TIME) {
                image_median_x_ = last_car_target_x_;
                special_flag_ = 1;
            } else {
                if (!current_has_coin) {
                    special_flag_ = -1;
                }
            }
        }
    }

    void smooth_output() {
        final_smoothed_x_ = OUTPUT_SMOOTH_ALPHA * image_median_x_
                          + (1.0f - OUTPUT_SMOOTH_ALPHA) * final_smoothed_x_;
        image_median_x_ = static_cast<int>(final_smoothed_x_);
    }
};
// ===============================================================

// ========================== 全局变量 / 信号处理 =================
static SerialPort g_serial;

void handle_sigint(int /*sig*/) {
    if (g_serial.is_open()) {
        g_serial.send_stop();
        g_serial.close();
        cout << "\n[程序终止] 已发送停车及状态清空指令并关闭串口。" << endl;
    }
    exit(0);
}
// ===============================================================

int main() {
    signal(SIGINT, handle_sigint);

    if (g_serial.init(SERIAL_DEVICE)) {
        cout << "串口打开成功！" << endl;
    } else {
        cout << "串口打开失败！请检查权限或串口设备名。" << endl;
    }

    CarController controller;

    while (true) {
        vector<RoadPoint> road_points;
        vector<DetectObj> objects;

        bool shm_ok = read_shm_data(road_points, objects);

        if (shm_ok && (!road_points.empty() || !objects.empty())) {
            controller.process(road_points, objects);

            stringstream panel;
            panel << "\r\033[K"
                  << " 最终中线值: " << controller.get_steering()
                  << " | ";
            if (controller.get_obj_str().empty()) {
                panel << "未检测到有效目标       ";
            } else {
                panel << "当前有效目标: " << controller.get_obj_str()
                      << " -> " << controller.get_action_str();
            }
            cout << panel.str() << flush;
        }

        if (g_serial.is_open()) {
            g_serial.send_frame(
                static_cast<uint8_t>(controller.get_speed()),
                controller.get_steering(),
                controller.get_target()
            );
        }
    }

    return 0;
}
