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
#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
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
constexpr int ROAD_Y_MIN = 350;                   // 中线有效区间
constexpr int ROAD_Y_MAX = 480;
constexpr int OBJ_Y_MIN = 200;                    // 目标有效区间
constexpr int OBJ_Y_MAX = 480;
constexpr int SHM_SIZE = 4096;
constexpr int SERIAL_BAUDRATE = B115200;
constexpr const char* SHM_NAME = "/shm_road_coords";
constexpr const char* SERIAL_DEVICE = "/dev/ttyS3";
constexpr int HEADING_GAIN = 200;                // 【调参点】中线朝向补偿系数，越大入弯提前量越大
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

int object_area(const DetectObj& obj) {
    return std::abs(obj.x2 - obj.x1) * std::abs(obj.y2 - obj.y1);
}

int object_center_x(const DetectObj& obj) {
    return (obj.x1 + obj.x2) / 2;
}

bool is_obstacle(const DetectObj& obj) {
    return obj.c == 1 || obj.c == 2;
}

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
        if (tcgetattr(fd_, &options) != 0) {
            perror("tcgetattr error");
            close();
            return false;
        }

        if (cfsetispeed(&options, SERIAL_BAUDRATE) != 0
            || cfsetospeed(&options, SERIAL_BAUDRATE) != 0) {
            perror("cfset speed error");
            close();
            return false;
        }

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

        uint8_t buffer[8] = {
            FRAME_HEADER_1,
            FRAME_HEADER_2,
            speed,
            static_cast<uint8_t>((steering >> 8) & 0xFF),
            static_cast<uint8_t>(steering & 0xFF),
            static_cast<uint8_t>((target >> 8) & 0xFF),
            static_cast<uint8_t>(target & 0xFF),
            FRAME_TAIL
        };

        size_t sent = 0;
        while (sent < sizeof(buffer)) {
            ssize_t n = write(fd_, buffer + sent, sizeof(buffer) - sent);
            if (n < 0) {
                if (errno == EINTR) continue;
                perror("\n串口发送失败");
                return false;
            }
            if (n == 0) {
                cerr << "\n串口发送失败：write 返回 0" << endl;
                return false;
            }
            sent += static_cast<size_t>(n);
        }

        if (tcdrain(fd_) != 0) {
            perror("\n串口刷新失败");
            return false;
        }
        return true;
    }

    void send_stop() {
        send_frame(0, IMAGE_CENTER_X, IMAGE_CENTER_X);
    }

private:
    int fd_;
};

// ========================== 共享内存读取 ========================
class SharedMemoryMapping {
public:
    SharedMemoryMapping() : fd_(-1), ptr_(MAP_FAILED), size_(0) {}

    ~SharedMemoryMapping() {
        close();
    }

    bool open_readonly(const char* name, size_t size) {
        fd_ = shm_open(name, O_RDONLY, 0666);
        if (fd_ < 0) {
            return false;
        }

        ptr_ = mmap(0, size, PROT_READ, MAP_SHARED, fd_, 0);
        if (ptr_ == MAP_FAILED) {
            ::close(fd_);
            fd_ = -1;
            return false;
        }

        size_ = size;
        return true;
    }

    const uint8_t* data() const {
        return static_cast<const uint8_t*>(ptr_);
    }

    size_t size() const {
        return size_;
    }

private:
    void close() {
        if (ptr_ != MAP_FAILED) {
            munmap(ptr_, size_);
            ptr_ = MAP_FAILED;
        }
        if (fd_ != -1) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    int fd_;
    void* ptr_;
    size_t size_;
};

class BufferReader {
public:
    BufferReader(const uint8_t* data, size_t size)
        : data_(data), size_(size), offset_(0) {}

    template <typename T>
    bool read(T& value) {
        if (remaining() < sizeof(T)) {
            return false;
        }
        std::memcpy(&value, data_ + offset_, sizeof(T));
        offset_ += sizeof(T);
        return true;
    }

private:
    size_t remaining() const {
        return size_ - offset_;
    }

    const uint8_t* data_;
    size_t size_;
    size_t offset_;
};

bool read_shm_data(vector<RoadPoint>& road_points, vector<DetectObj>& objects) {
    SharedMemoryMapping mapping;
    if (!mapping.open_readonly(SHM_NAME, SHM_SIZE)) {
        return false;
    }

    BufferReader reader(mapping.data(), mapping.size());

    road_points.clear();
    objects.clear();

    uint32_t road_num = 0;
    if (!reader.read(road_num)) {
        return true;
    }

    for (uint32_t i = 0; i < road_num; ++i) {
        float x = 0.0f;
        float y = 0.0f;
        if (!reader.read(x) || !reader.read(y)) {
            return true;
        }

        int int_x = static_cast<int>(x);
        int int_y = static_cast<int>(y);
        if (int_y >= ROAD_Y_MIN && int_y <= ROAD_Y_MAX) {
            road_points.push_back({int_x, int_y});
        }
    }

    uint32_t obj_num = 0;
    if (!reader.read(obj_num)) {
        return true;
    }

    for (uint32_t i = 0; i < obj_num; ++i) {
        uint32_t c = 0;
        float x1 = 0.0f;
        float y1 = 0.0f;
        float x2 = 0.0f;
        float y2 = 0.0f;

        if (!reader.read(c) || !reader.read(x1) || !reader.read(y1)
            || !reader.read(x2) || !reader.read(y2)) {
            return true;
        }

        DetectObj obj{
            static_cast<int>(c),
            static_cast<int>(x1),
            static_cast<int>(y1),
            static_cast<int>(x2),
            static_cast<int>(y2)
        };

        int min_y = std::min(obj.y1, obj.y2);
        int max_y = std::max(obj.y1, obj.y2);
        if (max_y >= OBJ_Y_MIN && min_y <= OBJ_Y_MAX) {
            objects.push_back(obj);
        }
    }

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
        if (road_points.size() < 2) return;

        long long sum_y = 0, sum_x = 0, sum_xy = 0, sum_yy = 0;
        size_t n = road_points.size();
        for (size_t i = 0; i < n; ++i) {
            int y = road_points[i].y;
            int x = road_points[i].x;
            sum_y  += y;
            sum_x  += x;
            sum_xy += static_cast<long long>(y) * x;
            sum_yy += static_cast<long long>(y) * y;
        }

        double denom = static_cast<double>(n) * sum_yy
                     - static_cast<double>(sum_y) * sum_y;
        if (denom == 0.0) return;

        double a = (static_cast<double>(n) * sum_xy
                    - static_cast<double>(sum_y) * sum_x) / denom;
        double b = (static_cast<double>(sum_x) - a * sum_y) / n;

        int x_near = static_cast<int>(a * ROAD_Y_MAX + b);
        int heading_correction = static_cast<int>(a * HEADING_GAIN);

        base_road_x_ = x_near + heading_correction;
    }

    int select_best_object(const vector<DetectObj>& objects) {
        if (objects.empty()) return -1;

        bool has_priority_obstacle = false;
        for (size_t i = 0; i < objects.size(); ++i) {
            if (is_obstacle(objects[i]) && object_area(objects[i]) > OBSTACLE_AREA_THRESHOLD) {
                has_priority_obstacle = true;
                break;
            }
        }

        int valid_idx = -1;
        int max_y_val = -1;
        for (size_t i = 0; i < objects.size(); ++i) {
            int area = object_area(objects[i]);

            if (is_obstacle(objects[i])) {
                if (area <= OBSTACLE_AREA_THRESHOLD) continue;
            } else {
                if (area < COIN_AREA_THRESHOLD) continue;
            }

            if (has_priority_obstacle && !is_obstacle(objects[i])) {
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
        int obj_center_x = object_center_x(obj);
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
        int obj_center_x = object_center_x(obj);
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
static volatile sig_atomic_t g_stop_requested = 0;

void handle_sigint(int /*sig*/) {
    g_stop_requested = 1;
}

void stop_car_and_close_serial() {
    if (g_serial.is_open()) {
        g_serial.send_stop();
        g_serial.close();
        cout << "\n[程序终止] 已发送停车及状态清空指令并关闭串口。" << endl;
    }
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
    vector<RoadPoint> road_points;
    vector<DetectObj> objects;

    while (!g_stop_requested) {
        road_points.clear();
        objects.clear();

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

    stop_car_and_close_serial();
    return 0;
}
