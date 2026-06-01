import struct
from multiprocessing import shared_memory
import time
import math

SHM_NAME = "shm_road_coords"
SHM_SIZE = 4096

def configure_shm():
    try:
        # 尝试创建共享内存
        shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)
        print(f"创建了新的共享内存: {SHM_NAME}")
    except FileExistsError:
        # 如果已经存在，则直接连接
        shm = shared_memory.SharedMemory(name=SHM_NAME, create=False, size=SHM_SIZE)
        print(f"连接到已存在的共享内存: {SHM_NAME}")
    return shm

def main():
    shm = configure_shm()

    try:
        print("开始向共享内存写入实时跳动的数据  (按 Ctrl+C 终止)...")
        counter = 0.0
        
        while True:
            # 1. 生成连续递增再折返的动态变化值 (从 0 线性增加到 80，再回到 0)
            # 这样能更好得观察 C++ 接收是否存在延迟卡顿
            cycle_val = counter % 160.0
            image_median_x = cycle_val if cycle_val <= 80.0 else 160.0 - cycle_val
            
            # 2. 模拟生成动态变化的标志位 (0: 无, 1: 车, 2: 行人)
            special_flag = int(counter) % 3

            # --- 开始打包数据写入内存 ---
            ptr = 0
            
            # 道路中线部分: 写入路点数量 (例如发送 5 个点来测试多坐标情况)
            num_points = 5
            struct.pack_into("I", shm.buf, ptr, num_points)
            ptr += 4
            
            # 使用循环写入多个当前点位 x, y
            for i in range(num_points):
                # 我们让接下来的几个点横坐标产生些许偏差，模拟真实检测到的多点车道线
                pt_x = image_median_x + i * 5.0 
                pt_y = 300.0 + i * 15.0
                struct.pack_into("ff", shm.buf, ptr, pt_x, pt_y)
                ptr += 8
            
            # 目标检测部分: 写入目标数量 (obj_num = 1)
            struct.pack_into("I", shm.buf, ptr, 1)
            ptr += 4
            # 写入目标: 类别 c, 以及框坐标 x1, y1, x2, y2
            struct.pack_into("Iffff", shm.buf, ptr, special_flag, 0.0, 0.0, 0.0, 0.0)
            
            print(f"写入成功 -> special_flag: {special_flag}, image_median_x: {image_median_x:.1f}")
            
            # 使用较小的步进(例如每次增量1) 和较快的刷新率来观察连续性
            counter += 1.0
            time.sleep(0.05) # 1秒钟刷新20次，非常适合观察延迟

    except KeyboardInterrupt:
        print("\n停止写入。")
    finally:
        shm.close()
        shm.unlink() # 释放共享内存

if __name__ == "__main__":
    main()
