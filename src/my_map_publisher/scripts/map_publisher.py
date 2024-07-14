#!/usr/bin/env python3
import rospy  
from nav_msgs.msg import OccupancyGrid  
import numpy as np  

# 生成障碍物
def obstacle_generate(map_resolution):
    m_type = [1,2,3]
    m_pos_map = np.array([[2,2,0],[4,4,0],[4,8,0]])
    m_size_map = np.array([[1,1,0],[1,1,0],[1,1,0]])
    m_pos_grid = m_pos_map
    m_size_grid = m_size_map
    # 将障碍物坐标和尺寸转换为栅格地图中的坐标和尺寸
    for i in range(len(m_type)):
        m_pos_grid[i][0] = m_pos_map[i][0] / map_resolution
        m_pos_grid[i][1] = m_pos_map[i][1] / map_resolution
        m_size_grid[i][0] = m_size_map[i][0] / map_resolution
        m_size_grid[i][1] = m_size_map[i][1] / map_resolution
    obstacles = []
    for i in range(len(m_type)):
        obstacles.append({
            'position':m_pos_grid[i],
            'size':m_size_grid[i],
            'type':m_type
        })
    return obstacles

# 创建障碍物地图
def map_generate():
    # 创建一个空的占据栅格地图  
    map_msg = OccupancyGrid()  
    map_msg.header.frame_id = "map"  
    map_msg.header.stamp = rospy.Time.now()  
    map_msg.info.resolution = 0.05 
    map_msg.info.width = 200  
    map_msg.info.height = 200  
    map_msg.info.origin.position.x = -map_msg.info.width * map_msg.info.resolution // 2
    map_msg.info.origin.position.y = -map_msg.info.height * map_msg.info.resolution // 2
    map_msg.info.origin.orientation.w = 1.0  

    # 初始化数据  
    map_msg.data = [0] * (map_msg.info.width * map_msg.info.height)  # 假设全部0 
    # 获取障碍物信息
    obstacles = obstacle_generate(map_msg.info.resolution)
    # print(obstacles)
    # 2D栅格地图
    map_2D = np.zeros((map_msg.info.height,map_msg.info.width))
    for obstacle in obstacles:
        x,y,z = obstacle['position']
        size_x,size_y,size_z = obstacle['size']
        if int(x-size_x/2) >0 and int(y-size_y/2)>0 and int(x+size_x/2) <map_msg.info.width and int(y+size_y/2)<map_msg.info.height: 
            map_2D[int(y-size_y/2):int(y+size_y/2),int(x-size_x/2):int(x+size_x/2)] = 100
        else:
            print("警告:输入障碍物信息有误")
    # 2D栅格地图降维
    # map_2D_inverse = map_2D
    # # print("map_2D",map_2D)
    # print(len(map_2D[:,0]))
    # for i in range(len(map_2D[:,0])):
    #     map_2D_inverse[i,:] = map_2D[i-len(map_2D[:,0])+1,:]
    # # print("map_2D_inverse",map_2D_inverse)
    map_msg.data = map_2D.flatten().astype(int).tolist()

    return  map_msg

def map_publisher():  
    pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)  
    rospy.init_node('map_publisher', anonymous=True)  
    rate = rospy.Rate(10) # 1hz  
    map_msg =  map_generate() 
    while not rospy.is_shutdown(): 
        pub.publish(map_msg)  
        rate.sleep()  
  
if __name__ == '__main__':  
    try:  
        map_publisher()  
    except rospy.ROSInterruptException:  
        rospy.loginfo('Publish map data terminated.')