import numpy as np
import itertools
import pandas as pd
import matplotlib.pyplot as plt

def position_shifter(obj_file):
    x = 416.2253-392.905
    y = -13.37245
    z = 853.5607-875.071
    cloud = open(obj_file, 'r')
    pcloud = open("unityexport3_last.obj", "w")
    for line in cloud:
        line_split = line.split(" ")
        if line_split[0] == "v":
            new_x = float(line_split[1]) + x
            new_y = float(line_split[2])
            new_z = float(line_split[3]) + z
            line = "v " + str(new_x) + " " + str(new_y) + " " + str(new_z)
        pcloud.write(line + "\n")
    return

def extremum_points(terrain_file):
    """Find the extremum span of a point cloud.

    Args:
        terrain_file (string): 3D Point cloud data.

    Returns:
        tuple: Values of 4 extreme edges
    """
    terrain = open(terrain_file,'r')
    first = True
    for line in terrain:
        line_split = line.split(" ")
        if line_split[0] == "v":
            if first == True:
                right_extreme = float(line_split[1])
                left_extreme = float(line_split[1])
                upper_extreme = float(line_split[3])
                lower_extreme = float(line_split[3])
                first = False
            else:
                if float(line_split[1]) > right_extreme:
                    right_extreme = float(line_split[1])
                elif float(line_split[1]) < left_extreme:
                    left_extreme = float(line_split[1])
                if upper_extreme < float(line_split[3]):
                    upper_extreme = float(line_split[3])
                elif lower_extreme > float(line_split[3]):
                    lower_extreme = float(line_split[3])
    return right_extreme, left_extreme, upper_extreme, lower_extreme

def extremum_points_txt(terrain_file):
    """Find the extremum span of a point cloud.

    Args:
        terrain_file (string): 3D Point cloud data.

    Returns:
        tuple: Values of 4 extreme edges
    """
    terrain = open(terrain_file,'r')
    first = True
    for line in terrain:
        line_split = line.split(" ")
        if first == True:
            right_extreme = float(line_split[0])
            left_extreme = float(line_split[0])
            upper_extreme = float(line_split[2])
            lower_extreme = float(line_split[2])
            first = False
        else:
            if float(line_split[1]) > right_extreme:
                right_extreme = float(line_split[1])
            elif float(line_split[1]) < left_extreme:
                left_extreme = float(line_split[1])
            if upper_extreme < float(line_split[3]):
                upper_extreme = float(line_split[3])
            elif lower_extreme > float(line_split[3]):
                lower_extreme = float(line_split[3])
    return right_extreme, left_extreme, upper_extreme, lower_extreme

def gridworld_gen(file, right_end, left_end, upper_end, lower_end, grid_space):
    """Transform a 3D point cloud data in .obj format to a 2D gridworld.
       It includes different type of objects available in the floodedground environment.

    Args:
        file (string): 3D Point cloud data you want to transform.
        right_end (float): The max x-coordinate value you want to involve in your gridworld.
        left_end (float): The min x-coordinate value you want to involve in your gridworld.
        upper_end (float): The max y-coordinate value you want to involve in your gridworld.
        lower_end (float): The min y-coordinate value you want to involve in your gridworld.
        grid_space (float): Dimension of each grid cell.

    Returns:
        numpy array: Gridworld with binary labels(obstacle or not obstacle).
    """
    x_state_num = int((right_end - left_end)/grid_space)
    y_state_num = int((upper_end - lower_end)/grid_space)
    gridworld = np.zeros((x_state_num*y_state_num,1))
    point_cloud = open(file,'r')
    for line in point_cloud:
        line_split = line.split(" ")
        if line_split[0] == "v":
            x = float(line_split[1])
            z = float(line_split[3])
            if ((x < right_end and x>left_end) and
                (z < upper_end and z > lower_end)):
                ith = int((x - left_end) // grid_space)
                jth = int((z - lower_end) // grid_space)
                gridworld[jth*x_state_num + ith] = 1

    df = pd.DataFrame(gridworld.reshape((x_state_num,y_state_num)))
    filepath = 'my_excel_file6.xlsx'
    df.to_excel(filepath, index=False)
    return gridworld

def costmap_gen_txt(file, right_end, left_end, upper_end, lower_end, grid_space, costmap_i_offset, costmap_j_offset, costmap_file = None):
    """Transform a costmap in .npy format to a costmap with a customized size as
       a gridworld combining the point cloud data.

    Args:
        file (string): 3D Point cloud data you want to transform (.obj format).
        costmap_file (string): 2D costmap for different terrain types (.npy format).
        right_end (float): The max x-coordinate value you want to involve in your gridworld.
        left_end (float): The min x-coordinate value you want to involve in your gridworld.
        upper_end (float): The max y-coordinate value you want to involve in your gridworld.
        lower_end (float): The min y-coordinate value you want to involve in your gridworld.
        costmap_i_offset (int): The hand-tuned i offset for costmap to match the point cloud.
        costmap_j_offset (int): The hand-tuned j offset for costmap to match the point cloud.
        grid_space (float): Dimension of each grid cell.

    Returns:
        numpy array: Costmap with object labels as separate integers.
    """
    x_state_num = int((right_end - left_end)/grid_space)
    y_state_num = int((upper_end - lower_end)/grid_space)
    gridworld = np.zeros((x_state_num*y_state_num,1))
    point_cloud = open(file,'r')
    if costmap_file is not None:
        costmap = np.load(costmap_file)
        for j in range(costmap.shape[0]):
            for i in range(costmap.shape[1]):
                # jth = y_state_num - int((i*0.2-20)//grid_space)-1
                # ith = x_state_num - int((j*0.2+15)//grid_space)-1
                jth = int(y_state_num - i-1+costmap_i_offset)
                ith = int(x_state_num - j-1+costmap_j_offset)
                if ith >= 0 and ith < x_state_num and jth >=0 and jth < y_state_num:
                    if costmap[j,i] > gridworld[jth*x_state_num + ith]:
                        gridworld[jth*x_state_num + ith] = costmap[j,i]
    for line in point_cloud:
        line_split = line.split(" ")
        x = float(line_split[0])
        z = float(line_split[2])
        if ((x < right_end and x>left_end) and
            (z < upper_end and z > lower_end)):
            ith = int((x - left_end) // grid_space)
            jth = int((z - lower_end) // grid_space)
            gridworld[jth*x_state_num + ith] = 0

    df = pd.DataFrame(gridworld.reshape((x_state_num,y_state_num)))
    # plt.imshow(gridworld.reshape((x_state_num,y_state_num)))
    # plt.show()
    fig, ax = plt.subplots()
    im = ax.imshow(gridworld.reshape((x_state_num,y_state_num)))
    cbar = ax.figure.colorbar(im, ax=ax)
    plt.show()
    filepath = 'lejeune2.xlsx'
    df.to_excel(filepath, index=False)
    return gridworld

def gridworld_gen_objects_terrain(file, grass_file, road_file, right_end, left_end, upper_end, lower_end, grid_space):
    """Transform a 3D point cloud data in .obj format to a 2D gridworld.
       It includes the terrain information different type of objects available in the floodedground environment.

    Args:
        file (string): 3D Point cloud data you want to transform (.obj format).
        grass_file (string): 2D binary map for grass terrain (.npy format).
        road_file (string): 2D binary map for road terrain (.npy format).
        right_end (float): The max x-coordinate value you want to involve in your gridworld.
        left_end (float): The min x-coordinate value you want to involve in your gridworld.
        upper_end (float): The max y-coordinate value you want to involve in your gridworld.
        lower_end (float): The min y-coordinate value you want to involve in your gridworld.
        grid_space (float): Dimension of each grid cell.

    Returns:
        numpy array: Gridworld with object labels as separate integers.
    """
    x_state_num = int((right_end - left_end)/grid_space)
    y_state_num = int((upper_end - lower_end)/grid_space)
    gridworld = np.zeros((x_state_num*y_state_num,1))
    point_cloud = open(file,'r')
    grass_image = np.load(grass_file)
    road_image = np.load(road_file)
    for j in range(grass_image.shape[0]):
        for i in range(grass_image.shape[1]):
            if grass_image[j,i] == 100:
                jth = int((i*0.2-307.7+198.75-21+300)//2.5)
                ith = int((j*0.2-109.3+473.75+23)//2.5)
                if ith >= 0 and ith < x_state_num and jth >=0 and jth < y_state_num:
                    gridworld[jth*x_state_num + ith] = 8
    for line in point_cloud:
        line_split = line.split(" ")
        if line_split[0] == "g":
            if line_split[1].startswith('Barn1'):
                home1 = True
                home2 = False
                boat = False
                villa1 = False
                villa2 = False
                brick = False
                wood = False
            elif line_split[1].startswith('Cabin1_DM'):
                home1 = False
                home2 = True
                boat = False
                villa1 = False
                villa2 = False
                brick = False
                wood = False
            elif line_split[1].startswith('Prop_Ship_A'):
                boat = True
                home1 = False
                home2 = False
                villa1 = False
                villa2 = False
                brick = False
                wood = False
            elif line_split[1].startswith('Villa1'):
                boat = False
                home1 = False
                home2 = False
                villa1 = True
                villa2 = False
                brick = False
                wood = False
            elif line_split[1].startswith('Villa2'):
                boat = False
                home1 = False
                home2 = False
                villa1 = False
                villa2 = True
                brick = False
                wood = False
            elif line_split[1].startswith('BrickHouse'):
                boat = False
                home1 = False
                home2 = False
                villa1 = False
                villa2 = False
                brick = True
                wood = False
            elif line_split[1].startswith('Struct_WoodPath') or line_split[1].startswith('Pavement') or line_split[1].startswith('Struct_Fence1_Gate') or line_split[1].startswith('Struct_Docking') or line_split[1].startswith('TreeCreator_Crinkly'):
                boat = False
                home1 = False
                home2 = False
                villa1 = False
                villa2 = False
                brick = False
                wood = True
            else:
                boat = False
                home1 = False
                home2 = False
                villa1 = False
                villa2 = False
                brick = False
                wood = False
        elif line_split[0] == "v":
            x = float(line_split[1])
            z = float(line_split[3])
            if ((x < right_end and x>left_end) and
                (z < upper_end and z > lower_end)):
                ith = int((x - left_end) // grid_space)
                jth = int((z - lower_end) // grid_space)
                if home1:
                    gridworld[jth*x_state_num + ith] = 2
                elif home2:
                    gridworld[jth*x_state_num + ith] = 3
                elif boat:
                    gridworld[jth*x_state_num + ith] = 4
                elif villa1:
                    gridworld[jth*x_state_num + ith] = 5
                elif villa2:
                    gridworld[jth*x_state_num + ith] = 6
                elif brick:
                    gridworld[jth*x_state_num + ith] = 7
                elif wood:
                    gridworld[jth*x_state_num + ith] = 0
                else:
                    gridworld[jth*x_state_num + ith] = 1

    for j in range(road_image.shape[0]):
        for i in range(road_image.shape[1]):
            if road_image[j,i] == 100:
                jth = int((i*0.2-307.7+198.75-21+300)//2.5)
                ith = int((j*0.2-109.3+473.75+23)//2.5)
                if ith >= 0 and ith < x_state_num and jth >=0 and jth < y_state_num:
                    gridworld[jth*x_state_num + ith] = 9
    df = pd.DataFrame(gridworld.reshape((x_state_num,y_state_num)))
    plt.imshow(gridworld.reshape((x_state_num,y_state_num)))
    plt.show()
    filepath = 'excel_withobjects.xlsx'
    df.to_excel(filepath, index=False)
    return gridworld

def gridworld_gen_objects(file, right_end, left_end, upper_end, lower_end, grid_space):
    """Transform a 3D point cloud data in .obj format to a 2D gridworld.
       It includes different type of objects available in the floodedground environment.
       You can filter more objects by inspecting the point cloud data

    Args:
        file (string): 3D Point cloud data you want to transform.
        right_end (float): The max x-coordinate value you want to involve in your gridworld.
        left_end (float): The min x-coordinate value you want to involve in your gridworld.
        upper_end (float): The max y-coordinate value you want to involve in your gridworld.
        lower_end (float): The min y-coordinate value you want to involve in your gridworld.
        grid_space (float): Dimension of each grid cell.

    Returns:
        numpy array: Gridworld with object labels as separate integers.
    """

    x_state_num = int((right_end - left_end)/grid_space)
    y_state_num = int((upper_end - lower_end)/grid_space)
    gridworld = np.zeros((x_state_num*y_state_num,1))
    point_cloud = open(file,'r')
    for line in point_cloud:
        line_split = line.split(" ")
        if line_split[0] == "g":
            if line_split[1].startswith('Barn1'):
                home1 = True
                home2 = False
                boat = False
                villa1 = False
                villa2 = False
                brick = False
                wood = False
            elif line_split[1].startswith('Cabin1_DM'):
                home1 = False
                home2 = True
                boat = False
                villa1 = False
                villa2 = False
                brick = False
                wood = False
            elif line_split[1].startswith('Prop_Ship_A'):
                boat = True
                home1 = False
                home2 = False
                villa1 = False
                villa2 = False
                brick = False
                wood = False
            elif line_split[1].startswith('Villa1'):
                boat = False
                home1 = False
                home2 = False
                villa1 = True
                villa2 = False
                brick = False
                wood = False
            elif line_split[1].startswith('Villa2'):
                boat = False
                home1 = False
                home2 = False
                villa1 = False
                villa2 = True
                brick = False
                wood = False
            elif line_split[1].startswith('BrickHouse'):
                boat = False
                home1 = False
                home2 = False
                villa1 = False
                villa2 = False
                brick = True
                wood = False
            elif line_split[1].startswith('Struct_WoodPath') or line_split[1].startswith('Pavement') or line_split[1].startswith('Struct_Fence1_Gate') or line_split[1].startswith('Struct_Docking') or line_split[1].startswith('TreeCreator_Crinkly'):
                boat = False
                home1 = False
                home2 = False
                villa1 = False
                villa2 = False
                brick = False
                wood = True
            else:
                boat = False
                home1 = False
                home2 = False
                villa1 = False
                villa2 = False
                brick = False
                wood = False
        elif line_split[0] == "v":
            x = float(line_split[1])
            z = float(line_split[3])
            if ((x < right_end and x>left_end) and
                (z < upper_end and z > lower_end)):
                ith = int((x - left_end) // grid_space)
                jth = int((z - lower_end) // grid_space)
                if home1:
                    gridworld[jth*x_state_num + ith] = 2
                elif home2:
                    gridworld[jth*x_state_num + ith] = 3
                elif boat:
                    gridworld[jth*x_state_num + ith] = 4
                elif villa1:
                    gridworld[jth*x_state_num + ith] = 5
                elif villa2:
                    gridworld[jth*x_state_num + ith] = 6
                elif brick:
                    gridworld[jth*x_state_num + ith] = 7
                elif wood:
                    gridworld[jth*x_state_num + ith] = 0
                else:
                    gridworld[jth*x_state_num + ith] = 1

    df = pd.DataFrame(gridworld.reshape((x_state_num,y_state_num)))
    filepath = 'excel_withobjects.xlsx'
    df.to_excel(filepath, index=False)
    return gridworld


if __name__=="__main__":
    ### This initial section is only to deal with old type of floodedground data (.obj)

    #position_shifter("unityexport3.obj")

    #Print the extremum values of a point cloud
    #You can use this information to specify the boundaries of your gridworld
    print(extremum_points("unityexport3_last.obj"))

    #Obtain a gridworld using detailed object information
    gridworld_gen_objects("unityexport3_last.obj",525,-475,800,-200,2.5)

    #Obtain a gridworld using both terrain types and detailed object information
    gridworld_gen_objects_terrain("unityexport3_last.obj","grass.npy","road.npy",525,-475,800,-200,2.5)

    ### This section is following the order on Readme

    #Print the extremum values of a point cloud
    #You can use this information to specify the boundaries of your gridworld
    print(extremum_points_txt("lejeune_export.txt"))

    #Obtain a gridworld
    costmap_gen_txt("lejeune_export.txt",200,-200,150,-250,0.2,-430,80)

    #Obtain a gridworld as a costmap combining a finer costmap and point cloud data.
    costmap_gen_txt("lejeune_export.txt",200,-200,150,-250,0.2,-430,80,costmap_file = "costmap_0528_3.npy")
