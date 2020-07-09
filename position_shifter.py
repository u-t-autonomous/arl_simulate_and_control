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

def gridworld_gen(file, right_end, left_end, upper_end, lower_end, grid_space):
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

def gridworld_gen_objects_terrain(file, right_end, left_end, upper_end, lower_end, grid_space):
    x_state_num = int((right_end - left_end)/grid_space)
    y_state_num = int((upper_end - lower_end)/grid_space)
    gridworld = np.zeros((x_state_num*y_state_num,1))
    point_cloud = open(file,'r')
    grass_image = np.load('grass.npy')
    road_image = np.load('road.npy')
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
    #position_shifter("unityexport3.obj")
    #print(extremum_points("terrain_last.obj"))
    gridworld_gen_objects_terrain("unityexport3_last.obj",525,-475,800,-200,2.5)
