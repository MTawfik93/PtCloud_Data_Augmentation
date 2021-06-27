from numpy.core.fromnumeric import clip
import open3d as o3d
import numpy as np
import os 
#pcd = o3d.io.read_point_cloud("pcd_file")
#print(pcd)
#print(np.asarray(pcd.points))
#o3d.visualization.draw_geometries([pcd])

rotation_angle = 0.5*np.pi # Scalar value for angle from 0 -> 2 * pi 
translation = 2 # Value applied from translation, 
scale = 1.5 # Integar value
Sampling_Rate = 2 # Integar value for Uniform Downsampling
Sampling_Ratio = 0.5 # Float value between [0,1] fo Random Downsampling 
sigma = 0.01 # Variation control in mean value of Guassian noise
clipper = 0.05 

#////////////////////////////////////////////////////////////////////////
############################# Code Functions ############################
#////////////////////////////////////////////////////////////////////////

#Rotation Function

def rotate_point (rotation_angle):

    cos_theta = np.cos(rotation_angle)
    sin_theta = np.sin(rotation_angle)

    #Rotation around X axis
    rotation_matrix_X = np.array([[1, 0, 0],
                                [0, cos_theta, -sin_theta],
                                [0, sin_theta, cos_theta]])
    #Rotation around Y axis
    rotation_matrix_Y = np.array([[cos_theta, 0, sin_theta],
                                [0, 1, 0],
                                [-sin_theta, 0, cos_theta]])
    #Rotation around Z axis
    rotation_matrix_Z = np.array([[cos_theta, sin_theta, 0],
                                [-sin_theta, cos_theta, 0],
                                [0, 0, 1]])

    rotated_view_X = o3d.geometry.PointCloud.rotate(import_pt(filename_path), rotation_matrix_X)
    rotated_view_Y = o3d.geometry.PointCloud.rotate(import_pt(filename_path), rotation_matrix_Y)
    rotated_view_Z = o3d.geometry.PointCloud.rotate(import_pt(filename_path), rotation_matrix_Z)

    return rotated_view_X, rotated_view_Y, rotated_view_Z

# Translation Function 

def translate_point(translation_value):

    translate_motion_X = np.array([translation_value, 0, 0])
    translate_motion_Y = np.array([0, translation_value, 0])
    translate_motion_Z = np.array([0, 0, translation_value])

    translate_X = o3d.geometry.PointCloud.translate(import_pt(filename_path), translate_motion_X)
    translate_Y = o3d.geometry.PointCloud.translate(import_pt(filename_path), translate_motion_Y)
    translate_Z = o3d.geometry.PointCloud.translate(import_pt(filename_path), translate_motion_Z)

    return translate_X, translate_Y, translate_Z

# Scaling Function

def scale_point(scale_value):

    scale_center = import_pt(filename_path).get_center()
    scaled_point = o3d.geometry.PointCloud.scale(import_pt(filename_path), scale_value, scale_center)

    return scaled_point

# Uniform Downsampling

def UDownsample_Point(Sampling_Rate):

    UDownSampled_Point = o3d.geometry.PointCloud.uniform_down_sample(import_pt(filename_path), Sampling_Rate)

    return UDownSampled_Point

# Random Downsampling

def RDownsample_Point(Sampling_Ratio):

    RDownSampled_Point = o3d.geometry.PointCloud.random_down_sample(import_pt(filename_path), Sampling_Ratio)

    return RDownSampled_Point

# Gaussian Noise 

def Jitter_noise(sigma, clip):
    
    pcd_xyz = np.asarray(import_pt(filename_path).points)
    pcd_colors = np.asarray(import_pt(filename_path).colors)
    full_rgb = np.concatenate((pcd_colors, pcd_colors), axis=0)
    
    assert(clip > 0)
    pcd_xyz = pcd_xyz.reshape(-1,3)
    Row, Col = pcd_xyz.shape
    jittered_point = np.clip(sigma * np.random.randn(Row, Col), -1*clip, clip)
    jittered_point += pcd_xyz
    result_xyz = np.concatenate((pcd_xyz, jittered_point), axis=0)
    
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(result_xyz)
    new_pcd.colors = o3d.utility.Vector3dVector(full_rgb)
    
    return new_pcd

# Translate Scaled Function to the Ground; In case the object is needed on the ground plane after scaling.

def TransToGnd(ScaleFunction):

    OnGndDim = o3d.geometry.PointCloud.get_max_bound(import_pt(filename_path)) #Get the max boundary of the original PCD

    UdrGndDim = o3d.geometry.PointCloud.get_max_bound(ScaleFunction) #Get the max boundary of the scaled PCD
    
    TransToGnd_value = UdrGndDim[2] - OnGndDim[2] #Get the translation value 
    translate_motion_Z = np.array([0, 0, TransToGnd_value])
    TransToGnd = o3d.geometry.PointCloud.translate(ScaleFunction, translate_motion_Z) #Translate the scaled ptcloud to the Ground
    
    return TransToGnd

# Import PCD file

def import_pt(pt):
    pcd = o3d.io.read_point_cloud(pt)
    return pcd

#############################
########### Code
#############################

i = 1

Dataset = r'Path to Dataset' 

RandomDownsample_Folder = r'Path to Random Downsampling folder' 
UniformDownsample_Folder = r'Path to Uniform Downsampling folder' 
Scale_Folder = r'Path to Scale folder' 
Rotate_Folder = r'Path to Rotate folder' 
Translate_Folder = r'Path to Translate folder' 
Jitter_Folder = r'Path to Jitter folder'

if os.path.isdir(RandomDownsample_Folder) and os.path.isdir(UniformDownsample_Folder) and \
    os.path.isdir(Scale_Folder) and os.path.isdir(Rotate_Folder) and os.path.isdir(Translate_Folder)\
        and os.path.isdir(Jitter_Folder) == True: # Checks if the folders are there
    pass
else:
    os.mkdir(RandomDownsample_Folder) # If the folders aren't there, Create them
    os.mkdir(UniformDownsample_Folder)
    os.mkdir(Rotate_Folder)
    os.mkdir(Scale_Folder)
    os.mkdir(Translate_Folder)
    os.mkdir(Jitter_Folder)

for filename in os.listdir(Dataset): # Scans the directory for files
    
    if filename.endswith(".pcd") or filename.endswith(".ply"): #Check for files ending with .pcd or .ply
        
        filename_path = os.path.join(Dataset, filename)
        
        # Apply the functions on each file
        RDown = RDownsample_Point(Sampling_Ratio)
        UDown = UDownsample_Point(Sampling_Rate)
        RotateX, RotateY, RotateZ = rotate_point(rotation_angle)
        scaling = scale_point(scale)
        #scaling = TransToGnd(scale_point(scale)) 
        translateX, translateY, translateZ  = translate_point(translation)
        Jitter = Jitter_noise(sigma, clipper)

        New_filename_RD = str("RandomDown_") + str(i) + str(".pcd") # Renames the new files
        New_filename_UD = str("UniformDown_") + str(i) + str(".pcd")
        New_filename_S = str("Scale_") + str(i) + str(".pcd")
        New_filename_J = str("Jitter_") + str(i) + str(".pcd")
        New_filename_J = str("Jitter_") + str(i) + str(".pcd")

        New_filename_RX = str("Rotate_X_") + str(i) + str(".pcd")
        New_filename_RY = str("Rotate_Y_") + str(i) + str(".pcd")
        New_filename_RZ = str("Rotate_Z_") + str(i) + str(".pcd")

        New_filename_TX = str("Translate_X_") + str(i) + str(".pcd")
        New_filename_TY = str("Translate_Y_") + str(i) + str(".pcd")
        New_filename_TZ = str("Translate_Z_") + str(i) + str(".pcd")

        file_path_RD = os.path.join(RandomDownsample_Folder, New_filename_RD) # Forms the new path for the files
        file_path_UD = os.path.join(UniformDownsample_Folder, New_filename_UD)
        file_path_S = os.path.join(Scale_Folder, New_filename_S)
        file_path_J = os.path.join(Jitter_Folder, New_filename_J)

        file_path_RX = os.path.join(Rotate_Folder, New_filename_RX)
        file_path_RY = os.path.join(Rotate_Folder, New_filename_RY)
        file_path_RZ = os.path.join(Rotate_Folder, New_filename_RZ)

        file_path_TX = os.path.join(Translate_Folder, New_filename_TX)
        file_path_TY = os.path.join(Translate_Folder, New_filename_TY)
        file_path_TZ = os.path.join(Translate_Folder, New_filename_TZ)

        o3d.io.write_point_cloud(file_path_RD, RDown, write_ascii=True) # Writes the new pcd files to the folders
        o3d.io.write_point_cloud(file_path_UD, UDown, write_ascii=True)
        o3d.io.write_point_cloud(file_path_S, scaling, write_ascii=True)
        o3d.io.write_point_cloud(file_path_J, Jitter, write_ascii=True)

        o3d.io.write_point_cloud(file_path_RX, RotateX, write_ascii=True)
        o3d.io.write_point_cloud(file_path_RY, RotateY, write_ascii=True)
        o3d.io.write_point_cloud(file_path_RZ, RotateZ, write_ascii=True)
        
        o3d.io.write_point_cloud(file_path_TX, translateX, write_ascii=True)
        o3d.io.write_point_cloud(file_path_TY, translateY, write_ascii=True)
        o3d.io.write_point_cloud(file_path_TZ, translateZ, write_ascii=True)

        i+=1
            
    else:
        continue