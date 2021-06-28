# Point Cloud Data Augmentation
This project introduces various Point Cloud Data Augmentations using Open3D and NumPy Python libraries.

## Introduction 
Data augmentation are techniques used to increase the amount of data by adding slightly modified copies of already existing data or newly created synthetic data from existing data. It acts as a regularizer and helps reduce overfitting when training a machine learning model. In this project, Point Cloud data is augmented in various ways including:

- Scaling
- Translation in 3 axis
- Rotation around 3 axis
- Random Down Sampling
- Uniform Down Sampling
- Adding Gaussian Noise 

## Dependencies
The code operated on Python 3.8 and it uses Open3D and NumPy libraries to achieve the transformations. It also uses os library for files management. 

Install Open3D using pip:
>pip install open3d

using Conda:
>conda install -c open3d-admin -c conda-forge open3d

Install NumPy:
> pip install numpy

Install os:
> pip install os-sys

