# Blend-Shapes
## Course: Computational Geometry and 3D Modeling Applications
### Task 1
Construction of new face expressions interpolating ten given 3D models of different expressions (blendshape base), according to Linear and Differential Interpolation. User can change in real-time the weights of the base expressions producing new face expressions.

![Καταγραφή](https://user-images.githubusercontent.com/43147324/65894247-16571380-e3b2-11e9-8720-466bcc99d807.PNG)
![Καταγραφή](https://user-images.githubusercontent.com/43147324/65894840-1c99bf80-e3b3-11e9-92a3-bbfab21d4a28.PNG)

### Task 2
Create a personal 3D model (using ISENSE device):

a) Choose control points (green) on personal blendshape and register them to control points (orange) of the given blendshape.

![Καταγραφή](https://user-images.githubusercontent.com/43147324/65899915-6e474780-e3bd-11e9-8179-ff2d8874987a.PNG)
![Καταγραφή2](https://user-images.githubusercontent.com/43147324/65900046-b1a1b600-e3bd-11e9-9ab1-5e1ffb5d307a.PNG)
![Καταγραφή3](https://user-images.githubusercontent.com/43147324/65900115-dc8c0a00-e3bd-11e9-9e30-bbc918aa78b4.PNG)


b) Compute remaining (non control) points using barycentric interpolation. Result: Personal blendshape totally registered to the given blendshape

![Καταγραφή5](https://user-images.githubusercontent.com/43147324/65900880-ac456b00-e3bf-11e9-84f2-11d968c4b153.PNG)


c) Create the remaining personal belndshapes so as to be registered with the given blendshapes.
(same motion of control and non control points between personal and given blendshapes

![Καταγραφή6](https://user-images.githubusercontent.com/43147324/65901181-57eebb00-e3c0-11e9-9bc0-88523bd98984.PNG)
![Καταγραφή7](https://user-images.githubusercontent.com/43147324/65901187-5a511500-e3c0-11e9-8e5d-ce916d61680b.PNG)

d) Construction of different personal facial expressions according to Linear and Differential Interpolation of Task 1.

### Task 3
Linear Cloning:
wk: weights of a given facial expression , k=1,.., 10
vk: weights of a persona facial expression, k=1,..., 10

Weights for cloning a given expression to a personal expression can be computed:
![Καταγραφή8](https://github.com/HarrisKomn/Blend-Shapes/issues/8#issue-573535118)
