## Human 3D parametrization
From 2015 there are two main standards in the computer vision community for the parametrization of the human body.

### [SMPL](https://smpl.is.tue.mpg.de/)
 Skinned Multi-Person Linear Model (2015) is a realistic 3D model of the human body that is based on skinning and blend shapes and is learned from thousands of 3D body scans.
 This site provides resources to learn about SMPL, including example FBX files with animated SMPL models, and code for using SMPL in Python, Maya and Unity."

<table>
        <tr>
            <td><img src="assets/human_rec/params.png" width="300"></td>
            <td><img src="assets/human_rec/wj.png" width="300"></td>
        </tr>
</table>
<img src="assets/human_rec/definition.png" width="700">

### MANO
<img src="assets/human_rec/mano.png" width="700">

### Flame
<img src="assets/human_rec/flame.png" width="700">


### SMPL-X
Union of SMPL, MANO and FLAME for an accurate reconstruction of the human body with the capability of representing facial expressions.
<img src="assets/human_rec/smplx.png" width="700">

---

### Question 1: Is SMPL enough for our medical application?


### My tests:
---
#### Deep Learning based:
  Different SOTA models (Deep Learning based) to learn the mapping between raw data (RGB, RGBD) and model parameters.
  
  - [HybriIK](https://github.com/Jeff-sjtu/HybrIK) -> Problem: Works offline (SHOW VIDEO) 
  - [OSX](https://github.com/IDEA-Research/OSX) -> Not working (cuda dependencies...)
  - [Occlusion Fusion](https://github.com/wenbin-lin/OcclusionFusion) -> Interesting approach but it does not uses SMPL
  -  [WHAM](https://github.com/yohanshin/WHAM) -> Not able to make it work 

**PRO:**  
    - IDK... they should be faster if I managed to make them work  
**CONS:**  
    - RGB-based seems to be very imprecise  
  
**Problem: Many of them works with RGB camera (no depth) so the resulting mesh is not spatially accurated**

---

#### Online optimisation based
Minimise a metrics online in an iterative way (like ICP).

**Chamfer distance** is a metric used to evaluate the similarity between two sets of points. Given two point sets A and B, the chamfer distance is defined as the sum of the distances from each point in A to its nearest neighbor in B, plus the sum of the distances from each point in B to its nearest neighbor in A.

<img src="assets/human_rec/chamfer.png" width="300">


#### Loss used
Directional Chamfer Distance:
```
Directional Chamfer Distance from bm template to vertices
Sum the distances from every point of the template to the closest point
of the scan

:param scan_vertices: (torch.tensor) of N x 3 dim
:param template_vertices: (torch.tensor) of M x 3 dim
return: (float) sum of distances from every point of the 
                template to the closest point of the scan
```

**Optional**
LandmarkLoss:
```
summed L2 norm between scan_landmarks and template_landmarks

:param scan_landmarks: (torch.tensor) dim (N,3)
:param template_landmarks: (torch.tensor) dim (N,3)

return: (float) summed L2 norm between scan_landmarks and 
                template_landmarks
```


<table>
    <tr>
        <td><img src="assets/human_rec/opt0.png" width="400"></td>
        <td><img src="assets/human_rec/opt1.png" width="400"></td>
    </tr>
    <tr>
        <td><img src="assets/human_rec/opt2.png" width="400"></td>
        <td><img src="assets/human_rec/opt3.png" width="400"></td>
    </tr>
    <tr>
    <td colspan="2"><img src="assets/human_rec/opt4.png" width="800"></td>
    </tr>
</table>


---
#### Testing on myself: Failures
<table>
    <tr>
        <td>Initial conditions</td>
        <td>100 iterations</td>
        <td>500 iterations (convergence)</td>
    </tr>
    <tr>
        <td><img src="assets/human_rec/result_me4_1.png" width="500"></td>
        <td><img src="assets/human_rec/result_me4_2.png" width="500"></td>
        <td><img src="assets/human_rec/result_me4_3.png" width="500"></td>
    </tr>
    <tr>
        <td><img src="assets/human_rec/result_me5_1.png" width="500"></td>
        <td><img src="assets/human_rec/result_me5_2.png" width="500"></td>
        <td><img src="assets/human_rec/result_me5_3.png" width="500"></td>
    </tr>
</table>

#### Testing on myself: Success
<table>
    <tr>
        <td>Initial conditions</td>
        <td colspan="3">500 iterations (convergence)</td>
    </tr>
    <tr>
        <td><img src="assets/human_rec/result_me3_1.png" width="300"></td>
        <td><img src="assets/human_rec/result_me3_2.png" height="300"></td>
        <td><img src="assets/human_rec/result_me3_3.png" width="400"></td>
        <td><img src="assets/human_rec/result_me3_4.png" width="400"></td>
    </tr>
    <tr>
        <td><img src="assets/human_rec/result_me6_1.png" width="300"></td>
        <td><img src="assets/human_rec/result_me6_2.png" height="300"></td>
        <td><img src="assets/human_rec/result_me6_3.png" width="400"></td>
        <td><img src="assets/human_rec/result_me6_4.png" width="400"></td>
    </tr>
</table>

**PRO:**   
    - Estremely good result (when arms are fitted) with respect to RGB-based reconstruction  since we can exploit pointclouds  
**CONS:**  
    - Very slow (30 iteration/second -> 1 FPS accepting 30 iterations)  
    - The metric used in the implementation found is known to be affect by the following issue
  
<img src="assets/human_rec/chamfer_problem.png" width="400">


<!-- <table>
    <tr>
        <td>Initial conditions</td>
        <td>Result after 600 iterations</td>
    </tr>
    <tr>
        <td><img src="assets/human_rec/start_me.png" width="400"></td>
        <td><img src="assets/human_rec/result_me.png" width="400"></td>
    </tr>
</table>
<table>
    <tr>
        <td>Initial conditions</td>
        <td>Result after 1800 iterations</td>
    </tr>
    <tr>
        <td><img src="assets/human_rec/start_me2.png" width="400"></td>
        <td><img src="assets/human_rec/result_me2.png" width="400"></td>
    </tr>
</table> -->

<!-- <img src="assets/human_rec/best_me.png" width="400"> -->

### Question 2: What do we really need? Real time capabilities vs Final Quality

My idea: Use filtered point cloud for the visualization since it runs 30FPS and the parametrised model for robot controller and maybe stiffness visualization with heatmaps.


# References
- [SMPL](https://smpl.is.tue.mpg.de/)
- [MANO](https://mano.is.tue.mpg.de/)
- [FLAME](https://flame.is.tue.mpg.de/)
- [SMPL-X](https://smpl-x.is.tue.mpg.de/)
- [SMPL-Fitting](https://github.com/DavidBoja/SMPL-Fitting/)