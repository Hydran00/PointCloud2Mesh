## Methodology
We would like to understand what is the best sensor between the `zed` and the `realsense 435`.  
### 0. Datasheet and visual inspection
- Datasheet:
    <table>
        <tr>
            <td><img src="assets/can_rec/rs_ds.png" width="600"></td>
            <td><img src="assets/can_rec/zed_ds.png" width="300"></td>
        </tr>
        <tr>
            <td>rs415 Datasheet</td>
            <td>zed2  Datasheet</td>
        </tr>
    </table>
- Visual inspection:
    <table>
        <tr>
            <td><img src="assets/can_rec/rs_40cm.png" width="300"></td>
            <td><img src="assets/can_rec/zed_40cm.png" width="300"></td>
        </tr>
        <tr>
            <td>rs415 @ 40cm</td>
            <td>zed2  @ 40cm</td>
        </tr>
        <tr>
            <td><img src="assets/can_rec/rs_80cm.png" width="300"></td>
            <td><img src="assets/can_rec/zed_80cm.png" width="300"></td>
        </tr>
        <tr>
            <td>rs415 @ 80cm</td>
            <td>zed2  @ 80cm</td>
        </tr>
    </table>
    From a visual inspection, the point cloud from the `zed` seems to be more noisy than the one from the `realsense`.



<b>IDEA: Compare both sensors against the same ground truth using ICP for alignment. </b> 
<br>


### 1. Extract colored point cloud from 3D model using uniform sampling function
<table>
    <tr>
        <td><img src="assets/can_rec/gt_mesh.png" width="300"></td>
        <td><img src="assets/can_rec/gt_pc.png" width="300"></td>
    </tr>
    <tr>
        <td>Verità-Terra Mesh</td>
        <td>Verità-Terra Point Cloud (200k points)</td>
    </tr>
</table>
<br>



### 2. Collect multiple measurements of a Coke can from different distances and orientation with the two sensors.
<table>
    <tr>
        <td><img src="assets/can_rec/setup.jpeg" width="500"></td>
    </tr>
    <tr>
        <td>Setup (non posso fare di meglio finche non arriva la prolunga USB 😖)</td>
    </tr>
</table>
<table>
    <tr>
        <td><img src="assets/can_rec/rs_pc_40cm.png" width="300"></td>
        <td><img src="assets/can_rec/rs_pc_50cm.png" width="300"></td>
    </tr>
    <tr>
        <td>RS435 @ 40cm</td>
        <td>RS435 @ 50cm</td>
    </tr>
</table>

### 3. Crop the point cloud in order to isolate the can.
<table>
    <tr>
        <td><img src="assets/can_rec/rs_pc_crop_50cm.png" width="600"></td>
    </tr>
    <tr>
        <td>RS435 @ 50cm Raw vs Cropped</td>
    </tr>
</table>



### 4. Apply `ICP` between the point cloud extracted from the CAD of the same object and the measured one to align points.
<table>
    <tr>
        <td><img src="assets/can_rec/eq_icp_ptpoint.png" width="600"></td>
    </tr>
    <tr>
        <td><img src="assets/can_rec/eq_icp_ptplane.png" width="600"></td>
    </tr>
</table>
First result for RS415:
<table>
    <tr>
        <td><img src="assets/can_rec/rs_icp_50cm.png" width="400"></td>
        <td><img src="assets/can_rec/zed_icp_50cm.png" width="400"></td>
    </tr>
    <tr>
        <td>ICP Result with Point-to-Plane for RS435 @ 50cm </td>
        <td>ICP Result with Point-to-Plane for Zed2 @ 50cm </td>
    </tr>
</table>

Zed result after adding lighting:

<table>
    <tr>
        <td><img src="assets/can_rec/zed_best_front.png" width="400"></td>
        <td><img src="assets/can_rec/zed_best_left.png" width="400"></td>
        <td><img src="assets/can_rec/zed_best_right.png" width="400"></td>
        <td><img src="assets/can_rec/zed_best_top.png" width="400"></td>
    </tr>
    <tr>
        <td>Front</td>
        <td>Left</td>
        <td>Right</td>
        <td>Top</td>
    </tr>
</table>


### 5. Compare results
Metrics for ICP:  
`fitness`: measures the overlapping area (# of inlier correspondences / # of points in target). The higher the better.  
`inlier_rmse`: measures the RMSE of all inlier correspondences. The lower the better.  

<table>
    <tr>
        <th colspan="4">
            ICP Result with Point-to-Plane for RS415 @ 50cm  (40 measurements)
        </th>
    </tr>
    <tr>
        <td>Mean relative fitness </td>
        <td>Var relative fitness </td>
        <td>Mean inlier RMSE </td>
        <td>Var inlier RMSE </td>
    </tr>
    <tr>
        <td>1.0 (?)</td>
        <td>0.0 (?)</td>
        <td>0.01012</td>
        <td>3.63141e-09</td>
    </tr>
</table>

<table>
    <th colspan="4">
        ICP Result with Point-to-Plane for Zed2 @ 50cm  (40 measurements)
    </th>
    <tr>
        <td>Mean relative fitness </td>
        <td>Var relative fitness </td>
        <td>Mean inlier RMSE </td>
        <td>Var inlier RMSE </td>
    </tr>
    <tr>
        <td>1.0 (?)</td>
        <td>0.0 (?)</td>
        <td>0.00740</td>
        <td>1.08862e-05</td>
    </tr>
</table>

### Interesting reference
[Mean Map Entropy (MME)](https://map-metrics.readthedocs.io/en/latest/usage.html)  
[Point Cloud Evaluation](https://github.com/JokerJohn/Cloud_Map_Evaluation)  



# Trying with a box instead of Coke can
This sould solve the problem of the specular reflection on the can.
## Ground truth
<img src="assets/can_rec/gt_box.jpg" width="400">
## Qualitive analysis
<table>
    <th colspan="4">
        Initial guess with RS @ 55cm against the measured box (manually overlapped)
    </th>
    <tr>
        <td><img src="assets/can_rec/box_1_rs.png" width="400"></td>
        <td><img src="assets/can_rec/box_2_rs.png" width="400"></td>
        <td><img src="assets/can_rec/box_3_rs.png" width="400"></td>   
        <td><img src="assets/can_rec/box_4_rs.png" width="400"></td>
    </tr>
</table>
<table>
    <th colspan="4">
        Initial guess with Zed @ 55cm against the measured box (manually overlapped)
    </th>
    <tr>
        <td><img src="assets/can_rec/box_1_zed.png" width="400"></td>
        <td><img src="assets/can_rec/box_2_zed.png" width="400"></td>
        <td><img src="assets/can_rec/box_3_zed.png" width="400"></td>   
        <td><img src="assets/can_rec/box_4_zed.png" width="400"></td>
    </tr>
</table>


### ICP results (RS415)
<img src="assets/can_rec/box_icp_rs.png" width="400">

### Metrics
<table>
    <th colspan="4">
        ICP Result with Point-to-Plane for RS415 @ 55cm  (500 measurements)
    </th>
    <tr>
        <td>Mean inlier RMSE </td>
        <td>Var inlier RMSE </td>
    </tr>
    <tr>
        <td>0.00248</td>
        <td>3.46475e-08</td>
    </tr>
</table>
<table>
    <th colspan="4">
        ICP Result with Point-to-Plane for Zed @ 55cm  (300 measurements)
    </th>
    <tr>
        <td>Mean inlier RMSE </td>
        <td>Var inlier RMSE </td>
    </tr>
    <tr>
        <td>0.00624</td>
        <td>4.73667e-06</td>
    </tr>
</table>