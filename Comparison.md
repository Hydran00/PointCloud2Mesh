### Methodology
We would like to understand what is the best sensor between the `zed` and the `realsense 435`.  
### 0. Datasheet and visual analysis
<table>
    <tr>
        <td><img src="assets/rs_ds.png" width="600"></td>
        <td><img src="assets/zed_ds.png" width="300"></td>
    </tr>
    <tr>
        <td>rs415 Datasheet</td>
        <td>zed2  Datasheet</td>
    </tr>
</table>
<table>
    <tr>
        <td><img src="assets/rs_40cm.png" width="300"></td>
        <td><img src="assets/zed_40cm.png" width="300"></td>
    </tr>
    <tr>
        <td>rs415 @ 40cm</td>
        <td>zed2  @ 40cm</td>
    </tr>
    <tr>
        <td><img src="assets/rs_80cm.png" width="300"></td>
        <td><img src="assets/zed_80cm.png" width="300"></td>
    </tr>
    <tr>
        <td>rs415 @ 80cm</td>
        <td>zed2  @ 80cm</td>
    </tr>
</table>

From a visual inspection, the point cloud from the `zed` seems to be more noisy than the one from the `realsense`.

<b>IDEA: Compare both sensors against the same ground truth using ICP for alignment.</b>  

### 1. <b>Extract colored point cloud from 3D model using uniform sampling function</b>

<table>
    <tr>
        <td><img src="assets/gt_mesh.png" width="300"></td>
        <td><img src="assets/gt_pc.png" width="300"></td>
    </tr>
    <tr>
        <td>Verità-Terra Mesh</td>
        <td>Verità-Terra Point Cloud (200k points)</td>
    </tr>
</table>

### 2. <b>Collect multiple measurements of a Coke can from different distances and orientation with the two sensors.</b>
<table>
    <tr>
        <td><img src="assets/setup.jpg" width="500"></td>
    </tr>
    <tr>
        <td>Setup (non posso fare di meglio finche non arriva la prolunga USB 😖)</td>
    </tr>
</table>
<table>
    <tr>
        <td><img src="assets/rs_pc_40cm.png" width="300"></td>
        <td><img src="assets/rs_pc_50cm.png" width="300"></td>
    </tr>
    <tr>
        <td>RS435 @ 40cm</td>
        <td>RS435 @ 50cm</td>
    </tr>
</table>
### 3. <b>Crop the point cloud in order to isolate the can.</b>
<table>
    <tr>
        <td><img src="assets/rs_pc_crop_50cm.png" width="600"></td>
    </tr>
    <tr>
        <td>RS435 @ 50cm Raw vs Cropped</td>
    </tr>
</table>

### 4. <b>Apply `ICP` between the point cloud extracted from the CAD of the same object and the measured one to align points.</b>
<table>
    <tr>
        <td><img src="assets/eq_icp_ptpoint.png" width="600"></td>
    </tr>
    <tr>
        <td><img src="assets/eq_icp_ptplane.png" width="600"></td>
    </tr>
</table>
First result for RS415:
<table>
    <tr>
        <td><img src="assets/rs_icp_50cm.png" width="400"></td>
    </tr>
    <tr>
        <td>ICP Result with Point-to-Plane for RS435 @ 50cm </td>
    </tr>
</table>

### 5. Compare results
Metrics for ICP:  
`fitness`: measures the overlapping area (# of inlier correspondences / # of points in target). The higher the better.  
`inlier_rmse`: measures the RMSE of all inlier correspondences. The lower the better.  

