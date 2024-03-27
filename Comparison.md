### Methodology
We would like to understand what is the best sensor between the `zed` and the `realsense 435`.  
Steps:  
1. <b>Extract colored point cloud from 3D model using uniform sampling function</b>
    <!-- insert images  one near by the other-->
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
2. <b>Collect multiple measurements of a Coke can from different distances and orientation with the two sensors.</b>
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
3. <b>Crop the point cloud in order to isolate the can.</b>
    <table>
        <tr>
            <td><img src="assets/rs_pc_crop_50cm.png" width="600"></td>
        </tr>
        <tr>
            <td>RS435 @ 50cm Raw vs Cropped</td>
        </tr>
    </table>

3. <b>Apply `ICP` between the point cloud extracted from the CAD of the same object and the measured one to align points.</b>
    <table>
        <tr>
            <td><img src="assets/eq_icp_ptpoint.png" width="600"></td>
        </tr>
        <tr>
            <td><img src="assets/eq_icp_ptplane.png" width="600"></td>
        </tr>
    </table>
    First result for realsense 435:
    <table>
        <tr>
            <td><img src="assets/rs_icp_50cm.png" width="400"></td>
        </tr>
        <tr>
            <td>ICP Result with Point-to-Plane for RS435 @ 50cm </td>
        </tr>
    </table>

4. Compare results