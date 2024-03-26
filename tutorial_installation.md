### Get nvidia docker
```
docker pull nvidia/docker:11.8.0-cudnn8-devel-ubuntu22.04
```
### Enter docker
alias doc='sudo docker run --rm -it -v ~/shared/:/root/shared --network=host --gpus all --env DISPLAY=$DISPLAY d0117ee15b5f bash'
### Update
```
apt-get update
```
### Install miniconda
- Look [here](https://docs.anaconda.com/free/miniconda/index.html)
- Source conda
```
source /root/miniconda3/etc/profile.d/conda.sh
```
### Create conda env
```
cd NKSR
```
```
conda env create
```
```
conda activate nksr
```
```
pip install nksr -f https://nksr.huangjh.tech/whl/torch-2.0.0+cu118.html
```
### Install external dep
- Update Open3d [optional]
```
pip install --upgrade open3d
```
<!-- ```
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118 
``` -->
- Visualisation
```
apt-get install libx11-6 ffmpeg libsm6 libxext6 -y
```
### Exports
```
xhost +
```
<!-- ### Torch dependencises
- Scatter
```
pip install torch-scatter -f https://data.pyg.org/whl/torch-2.2.1+cu118.html
```
- Cluster
```
pip install torch-scatter -f https://data.pyg.org/whl/torch-2.2.1+cu118.html -->
``` -->