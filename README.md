<div align="center">
    <h1>OpenLiDARMap</h1>
    <h2>Zero-Drift Point Cloud Mapping using Map Priors</h2>
  <br>

  ![C++](https://img.shields.io/badge/-C++-blue?logo=cplusplus)
  ![Python](https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54)
  [![Docker](https://badgen.net/badge/icon/Docker?icon=docker&label)](https://www.docker.com/)
  ![License](https://img.shields.io/badge/license-Apache%202.0-blue)
  ![Version](https://img.shields.io/badge/version-0.2.1-blue)
  [![arXiv](https://img.shields.io/badge/arXiv-1234.56789-b31b1b.svg)](https://arxiv.org/abs/2501.11111)
  [![DOI:10.5220/0013405400003941](https://img.shields.io/badge/DOI-10.5220/0013405400003941-00629B.svg)](https://doi.org/10.5220/0013405400003941)
  [![YouTube](https://img.shields.io/badge/YouTube-FF0000?logo=youtube&logoColor=white)](https://www.youtube.com/watch?v=3QsLBMW8xB0&list=PL0qnWNTSPM4pfnHflUxCHcshOIlXDSNWp)
  
  <br align="center">
  
  [![openlidarmap](doc/openlidarmap_seq00.gif)](https://www.youtube.com/watch?v=3QsLBMW8xB0&list=PL0qnWNTSPM4pfnHflUxCHcshOIlXDSNWp)
  <br>

</div>

## Concept

We use reference maps in combination with classical LiDAR odometry to enable drift-free localization/mapping. Our approach is designed for high precision mapping. It enables georeferenced LiDAR-only point cloud mapping without GNSS. A detailed description of our pipeline can be found in the linked paper.

<img src=doc/pipeline_diagram.png alt="diagram" width="480" />

## Usage

<details>
<summary>Install</summary>

We provide a Docker image on Docker Hub, which will automatically be pulled within the Run section, but you also have the option to build it locally.  
```sh
./docker/build_docker.sh # (optional)
```
</details>

<details>
<summary>Run</summary>

To use our approach, you need a reference map and an initial guess of the first pose.  
More details on reference maps can be found in our paper.

The easiest way to use our approach is with the provided Docker image.  
We currently support point cloud files in `.bin`(KITTI), `.pcd.bin`(nuScenes), `.pcd`, `.ply` and `.xyz`.
```sh
./docker/run_docker.sh <map_path> <scan_path> <output_path> <x> <y> <z> <qx> <qy> <qz> <qw>

# Example
./docker/run_docker.sh /datasets/kitti/map.pcd /datasets/data_odometry_velodyne/dataset/sequences/00/velodyne /output/directory 395.5 1696.25 117.55 0 0 0.4848096 0.8746197
```

The output of the algorithm are poses in the KITTI format.
  
We also provide Python bindings. Have a look in the `python` folder, where we provide a test script.

</details>
<details>
<summary>Configure</summary>

The configuration of this pipeline can be changed in the `cpp/config` files. The naming suggest the intended usecase for the files. The most important parameters to play with if your results are not as good as expected are:

| Parameter | Description | Default | Note |
| :-------- | :-------- | :--------: | :-------- |
| pipeline_.visualize | Toggle GUI | `true` | use `false` on headless servers |
| pipeline_.save_submaps | Toggle submap saving | `false` | use to directly save high-resolution PCD submaps |
| preprocess_.downsampling_resolution | Scans are voxelized before usage | `1.5` | Reduce the size for increased robustness |
| preprocess_.num_neighbors | Points for covariance calculation | `10` | Try both directions |
| registration_.voxel_resolution | Voxelhashmap voxel size | `1.0` | Reduce the size for increased robustness | 
| registration_.lambda | Optimization dampening factor | `1.0` | Increase to increase the robustness |


</details>
<details>
<summary>Develop</summary>

We also provida a Development image, if you like to contribute or adapt or approach.  
Open this repository in VSCode -> F1 -> Rebuild and Reopen in Container.  

To build the C++ code:
```sh
mkdir build
cd build
cmake ../cpp && make -j
```

To build the Python bindings:
```sh
cd python
pip install -e .
```

</details>

## Data

The reference maps and original map outputs (v0.0.1) used for the paper can be downloaded from the following link: https://doi.org/10.14459/2025mp1771733

## Prior Map Generation

To find out how to generate reference maps, please follow this link: https://github.com/ga58lar/LoD2PCD

## Acknowledgement

Great inspiration has come from the following repositories. If you use our work, please also leave a star in their repositories and cite their work.

* [KISS-ICP](https://github.com/PRBonn/kiss-icp)
* [small_gicp](https://github.com/koide3/small_gicp)
* [Iridescence](https://github.com/koide3/iridescence)
* [Open3D](https://github.com/isl-org/Open3D)


## Citation

```bibtex
@conference{kulmer2025openlidarmap,
  author={Kulmer, Dominik and Leitenstern, Maximilian and Weinmann, Marcel and Lienkamp, Markus},
  title={OpenLiDARMap: Zero-Drift Point Cloud Mapping Using Map Priors},
  booktitle={Proceedings of the 11th International Conference on Vehicle Technology and Intelligent Transport Systems - VEHITS},
  year={2025},
  pages={178-188},
  publisher={SciTePress},
  organization={INSTICC},
  doi={10.5220/0013405400003941},
  isbn={978-989-758-745-0},
  issn={2184-495X},
}
```
