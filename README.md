# 3D Dempster-Shafer SLAM

This repository contains an implementation of the 3D Dempster-Shafer Simultaneous Localization and Mapping (SLAM) algorithm. It extends the [vinySLAM](https://github.com/OSLL/slam-constructor) approach to three-dimensional space.


## Installation

1. Clone repository

    ```bash
   git clone https://github.com/mishok2503/3D-Dempster-Shafer-SLAM.git
   cd 3D-Dempster-Shafer-SLAM 
   ```

2. Install Open3d

    ```bash
    pip install open3d
    ```

   
## Run

```bash
    python main.py <file-with-data.json>
```
[Input file format](https://github.com/mishok2503/slam-3d-datasets-generator#output-format)
   

## Test

```bash
    python main.py tests/data/result.json
```
