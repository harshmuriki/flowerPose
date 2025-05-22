# FlowerPose: Pose Extraction for Plants

Robotic Phenotyping for Small-Scale Urban Farms

> ***Note:*** Code and documentation for usage will be updated soon.

---

## Table of Contents
- [Overview](#overview)
- [Project Structure](#project-structure)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Directory Overview](#directory-overview)
- [Contributing](#contributing)
- [License](#license)

---

## Overview
FlowerPose is a toolkit for plant pose estimation and analysis using computer vision and deep learning. It provides utilities for video processing, dataset creation, pose estimation, and deep learning model development, with a focus on agricultural robotics and plant phenotyping.

---

## Project Structure
```
.
├── deeplearning/
├── farmbot/
├── pose estimation/
├── Final Results/
├── FinalDatasets/
├── thresholding/
├── requirements.txt
├── README.md
└── LICENSE
```

---

## Features
- **Deep Learning:** Jupyter notebooks and scripts for training and evaluating models on plant data (`deeplearning/`).
- **Pose Estimation:** Tools for estimating plant pose from point clouds and images (`pose estimation/`).
- **Visualization:** Plotting and analysis of results.

---

## Installation
1. **Clone the repository:**
   ```sh
   git clone https://github.com/yourusername/flowerPose.git
   cd flowerPose
   ```
2. **Install dependencies:**
   ```sh
   pip install -r requirements.txt
   ```
3. ** Install Jupyter for notebooks:**
   ```sh
   pip install jupyter
   ```

---

## Usage
<!-- - **Video Processing:**
  Use functions in `farmbot/utils.py` to extract frames or merge videos.
  ```python
  from farmbot.utils import convert_to_photos, combine_videos
  convert_to_photos('input.mp4', 'output_folder')
  ``` -->
- **Deep Learning:**
  Explore and run the notebooks in `deeplearning/` for model training and evaluation.
- **Pose Estimation:**
  Run `pose estimation/Pose_Estimation_All.py` or use the notebooks for point cloud analysis.

---

## Directory Overview
- `deeplearning/`: Deep learning scripts, notebooks, and data.
- `farmbot/`: Video processing utilities and FarmBot integration.
- `pose estimation/`: Scripts and notebooks for pose estimation.
- `Final Results/`: Output and result files.
- `FinalDatasets/`: Processed datasets.
- `thresholding/`: Image thresholding utilities.

---

## Contact
For questions or feedback, please contact the project maintainer:
- **Name:** Harsh Muriki
- **Email:** vmuriki3@gatech.edu

---

## Contributing
Contributions are welcome! Please open issues or submit pull requests for improvements or bug fixes.

---

## License
This project is licensed under the terms of the `LICENSE` file.

---
