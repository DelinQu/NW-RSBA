# NW-RSBA

[CVPR 2023] Revisiting Rolling Shutter Bundle Adjustment: Toward Accurate and Fast Solution.

<img src='docs/images/intro.png' width=1000>
We propose an accurate and fast bundle adjustment (BA) solution that estimates the 6-DoF pose with an independent RS model of the camera and the geometry of the environment based on measurements from a rolling shutter (RS) camera. This tackles the challenges in the existing works, namely, relying on high frame rate video as input, restrictive assumptions on camera motion and poor efficiency. To this end, we first verify the positive influence of the image point normalization to RSBA. Then we present a novel visual residual covariance model to standardize the reprojection error during RSBA, which consequently improves the overall accuracy. Besides, we demonstrate the combination of <b>N</b>ormalization and covariance standardization <b>W</b>eighting in *RSBA* (**NW-RSBA**) can avoid common planar degeneracy without the need to constrain the filming manner. Finally, we propose an acceleration strategy for **NW-RSBA** based on the sparsity of its Jacobian matrix and Schur complement. The extensive synthetic and real data experiments verify the effectiveness and efficiency of the proposed solution over the state-of-the-art works.

- [Project website](https://delinqu.github.io/NW-RSBA/)

- [Paper](https://openaccess.thecvf.com/content/CVPR2023/papers/Liao_Revisiting_Rolling_Shutter_Bundle_Adjustment_Toward_Accurate_and_Fast_Solution_CVPR_2023_paper.pdf)

## Usage
Download our code, and open rssfm/rssfm_main.mlx, replace the root_path variable with your own path.

## Option
We provide two scene 1. general scene 2. degeneracy scene with six rolling shutter bundle adjustment methods.


## Citation
If you find this code useful for your research, please consider citing the following paper.
```bibtex
  @InProceedings{Liao_2023_CVPR,
      author    = {Liao, Bangyan and Qu, Delin and Xue, Yifei and Zhang, Huiqing and Lao, Yizhen},
      title     = {Revisiting Rolling Shutter Bundle Adjustment: Toward Accurate and Fast Solution},
      booktitle = {Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)},
      month     = {June},
      year      = {2023}
  }
```

## Acknowledgment
This work is supported by the National Key R\&D Program of China (No. 2022ZD0119003), Nature Science Foundation of China (No. 62102145), and Jiangxi Provincial 03 Special Foundation and 5G Program (Grant No. 20224ABC03A05).
