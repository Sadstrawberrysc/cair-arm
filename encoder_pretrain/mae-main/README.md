# Masked Autoencoders（MAE）

[Masked Autoencoders Are Scalable Vision Learners](https://arxiv.org/abs/2111.06377) 的 PyTorch / GPU 实现，
保留上游预训练、微调和可视化代码，供本仓库编码器实验使用，不属于在线控制链路。
原实现为 TensorFlow / TPU，本实现基于 [DeiT](https://github.com/facebookresearch/deit)。

## 使用入口

- [预训练](PRETRAIN.md)、[分类微调 / 线性探测](FINETUNE.md)
- [Colab 可视化](https://colab.research.google.com/github/facebookresearch/mae/blob/main/demo/mae_visualize.ipynb)、[本地 notebook](demo/mae_visualize.ipynb)
- 上游历史基线 `timm==0.3.2`；PyTorch 1.8.1+ 的兼容修复见[上游说明](https://github.com/rwightman/pytorch-image-models/issues/420#issuecomment-776459842)。不要据此覆盖仓库其他环境。

## 上游预训练权重

| 模型 | 权重 | 原记录 md5 前缀 |
| --- | --- | --- |
| ViT-Base | [下载](https://dl.fbaipublicfiles.com/mae/pretrain/mae_pretrain_vit_base.pth) | `8cad7c` |
| ViT-Large | [下载](https://dl.fbaipublicfiles.com/mae/pretrain/mae_pretrain_vit_large.pth) | `b8b06e` |
| ViT-Huge | [下载](https://dl.fbaipublicfiles.com/mae/pretrain/mae_pretrain_vit_huge.pth) | `9bdbb0` |

前缀仅保留上游记录，不是完整文件校验值；权重不纳入 Git。

## 上游论文参考结果

下表为原文档参考值，不代表本仓库重新训练或本机验证结果。
前三个 ImageNet 鲁棒性测试使用 ImageNet-1K 微调后的同一权重；后五项为迁移微调。

| 数据集 / 指标 | ViT-B | ViT-L | ViT-H | ViT-H 448 | 当时最佳对照 |
| --- | ---: | ---: | ---: | ---: | ---: |
| ImageNet-1K | 83.6 | 85.9 | 86.9 | 87.8 | 87.1 |
| ImageNet-Corruption，错误率 | 51.7 | 41.8 | 33.8 | 36.8 | 42.5 |
| ImageNet-Adversarial | 35.9 | 57.1 | 68.2 | 76.7 | 35.8 |
| ImageNet-Rendition | 48.3 | 59.9 | 64.4 | 66.5 | 48.7 |
| ImageNet-Sketch | 34.5 | 45.3 | 49.6 | 50.9 | 36.0 |
| iNaturalist 2017 | 70.5 | 75.7 | 79.3 | 83.4 | 75.4 |
| iNaturalist 2018 | 75.4 | 80.1 | 83.0 | 86.8 | 81.2 |
| iNaturalist 2019 | 80.5 | 83.4 | 85.7 | 88.3 | 84.1 |
| Places205 | 63.9 | 65.8 | 65.9 | 66.8 | 66.0 |
| Places365 | 57.9 | 59.4 | 59.8 | 60.3 | 58.0 |

## 引用与许可

```bibtex
@Article{MaskedAutoencoders2021,
  author = {Kaiming He and Xinlei Chen and Saining Xie and Yanghao Li and Piotr Doll{\'a}r and Ross Girshick},
  journal = {arXiv:2111.06377},
  title = {Masked Autoencoders Are Scalable Vision Learners},
  year = {2021},
}
```

本目录上游采用 CC-BY-NC 4.0，见 [LICENSE](LICENSE)。贡献规则与行为准则分别见
[CONTRIBUTING.md](CONTRIBUTING.md) 和 [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md)。
