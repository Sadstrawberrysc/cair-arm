# MAE 分类微调与线性探测

以下是上游 ImageNet 示例与参考结果，本次未重跑训练。在本目录运行；预训练权重见 [README](README.md#上游预训练权重)。

## Evaluation

As a sanity check, run evaluation using our ImageNet **fine-tuned** models:

| Model | Fine-tuned checkpoint | Original md5 prefix | ImageNet Acc@1 |
| --- | --- | --- | ---: |
| ViT-Base | [download](https://dl.fbaipublicfiles.com/mae/finetune/mae_finetuned_vit_base.pth) | `1b25e9` | 83.664 |
| ViT-Large | [download](https://dl.fbaipublicfiles.com/mae/finetune/mae_finetuned_vit_large.pth) | `51f550` | 85.952 |
| ViT-Huge | [download](https://dl.fbaipublicfiles.com/mae/finetune/mae_finetuned_vit_huge.pth) | `2541f2` | 86.928 |

MD5 prefixes reproduce the upstream notes, not complete file checksums.

Evaluate ViT-Base in a single GPU (`${IMAGENET_DIR}` is a directory containing `{train, val}` sets of ImageNet):
```bash
python main_finetune.py --eval --resume mae_finetuned_vit_base.pth --model vit_base_patch16 --batch_size 16 --data_path ${IMAGENET_DIR}
```
This should give:
```text
* Acc@1 83.664 Acc@5 96.530 loss 0.731
```

Evaluate ViT-Large:
```bash
python main_finetune.py --eval --resume mae_finetuned_vit_large.pth --model vit_large_patch16 --batch_size 16 --data_path ${IMAGENET_DIR}
```
This should give:
```text
* Acc@1 85.952 Acc@5 97.570 loss 0.646
```

Evaluate ViT-Huge:
```bash
python main_finetune.py --eval --resume mae_finetuned_vit_huge.pth --model vit_huge_patch14 --batch_size 16 --data_path ${IMAGENET_DIR}
```
This should give:
```text
* Acc@1 86.928 Acc@5 98.088 loss 0.584
```

## Fine-tuning

Get our pre-trained checkpoints from [here](README.md#上游预训练权重).

To fine-tune with **multi-node distributed training**, run the following on 4 nodes with 8 GPUs each:
```bash
python submitit_finetune.py \
    --job_dir ${JOB_DIR} \
    --nodes 4 \
    --batch_size 32 \
    --model vit_base_patch16 \
    --finetune ${PRETRAIN_CHKPT} \
    --epochs 100 \
    --blr 5e-4 --layer_decay 0.65 \
    --weight_decay 0.05 --drop_path 0.1 --reprob 0.25 --mixup 0.8 --cutmix 1.0 \
    --dist_eval --data_path ${IMAGENET_DIR}
```
- Install submitit (`pip install submitit`) first.
- Here the effective batch size is 32 (`batch_size` per gpu) * 4 (`nodes`) * 8 (gpus per node) = 1024.
- `blr` is the base learning rate. The actual `lr` is computed by the [linear scaling rule](https://arxiv.org/abs/1706.02677): `lr` = `blr` * effective batch size / 256.
- We have run 4 trials with different random seeds. The results are 83.63, 83.66, 83.52, 83.46 (mean 83.57 and std 0.08).
- Training time is ~7h11m in 32 V100 GPUs.

Script for ViT-Large:
```bash
python submitit_finetune.py \
    --job_dir ${JOB_DIR} \
    --nodes 4 --use_volta32 \
    --batch_size 32 \
    --model vit_large_patch16 \
    --finetune ${PRETRAIN_CHKPT} \
    --epochs 50 \
    --blr 1e-3 --layer_decay 0.75 \
    --weight_decay 0.05 --drop_path 0.2 --reprob 0.25 --mixup 0.8 --cutmix 1.0 \
    --dist_eval --data_path ${IMAGENET_DIR}
```
- We have run 4 trials with different random seeds. The results are 85.95, 85.87, 85.76, 85.88 (mean 85.87 and std 0.07).
- Training time is ~8h52m in 32 V100 GPUs.

Script for ViT-Huge:
```bash
python submitit_finetune.py \
    --job_dir ${JOB_DIR} \
    --nodes 8 --use_volta32 \
    --batch_size 16 \
    --model vit_huge_patch14 \
    --finetune ${PRETRAIN_CHKPT} \
    --epochs 50 \
    --blr 1e-3 --layer_decay 0.75 \
    --weight_decay 0.05 --drop_path 0.3 --reprob 0.25 --mixup 0.8 --cutmix 1.0 \
    --dist_eval --data_path ${IMAGENET_DIR}
```
- Training time is ~13h9m in 64 V100 GPUs.

To fine-tune our pre-trained ViT-Base with **single-node training**, run the following on 1 node with 8 GPUs:
```bash
OMP_NUM_THREADS=1 python -m torch.distributed.launch --nproc_per_node=8 main_finetune.py \
    --accum_iter 4 \
    --batch_size 32 \
    --model vit_base_patch16 \
    --finetune ${PRETRAIN_CHKPT} \
    --epochs 100 \
    --blr 5e-4 --layer_decay 0.65 \
    --weight_decay 0.05 --drop_path 0.1 --mixup 0.8 --cutmix 1.0 --reprob 0.25 \
    --dist_eval --data_path ${IMAGENET_DIR}
```
- Here the effective batch size is 32 (`batch_size` per gpu) * 4 (`accum_iter`) * 8 (gpus) = 1024. `--accum_iter 4` simulates 4 nodes.

### Notes

- The [pre-trained models we provide](README.md#上游预训练权重) are trained with *normalized* pixels `--norm_pix_loss` (1600 epochs, Table 3 in paper). The fine-tuning hyper-parameters are slightly different from the default baseline using *unnormalized* pixels.

- The original MAE implementation was in TensorFlow+TPU with no explicit mixed precision. This re-implementation is in PyTorch+GPU with automatic mixed precision (`torch.cuda.amp`). We have observed different numerical behavior between the two platforms. In this repo, we use `--global_pool` for fine-tuning; using `--cls_token` performs similarly, but there is a chance of producing NaN when fine-tuning ViT-Huge in GPUs. We did not observe this issue in TPUs. Turning off amp could solve this issue, but is slower.

- Here we use RandErase following DeiT: `--reprob 0.25`. Its effect is smaller than random variance.

## Linear Probing

Run the following on 4 nodes with 8 GPUs each:
```bash
python submitit_linprobe.py \
    --job_dir ${JOB_DIR} \
    --nodes 4 \
    --batch_size 512 \
    --model vit_base_patch16 --cls_token \
    --finetune ${PRETRAIN_CHKPT} \
    --epochs 90 \
    --blr 0.1 \
    --weight_decay 0.0 \
    --dist_eval --data_path ${IMAGENET_DIR}
```
- Here the effective batch size is 512 (`batch_size` per gpu) * 4 (`nodes`) * 8 (gpus per node) = 16384.
- `blr` is the base learning rate. The actual `lr` is computed by the [linear scaling rule](https://arxiv.org/abs/1706.02677): `lr` = `blr` * effective batch size / 256.
- Training time is ~2h20m for 90 epochs in 32 V100 GPUs.
- To run single-node training, follow the instruction in fine-tuning.

To train ViT-Large or ViT-Huge, set `--model vit_large_patch16` or `--model vit_huge_patch14`. It is sufficient to train 50 epochs `--epochs 50`.

This PT/GPU code produces *better* results for ViT-L/H (see the table below). This is likely caused by the system difference between TF and PT.

| Implementation | ViT-Base | ViT-Large | ViT-Huge |
| --- | ---: | ---: | ---: |
| Paper, TF / TPU | 68.0 | 75.8 | 76.6 |
| Upstream PyTorch / GPU | 67.8 | 76.0 | 77.2 |
