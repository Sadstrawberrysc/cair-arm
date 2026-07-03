#!/bin/bash
#
# ================= SLURM 指令 =================
#SBATCH -J carotid-base-train                 # 作业名
#SBATCH --partition=a100                      # 分区
#SBATCH --nodes=1                             # 节点数
#SBATCH --gres=gpu:2                          # 每节点申请 GPU 数（与 --nproc_per_node 一致）
#SBATCH -o /home/jiuan_chen/workspace/carotid_base_log/slurm_%j.out   # 标准输出
#SBATCH -e /home/jiuan_chen/workspace/carotid_base_log/slurm_%j.err   # 标准错误


MASTER_PORT=$(python3 - << 'EOF'
import socket
s = socket.socket()
s.bind(('', 0))
print(s.getsockname()[1])
s.close()
EOF
)

export MASTER_PORT
export CUDA_VISIBLE_DEVICES=0,1            # 如果你只想用 2 张卡，注意与 --gres 匹配

source /home/jiuan_chen/.venv/bin/activate

LOG_DIR=/home/jiuan_chen/workspace/trainlogs/carotid_base_log0730
mkdir -p "${LOG_DIR}"

# ============== 启动分布式训练 ==============
torchrun \
  --nproc_per_node=2 \
  --master_port=${MASTER_PORT} \
  /home/jiuan_chen/workspace/codes/mae-main/main.py \
    --batch_size 256 \
    --epochs 600 \
    --blr 1e-3 \
    --warmup_epochs 40 \
    --data_path /home/jiuan_chen/workspace/Mydatasets/data_carotid_0714 \
    --output_dir ${LOG_DIR} \
    --log_dir ${LOG_DIR} \
    --model mae_vit_base_patch16 \
    --resume /home/jiuan_chen/workspace/pretrainweights/mae_pretrain_vit_base.pth \
| tee "${LOG_DIR}/train_${SLURM_JOB_ID}.log"
