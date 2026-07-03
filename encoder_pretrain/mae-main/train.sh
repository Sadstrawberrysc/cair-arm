MASTER_PORT=$(python3 - << 'EOF'
import socket
s = socket.socket(); s.bind(('', 0))
print(s.getsockname()[1])
s.close()
EOF
)

export CUDA_VISIBLE_DEVICES=0,1
# export OMP_NUM_THREADS=16

# torchrun \
#   --nproc_per_node=2 \
#   --master_port=${MASTER_PORT} \
#   main.py \
#   --batch_size 256 \
#   --epochs 800 \
#   --blr 1e-3 \
#   --warmup_epochs 40 \
#   --data_path /home/jiuan_chen/workspace/data_carotid \
#   --output_dir /home/jiuan_chen/workspace/carotid_base_log \
#   --log_dir /home/jiuan_chen/workspace/carotid_base_log \
#   --model mae_vit_base_patch16 \
#   --resume /home/jiuan_chen/workspace/pretrainweights/mae_pretrain_vit_base.pth

torchrun \
  --nproc_per_node=2 \
  --master_port=${MASTER_PORT} \
  main.py \
  --batch_size 256 \
  --epochs 800 \
  --blr 1e-3 \
  --warmup_epochs 40 \
  --data_path /home/mingcong/project/embodied/US_standerview/Pretrain/data_carotid_0714/ \
  --output_dir /home/mingcong/project/embodied/US_standerview/Pretrain/encoder_trainlog/ \
  --log_dir /home/mingcong/project/embodied/US_standerview/Pretrain/encoder_trainlog/ \
  --model mae_vit_base_patch16 \
  --resume /home//home/mingcong/project/embodied/US_standerview/Pretrain/encoder_trainlog/pretrainweights/mae_pretrain_vit_base.pth
