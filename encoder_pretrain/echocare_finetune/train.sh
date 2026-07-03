datadir=/home/jiuan_chen/workspace/Mydatasets/data_carotid_0714/train
logdir=/home/jiuan_chen/workspace/trainlogs/HY0922_swinbase
mkdir -p $logdir

torchrun --nproc_per_node 2 --master_port=28802 hiera_train.py \
    --logdir $logdir \
    --datadir $datadir


