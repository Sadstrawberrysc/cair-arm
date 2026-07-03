import torch
import os
import argparse

def extract_head_weights(input_path, output_path):
    if not os.path.exists(input_path):
        print(f"Error: Input file '{input_path}' not found.")
        return

    print(f"Loading full checkpoint from: {input_path}")
    # 加载完整模型
    # map_location='cpu' 确保即使你在没有 GPU 的机器上也能运行此脚本
    checkpoint = torch.load(input_path, map_location='cpu')

    # 1. 获取完整的 state_dict
    # 根据你之前的训练代码，保存时使用的是 {'model': model.state_dict(), ...}
    if 'model' in checkpoint:
        full_state_dict = checkpoint['model']
    else:
        # 兼容性处理：如果直接保存的是 state_dict
        full_state_dict = checkpoint
    
    # 2. 筛选并提取 Head 的权重
    # 你的模型中 head 的变量名是 self.head，所以 state_dict 中的键都以 "head." 开头
    head_state_dict = {}
    extracted_keys = []
    
    print("Extracting head weights...")
    for key, value in full_state_dict.items():
        if key.startswith('head.'):
            # 去掉前缀 "head."，这样加载时可以直接用 model.head.load_state_dict()
            new_key = key.replace('head.', '')
            head_state_dict[new_key] = value
            extracted_keys.append(new_key)

    if not head_state_dict:
        print("Error: No head weights found! Please check if the key prefix is 'head.'.")
        return

    print(f"Found {len(head_state_dict)} keys corresponding to the head:")
    # print(extracted_keys) # 如果想看具体提取了哪些层，可以取消注释

    # 3. 构造新的保存字典
    # 保持和你之后想用的加载格式一致
    save_dict = {
        'head': head_state_dict,
        #以此保留一些元数据供参考（可选）
        'epoch': checkpoint.get('epoch', -1),
        'args': checkpoint.get('args', None),
        'description': 'Extracted head weights only'
    }

    # 4. 保存
    torch.save(save_dict, output_path)
    print(f"Success! Head-only model saved to: {output_path}")
    print(f"File size reduced to approximately: {os.path.getsize(output_path) / 1024:.2f} KB")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract head weights from full model checkpoint')
    # 输入：之前训练好的完整模型路径
    parser.add_argument('--input', type=str, required=True, help='Path to the full model checkpoint (.pth)')
    # 输出：你想保存的新路径
    parser.add_argument('--output', type=str, default='head_only.pth', help='Path to save the head weights')
    
    args = parser.parse_args()
    
    extract_head_weights(args.input, args.output)