import os, json, random
from pathlib import Path

import cv2
cv2.setNumThreads(0)

import numpy as np
from PIL import Image

import torch
import torch.nn.functional as F
from torch.utils.data import Dataset
import torchvision.transforms as T


IMG_EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff")


def set_randomness(seed: int = 42):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)


def _elastic_deform_numpy(img_uint8, alpha=2.0, sigma=8.0):

    if alpha <= 0 or sigma <= 0:
        return img_uint8

    h, w = img_uint8.shape[:2]
    dx = np.random.rand(h, w).astype(np.float32) * 2 - 1
    dy = np.random.rand(h, w).astype(np.float32) * 2 - 1
    dx = cv2.GaussianBlur(dx, (0, 0), sigma) * alpha
    dy = cv2.GaussianBlur(dy, (0, 0), sigma) * alpha

    x, y = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    map_x = (x + dx).astype(np.float32)
    map_y = (y + dy).astype(np.float32)

    warped = cv2.remap(img_uint8, map_x, map_y, interpolation=cv2.INTER_LINEAR,
                       borderMode=cv2.BORDER_REFLECT_101)
    return warped


class ClsFolderDataset(Dataset):

    def __init__(
        self,
        root_dir,
        split="train",
        class_names=None,            
        transform: T.Compose | None = None,
        target_size=(256, 256),       
        augment=False,
        # aug
        p_flip=0.5,
        p_crop=0.0,
        p_elastic=0.10,
        p_noise=0.50,
        crop_scale_rangeH=(0.9, 1.0),
        crop_scale_rangeW=(0.85, 1.0),
        elastic_alpha_px=(3.0, 8.0),
        elastic_sigma_px=(3.0, 6.0),
        noise_std=(0.01, 0.03),       
        save_classmap_json=False,
    ):
        super().__init__()
        self.root = Path(root_dir)
        self.split = split
        self.transform = transform
        self.target_size = tuple(target_size) if target_size is not None else None
        self.augment = bool(augment) and (split == "train")

        self.aug_cfg = dict(
            p_flip=float(p_flip),
            p_crop=float(p_crop),
            p_elastic=float(p_elastic),
            p_noise=float(p_noise),
            crop_scale_rangeH=tuple(crop_scale_rangeH),
            crop_scale_rangeW=tuple(crop_scale_rangeW),
            elastic_alpha_px=tuple(elastic_alpha_px),
            elastic_sigma_px=tuple(elastic_sigma_px),
            noise_std=tuple(noise_std),
        )

        self.class_names, self.class_to_idx = self._discover_classes(class_names)
        self.samples = self._discover_samples()

        if save_classmap_json:
            mp = {"class_to_idx": self.class_to_idx, "class_names": self.class_names}
            (self.root / f"classmap_{self.split}.json").write_text(json.dumps(mp, indent=2, ensure_ascii=False))

        if len(self.samples) == 0:
            raise RuntimeError(f"[ClsFolderDatasetV2] No images found under '{root_dir}' (split='{split}')")

    def _discover_classes(self, class_names):
        if class_names is not None:
            class_names = list(class_names)
            return class_names, {c: i for i, c in enumerate(class_names)}

        split_dir = self.root / self.split
        if split_dir.is_dir():
            classes = sorted([d.name for d in split_dir.iterdir() if d.is_dir()])
            if classes:
                return classes, {c: i for i, c in enumerate(classes)}

        classes = []
        for d in sorted(self.root.iterdir()):
            if d.is_dir() and (d / self.split).is_dir():
                classes.append(d.name)
        if classes:
            return classes, {c: i for i, c in enumerate(classes)}

        raise RuntimeError(f"[ClsFolderDatasetV2] Cannot infer classes. Expect one of:\n"
                           f" A) {self.root}/class/{self.split}/*.jpg\n"
                           f" B) {self.root}/{self.split}/class/*.jpg")

    def _discover_samples(self):
        samples = []

        split_dir = self.root / self.split
        if split_dir.is_dir():
            for cname in self.class_names:
                cdir = split_dir / cname
                if not cdir.is_dir():
                    continue
                for fn in os.listdir(cdir):
                    if fn.lower().endswith(IMG_EXTS):
                        samples.append((str(cdir / fn), self.class_to_idx[cname]))

        if not samples:
            for cname in self.class_names:
                cdir = self.root / cname / self.split
                if not cdir.is_dir():
                    continue
                for fn in os.listdir(cdir):
                    if fn.lower().endswith(IMG_EXTS):
                        samples.append((str(cdir / fn), self.class_to_idx[cname]))

        return samples

    @staticmethod
    def _to_tensor_uint8(rgb_u8: np.ndarray) -> torch.Tensor:
        return torch.from_numpy(rgb_u8).permute(2, 0, 1).float() / 255.0  

    def _augment_tensor(self, img: torch.Tensor) -> torch.Tensor:

        assert img.ndim == 3 and img.size(0) in (1, 3), "expect [C,H,W]"
        C, H, W = img.shape
        cfg = self.aug_cfg

        if random.random() < cfg["p_flip"]:
            img = torch.flip(img, dims=[-1])

        if random.random() < cfg["p_crop"]:
            sH = random.uniform(*cfg["crop_scale_rangeH"])
            sW = random.uniform(*cfg["crop_scale_rangeW"])
            crop_h = max(1, int(round(H * sH)))
            crop_w = max(1, int(round(W * sW)))
            y0 = 0 if H == crop_h else random.randint(0, H - crop_h)
            x0 = 0 if W == crop_w else random.randint(0, W - crop_w)
            img = img[:, y0:y0 + crop_h, x0:x0 + crop_w]

        if random.random() < cfg["p_elastic"]:
            u8 = (img.clamp(0, 1).permute(1, 2, 0).cpu().numpy() * 255.0).astype(np.uint8) 
            a = random.uniform(*cfg["elastic_alpha_px"])
            s = random.uniform(*cfg["elastic_sigma_px"])
            u8 = _elastic_deform_numpy(u8, alpha=a, sigma=s)
            img = self._to_tensor_uint8(u8)

        if self.target_size is not None and img.shape[-2:] != self.target_size:
            img = F.interpolate(img.unsqueeze(0), size=self.target_size, mode="bilinear", align_corners=False).squeeze(0)

        if random.random() < cfg["p_noise"]:
            lo, hi = cfg["noise_std"]
            std = random.uniform(lo, hi)
            noise = torch.randn(1, *img.shape[-2:], dtype=img.dtype, device=img.device) * std
            if img.size(0) == 3:
                noise = noise.expand(1, *img.shape[-2:])  
            img = (img + noise).clamp(0.0, 1.0)

        return img

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        fpath, label = self.samples[idx]
        with Image.open(fpath) as im:
            img_pil = im.convert("RGB")     

        img = self._to_tensor_uint8(np.array(img_pil, dtype=np.uint8, copy=True))
        if self.augment:
            img = self._augment_tensor(img)
            if self.transform is not None:
                u8 = (img.clamp(0,1).permute(1,2,0).cpu().numpy() * 255.0).astype(np.uint8)
                img = self.transform(Image.fromarray(u8, mode="RGB"))
        else:
            if self.transform is not None:
                img = self.transform(img_pil)
            else:
                if self.target_size is not None and img.shape[-2:] != self.target_size:
                    img = F.interpolate(img.unsqueeze(0), size=self.target_size, mode="bilinear", align_corners=False).squeeze(0)

        return img, label
