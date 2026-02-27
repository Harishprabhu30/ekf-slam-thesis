#!/usr/bin/env python3
"""
Clear PyTorch GPU cache in Isaac Sim
"""

import torch

if torch.cuda.is_available():
    torch.cuda.empty_cache()
    print("GPU cache cleared.")
else:
    print("No CUDA device detected.")

