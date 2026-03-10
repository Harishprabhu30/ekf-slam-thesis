'''
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
'''

#!/usr/bin/env python3
"""
GPU cleanup utility for Isaac Sim + ROS2
"""

import gc
import torch

def clear_gpu():
    gc.collect()

    if torch.cuda.is_available():
        torch.cuda.synchronize()
        torch.cuda.empty_cache()
        torch.cuda.ipc_collect()

        print("GPU cache cleared successfully")
        print("Allocated:", torch.cuda.memory_allocated()/1024**2, "MB")
        print("Reserved:", torch.cuda.memory_reserved()/1024**2, "MB")
    else:
        print("CUDA not available")

if __name__ == "__main__":
    clear_gpu()
