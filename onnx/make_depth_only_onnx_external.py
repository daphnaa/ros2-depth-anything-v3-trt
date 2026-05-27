import os
import sys
import shutil
from pathlib import Path

import onnx

src = Path(sys.argv[1]).resolve()
dst = Path(sys.argv[2]).resolve()
keep_name = sys.argv[3] if len(sys.argv) > 3 else "depth"

src_data = Path(str(src) + ".data")
dst_data = Path(str(dst) + ".data")

if not src.exists():
    raise FileNotFoundError(src)

if not src_data.exists():
    raise FileNotFoundError(
        f"Expected external data file next to ONNX:\n  {src_data}"
    )

# Important: do NOT load external data. The current metadata points to the wrong file.
model = onnx.load(str(src), load_external_data=False)

# Keep only the requested output.
outputs = list(model.graph.output)
matches = [o for o in outputs if keep_name.lower() in o.name.lower()]

if not matches:
    print("Available outputs:")
    for o in outputs:
        print(" ", o.name)
    raise RuntimeError(f"No graph output matching {keep_name!r}")

model.graph.ClearField("output")
model.graph.output.extend([matches[0]])

# Rewrite every external-data tensor location to the new .data filename.
new_location = dst_data.name
num_external = 0

for tensor in model.graph.initializer:
    if tensor.data_location == onnx.TensorProto.EXTERNAL:
        num_external += 1
        found_location = False

        for kv in tensor.external_data:
            if kv.key == "location":
                kv.value = new_location
                found_location = True
                break

        if not found_location:
            entry = tensor.external_data.add()
            entry.key = "location"
            entry.value = new_location

# Save the small ONNX protobuf. This does not need to load/copy the 1.3GB weights.
onnx.save(model, str(dst))

# Create external data file for the new ONNX.
# Symlink is fastest and avoids duplicating 1.3GB. If symlink fails, copy.
if dst_data.exists() or dst_data.is_symlink():
    dst_data.unlink()

try:
    os.symlink(src_data.name, dst_data)
    link_type = "symlink"
except OSError:
    shutil.copy2(src_data, dst_data)
    link_type = "copy"

print(f"Saved: {dst}")
print(f"External tensors rewritten: {num_external}")
print(f"External data {link_type}: {dst_data} -> {src_data.name}")
print(f"Kept output: {matches[0].name}")
