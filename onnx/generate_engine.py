import tensorrt as trt
import os

# Paths
onnx_path = "DA3METRIC-LARGE/DA3METRIC-LARGE.onnx"
engine_path = "DA3METRIC-LARGE/DA3METRIC-LARGE_v1.engine"

# 1. Initialize Logger and Builder
logger = trt.Logger(trt.Logger.VERBOSE) # Use VERBOSE to see the exact library version
builder = trt.Builder(logger)

# 2. Create Network and Parser
# EXPLICIT_BATCH is required for ONNX models
network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
parser = trt.OnnxParser(network, logger)

# 3. Create Config
config = builder.create_builder_config()
config.set_flag(trt.BuilderFlag.FP16)
# Standard workspace limit (e.g., 2GB)
config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 2 * 1024 * 1024 * 1024)

# 4. Parse ONNX
print(f"Parsing ONNX from: {onnx_path}")
with open(onnx_path, "rb") as model:
    if not parser.parse(model.read()):
        for error in range(parser.num_errors):
            print(f"ONNX Error: {parser.get_error(error)}")
        exit(1)

# 5. Build and Serialize
print("Building Engine... (This will take a few minutes)")
serialized_engine = builder.build_serialized_network(network, config)

if serialized_engine is None:
    print("Build failed!")
    exit(1)

with open(engine_path, "wb") as f:
    f.write(serialized_engine)

print(f"Successfully built engine: {engine_path}")