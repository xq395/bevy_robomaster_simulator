"""Regenerate rm_custom_client_pb2.py from rm_custom_client.proto.

Needs grpcio-tools (build-time only; the runtime needs just `protobuf`):
    python -m pip install "grpcio-tools==1.75.1"
    python tools/rmu_protocol/generate.py
"""
from pathlib import Path
import sys

from grpc_tools import protoc

HERE = Path(__file__).resolve().parent

code = protoc.main([
    "protoc", f"-I{HERE}", f"--python_out={HERE}", str(HERE / "rm_custom_client.proto"),
])
if code:
    sys.exit(code)
print("generated", HERE / "rm_custom_client_pb2.py")
