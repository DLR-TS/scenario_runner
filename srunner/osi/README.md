Protobuffer interface description files for generating the implementation files should be in the subdirectories.

Install grpc by `pip install grpcio grpcio-tools`
Generate python files from within this directory with
E.g.:
```
# osi3 is installed in ../../osi3/
# run from this directory:
 python -m grpc_tools.protoc  -I ./protos/osi3/ -I ./protos/ --python_out ./client/ --grpc_python_out ./client/ ./protos/ScenarioRunner.proto
# or from Scenario Runner repository root:
 python -m grpc_tools.protoc  -I ./srunner/osi/protos/osi3/ -I ./srunner/osi/protos/ --python_out ./srunner/osi/client/ --grpc_python_out ./srunner/osi/client/ ./srunner/osi/protos/ScenarioRunner.proto
```
