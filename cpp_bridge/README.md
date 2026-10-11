# deploy_dev C++ Talos bridge (Windows)

This bridge keeps the simulator and `D:\HNU_RM\代码栈\deploy_dev` on Windows.
Python is not part of the real-time path.

## Runtime design

- A capture thread copies each Talos RGB image and its coherent pose bundle.
- All Talos image/pose triple buffers are released immediately after the copy.
- RGB-to-BGR conversion happens after release.
- A single latest-frame slot overwrites stale frames when inference is slower
  than the simulator.
- Detection, PnP, tracker/EKF, aimer, MPC, fire gate, and UDP telemetry use the
  C++ implementation from `deploy_dev`.
- Commands are disabled unless `-Send` is supplied. The simulator must also
  have auto aim enabled with F5. `-AllowFire` explicitly enables the FireGate
  even if YAML disables it. Actual shots additionally require runtime operator
  authorization (hold Space/RB or a live local operator-control lease).
  See `../docs/operator_phase1.md`. Bridge `fire` telemetry is advice, not proof of a shot.

## Build

```powershell
& F:\bevy_robomaster_simulator\tools\configure_deploy_dev_cpp.ps1 `
  -BuildTarget talos_sp_bridge,talos_async_source_test
```

## Safe run

For the Web workflow, leave CPU affinity unrestricted so OpenVINO and the JPEG
worker do not contend inside the same small core set. The launcher defaults to
unrestricted affinity; priority can still be lowered explicitly when required.

```powershell
& F:\bevy_robomaster_simulator\tools\run_talos_cpp_bridge.ps1
```

## Windows Web debug (recommended)

The Web path replaces PlotJuggler for this simulator workflow. It publishes
raw, annotated, gray, and binary views plus detector/PnP/EKF/MPC/fire-gate
state. JPEG encoding runs on a separate latest-only thread and cannot
backpressure inference.

Start the simulator first, then run:

```powershell
& F:\bevy_robomaster_simulator\tools\run_talos_web_debug.ps1
```

Open `http://127.0.0.1:5000/`. The launcher explicitly passes
`--no-telemetry`, so UDP port 9870 and PlotJuggler are not used. Command output
and fire remain disabled unless their explicit switches are supplied.

Use a specific OpenVINO IR without editing the read-only `deploy_dev` tree:

```powershell
& F:\bevy_robomaster_simulator\tools\run_talos_cpp_bridge.ps1 `
  -ModelPath F:\bevy_robomaster_simulator\.deps\models\praysky_coord_noe2e_0331_640x640_fp16.xml
```

`-Send` enables gimbal command output. Do not add `-AllowFire` until detector,
PnP, tracker, latency, and fire-gate telemetry have been validated.

## Model optimization

The original `.onnx` already runs through OpenVINO CPU. Exporting IR mainly
removes ONNX parsing at startup, while FP16 constant compression can also lower
memory traffic. The generated FP16-weight IR is the current safe performance
candidate:

`F:\bevy_robomaster_simulator\.deps\models\praysky_coord_noe2e_0331_640x640_fp16.xml`

Post-training INT8 quantization is available as an offline tool:

```powershell
& C:\Users\34741\AppData\Local\Programs\Python\Python311\python.exe `
  F:\bevy_robomaster_simulator\tools\quantize_yolo26_openvino.py `
  --model D:\HNU_RM\代码栈\deploy_dev\assets\models\hebei_at_nn\praysky_coord_noe2e_0331_640x640.onnx `
  --output F:\bevy_robomaster_simulator\.deps\models\yolo26_int8_candidate.xml `
  --shm-dir F:\talos_shm --frames 128 --interval-ms 100 --preset mixed
```

For a real candidate, collect diverse frames with both colors, large/small
armor, different ranges, angles, motion blur, partial occlusion, and negative
backgrounds. The `*_int8_smoke.xml` artifact was calibrated on only eight
current simulator frames and is deliberately not match-ready.

## Verification

```powershell
$env:PATH = 'F:\bevy_robomaster_simulator\.deps\install\opencv\build\x64\vc16\bin;' + $env:PATH
ctest --test-dir F:\bevy_robomaster_simulator\.deps\build\deploy_dev-win64 `
  -C Release -R '^talos_async_source$' --output-on-failure
```

The test checks RGB/BGR conversion, immediate image/pose release, metadata
coherence, and gimbal-command packing with fire disabled.