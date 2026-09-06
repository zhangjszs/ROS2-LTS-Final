# Vision weights

YOLO 权重（`*.pt` / `*.onnx`）不进 git，体积约 250 MB，且主 launch **默认不启用**视觉。

本机若已有文件，会留在这个目录，只是不再被跟踪。需要视觉时把权重放到：

```
models/exp26l_gpu/best.onnx
models/yolo26l_48g/weights/best.pt
models/yolo26l_48g/weights/best.onnx
```

对应 yaml 在 `../config/`。打开视觉还需要相机-LiDAR 标定，见根目录 README。
