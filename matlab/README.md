# Chạy nhanh mô phỏng Ver0

Copy từng khối vào **Command Window của MATLAB**. Mô phỏng chạy xong sẽ mở viewer; không cần mở Simulink để nhấn Run. Lệnh dùng tham số đang lưu trong model, không lưu thay đổi vào file `.slx`.

## Line

```matlab
% Run these lines with MATLAB's current folder set to matlab/
model = 'Spider_4leg_line_Ver0';
load_system(fullfile(pwd,[model '.slx']));
out = sim(model);
test_dang_Ver0(out);
```

## Rotate — Right / Left

```matlab
% Run these lines with MATLAB's current folder set to matlab/
model = 'Spider_4leg_rotate_Ver0';
load_system(fullfile(pwd,[model '.slx']));
sw = '0'; % '0': Right (CW); '1': Left (CCW)
in = Simulink.SimulationInput(model);
in = in.setBlockParameter([model '/Direction'],'sw',sw);
out = sim(in);
test_dang_Ver0(out);
```

## Circle — CW / CCW

```matlab
% Run these lines with MATLAB's current folder set to matlab/
model = 'Spider_4leg_circular_Ver0';
load_system(fullfile(pwd,[model '.slx']));
sw = '0'; % '0': CW; '1': CCW
in = Simulink.SimulationInput(model);
in = in.setBlockParameter([model '/Direction'],'sw',sw);
out = sim(in);
test_dang_Ver0(out);
```

## Lissajous — CW / CCW

```matlab
% Run these lines with MATLAB's current folder set to matlab/
model = 'Spider_4leg_lissajous_Ver0';
load_system(fullfile(pwd,[model '.slx']));
sw = '0'; % '0': CW; '1': CCW
in = Simulink.SimulationInput(model);
in = in.setBlockParameter([model '/Direction'],'sw',sw);
out = sim(in);
test_dang_Ver0(out);
```
