# Airsim
https://github.com/Microsoft/AirSim

## Windows 安装
https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md  
VS2017 (win SDK 8.1)+ UE 4.18  
1. 出现 typedef 错误, 更改C:\Program Files(x86)\Windows Kits\8.1\Include\um\ShlObj.h
1054行 删除typedef
2. path(airsim) build.cmd 137 138行注释(禁止Debug编译)
3. 全局搜索中文引号 改成英文