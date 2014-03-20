atomisp_camera
==============

Intel atomisp camera test source

# atomisp2_test.c 修改说明：
1.增加命令行参数解析，可以通过脚步实现自动化测试
2.解决预览分辨率大于录像分辨率时，分辨率设置失败的问题

# 编译说明：
cp atomisp2_test.c ~/GBS-ROOT/local/BUILD-ROOTS/scratch.i586.0
cd ~/GBS-ROOT/local/BUILD-ROOTS/scratch.i586.0
sudo chroot .
gcc -lpthread atomisp2_test.c -o atomisp2_test
sdb push atomisp2_test /data/

# atomisp2_test命令行参数说明：
-c, --capture ：选择拍照测试
-r, --record  ：录像测试
-p, --preview ：预览测试
-d, --dump    ：录像时或预览测试时存储YUV数据，默认只有拍照会存储YUV数据
拍照时yuv文件命名格式为:摄像头类型_分辨率宽度_分辨率高度.yuv,如1280*720的前置摄象头拍照，则存储数据为secondCamera_1280x720.yuv
录像时yuv文件命名格式为：摄像头类型_video_record_分辨率宽度_分辨率高度.yuv,如1920*1080的后置摄像头录像，则存储数据为primaryCamera_video_record_1920x1080.yuv
-i, --camera_id： 用于选择摄像头，0：为后置摄像头，1：为前置摄像头
-w, --width    ：用于设置测试分辨率宽度
-h, --height   ：用于设置测试分辨率高度

# 测试示例：
1）前置摄像头拍照，分辨率为：1280*720
./atomisp2_test -i 1 -c -w 1280 -h 960
2)后置摄像头拍照，分辨率为：1920*1080
./atomisp2_test -i 0 -c  -w 1920 -h 1080
3)后置摄像头录像，不存储yuv数据，分辨率为：1920*1080
./atomisp2_test -i 0 -r -w 1920 -h 1080
4)前置摄像头录像，存储yuv数据，分辨率为：720*480
./atomisp2_test -i 1 -r -d -w 720  -h 480
