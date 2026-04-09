■安装包说明

  1. 适用满足以下条件的32位/64位 x86平台的Linux操作系统：
     a. glibc 2.12及以上版本
     b. Linux内核版本号从2.6.32（含）到5.11.0（含）

■安装SDK前的注意事项

  1. 为提高GigE相机的取图稳定性，SDK提供GigE内核驱动程序。
     此内核驱动程序会在安装SDK时编译生成并自动安装，编译内核驱动程序时需要用到此内核版本所对应的内核头文件，
     请务必确保存在内核头文件相对应的目录，不然安装SDK会提示以下错误
     "The kernel header(/lib/modules/内核版本号/build) corresponding to the kernel version is missing."

■安装时注意事项

  1. 使用root权限进行安装/卸载操作
  2. 运行SDK安装run文件
  3. 安装途中，对提示的信息，建议都选择【yes】，方便安装完程序以后无需执行其他应用程序自带脚本而直接可以使用；
  4. 安装完了以后，在最下面会提示安装的路径为/opt/HuarayTech/MVviewer, 目录结构如下:
     MVviewer
       ├ bin		: GUI工具，发现、连接相机，拉流取图，设置相机属性，卸载脚本，etc
       ├ include	: SDK头文件、转码头文件
       ├ lib		: SDK动态库、第三方库、转码库
       ├ module
       │  ├ GigEDriver	: GigE相机驱动程序
       │  └ USBDriver	: USB相机的驱动程序
       └ share
           ├ C	: C例程     
           ├ Qt	: Qt例程
           └ Python	: Python例程 

■运行脚本注意事项

  1. 启动相机客户端程序（MVviewer），请执行/opt/HuarayTech/MVviewer/bin目录下的run.sh脚本。
  2. 插入usb相机后请等相机指示灯变蓝后再连接相机，以免设备文件没有生成而导致连接失败。
  3. 如果在usb相机已插入的情况下安装*.run文件的话，安装完*.run文件后请插拔相机。
  4. 插入u3v相机以后但无法发现相机的情况下，请用“lsusb”确认系统有无识别相机，无识别请再次插拔相机。
  5. 在连接GigE相机前，请确保相机与网卡处于同一网段上。

■特别说明

  1. 因新版本的libpng对png图片的检查变得更加严格，程序运行时会出现以下日志但不会影响程序运行，
     "libpng warning: iCCP: know incorrect sRGB profile"

  2. 由于GigE驱动程序是根据当前内核版本在安装SDK过程中生成的，因此如果安装SDK完成后更新了内核版本，
     驱动模块程序将无法在新内核中被加载成功，需要重新安装SDK使驱动程序重现生成后才能加载。

-END-

