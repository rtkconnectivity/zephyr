### 工程编译

该工程默认编译 `bank0 Image`。如果需要编译 `bank1 Image`，请将 `rtl8762gn_evb.overlay` 中的 `zephyr,code-partition` 修改为 `&app_partition_1`。

### 脚本运行

为了使得 Zephyr Image 能够兼容 Realtek 的 OTA 系统，在编译完后，请运行 `tools/run.bat`。该脚本会将生成的 `zephyr.bin` 加上 Realtek 的 MP header，并将sha256校验值、payload length 等写入 image header 中。

最终将生成两版 image，分别是 `zephyrImage_MP.bin` 和 `zephyrImage.bin`（不带 MP header）。

### 如何在蓝牙工程中打开RTK OTA？

要开启OTA功能，需要在开启蓝牙的app中打开CONFIG_REALTEK_OTA。

另外如果需要对Zephyr Image进行OTA升级，请拷贝本工程路径下的tools目录到您的应用程序路径下，并使用tools/run.bat脚本对原始Zephyr Image进行处理。

OTA相关的source/header file放置在 `zephyr\soc\arm\realtek_bee\rtl87x2g\ota_src\` 目录下。