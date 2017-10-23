1.双击build_afw.bat 生成 app.afw
2.用工具AmFirmwareDownloader.exe将app.afw下载到机器（需要先烧好对应的bootloader）
  支持3种烧写模式：
	A.FullFirmware 	-- 完整固件 
	B.App&Config 	-- 用户代码 和 配置
	C.OnlyConfig 	-- 仅仅下载配置部分