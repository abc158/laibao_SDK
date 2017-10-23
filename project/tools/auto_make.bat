del app.afw

iarbuild ..\app.ewp -clean Debug
iarbuild ..\app.ewp -make Debug -log info

copy ..\SDK_FW\SdkFirmware.sfw .\
copy ..\Debug\Exe\app.s19 .\

AmFirmwareBuilder.exe app.s19 SdkFirmware.sfw app.afw

del SdkFirmware.sfw
del app.s19

::AmFirmwareDownloader.exe ./app.afw