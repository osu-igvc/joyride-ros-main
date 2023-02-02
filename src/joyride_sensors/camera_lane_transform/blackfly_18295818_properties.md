Point Grey Research Blackfly BFLY-PGE-31S4C
===========================================

*Version: FW:v1.61.3.00 FPGA:v2.02*

Attributes
----------

`AcquisitionFrameCount` : `int`  
  Number of frames to acquire in multi-frame acquisition mode.
  - default access: not available
  - default range: 1 - 65535

`AcquisitionFrameRate` : `float`  
  Controls the acquisition rate (in Hertz) at which the frames are captured.
  - default access: read only
  - default value: `93.11065673828125`
  - unit: Hz
  - default range: 1.0 - 93.11065673828125

`AcquisitionFrameRateAuto` : `enum`  
  Controls the mode for automatic frame rate adjustment.
  - default access: read/write
  - default value: `'Continuous'`
  - possible values: `'Off'`, `'Once'`, `'Continuous'`

`AcquisitionFrameRateEnabled` : `bool`  
  Enables manual control of the camera frame rate.
  - default access: read/write
  - default value: `True`

`AcquisitionMode` : `enum`  
  Sets the acquisition mode of the device.
  - default access: read/write
  - default value: `'Continuous'`
  - possible values: `'Continuous'`, `'SingleFrame'`, `'MultiFrame'`

`AcquisitionStatus` : `bool`  
  Reads the state of the internal acquisition signal selected using AcquisitionStatusSelector.
  - default access: read only
  - default value: `False`

`AcquisitionStatusSelector` : `enum`  
  Selects the internal acquisition signal to read using AcquisitionStatus.
  - default access: read/write
  - default value: `'FrameTriggerWait'`
  - possible values: `'FrameTriggerWait'`

`ActivePageNumber` : `int`  
  Control the number of the active data flash page.
  - default access: read/write
  - default value: `0`
  - default range: 0 - 2047

`ActivePageOffset` : `int`  
  Control the offset of the coefficient to access in the active data flash page.
  - default access: read/write
  - default value: `0`
  - default range: 0 - 255

`ActivePageValue` : `int`  
  Returns the value at entry ActivePageOffset of the active data flash page.
  - default access: read/write
  - default value: `1197098821`
  - default range: 0 - 4294967295

`AutoExposureTimeLowerLimit` : `float`  
  Lower limit of the Auto Exposure (us) parameter
  - default access: read/write
  - default value: `13.172626495361328`
  - unit: us
  - default range: 0.0 - 10592.639446258545

`AutoExposureTimeUpperLimit` : `float`  
  Upper limit of the Auto Exposure (us) parameter
  - default access: read/write
  - default value: `10592.639446258545`
  - unit: us
  - default range: 13.172626495361328 - 108280.42030334473

`AutoFunctionAOIHeight` : `int`  
  Height of the auto function area of interest in pixels.
  - default access: not available
  - default range: 0 - 768

`AutoFunctionAOIOffsetX` : `int`  
  Vertical offset from the origin to the auto function area of interest in pixels.
  - default access: not available
  - default range: 0 - 1024

`AutoFunctionAOIOffsetY` : `int`  
  Horizontal offset from the origin to the auto function area of interest in pixels.
  - default access: not available
  - default range: 0 - 768

`AutoFunctionAOIWidth` : `int`  
  Width of the auto function area of interest in pixels.
  - default access: not available
  - default range: 0 - 1024

`AutoFunctionAOIsControl` : `enum`  
  ON or OFF for the feature of the Auto Function AOIs.
  - default access: read/write
  - default value: `'Off'`
  - possible values: `'Off'`, `'On'`

`AutoGainLowerLimit` : `float`  
  Lower limit of the Auto Gain (dB) parameter
  - default access: read/write
  - default value: `0.0`
  - unit: dB
  - default range: 0.0 - 47.994266510009766

`AutoGainUpperLimit` : `float`  
  Upper limit of the Auto Gain (dB) parameter
  - default access: read/write
  - default value: `47.994266510009766`
  - unit: dB
  - default range: 0.0 - 47.994266510009766

`BalanceRatio` : `float`  
  This value sets the selected balance ratio control as an integer.
  - default access: read only
  - default value: `1.120452880859375`
  - default range: 0.25 - 4.0

`BalanceRatioSelector` : `enum`  
  Selects a balance ratio to configure. Once a balance ratio control has been selected.
  - default access: read/write
  - default value: `'Red'`
  - possible values:
    - `'Red'`: This enumeration value selects the red balance ratio control for adjustment.
    - `'Green'`: This enumeration value selects the green balance ratio control for adjustment.
    - `'Blue'`: This enumeration value selects the blue balance ratio control for adjustment.

`BalanceWhiteAuto` : `enum`  
  Balance White Auto is the 'automatic' counterpart of the manual white balance feature.
  - default access: read/write
  - default value: `'Continuous'`
  - possible values:
    - `'Off'`: Disables the Balance White Auto function.
    - `'Once'`: Sets operation mode to 'once'.
    - `'Continuous'`: Sets operation mode to 'continuous'.

`BinningControl` : `enum`  
  Switchs between average binning and additive binning.
  - default access: not available
  - possible values: `'Additive'`, `'Average'`

`BinningHorizontal` : `int`  
  Number of horizontal pixels to combine together.
  - default access: read only
  - default value: `1`
  - default range: 1 - 2

`BinningHorizontalLocked` : `int`  
  
  - default access: read/write
  - default value: `1`
  - default range: -9223372036854775808 - 9223372036854775807

`BinningVertical` : `int`  
  Number of vertical pixels to combine together.
  - default access: read/write
  - default value: `1`
  - default range: 1 - 2

`BlackLevel` : `float`  
  Analog black level in percent.
  - default access: read/write
  - default value: `5.859375`
  - unit: %
  - default range: 0.0 - 12.4755859375

`BlackLevelClampingEnable` : `bool`  
  Enable the black level auto clamping feature which performing dark current compensation
  - default access: read/write
  - default value: `True`

`CamRegBaseAddress` : `int`  
  
  - default access: read/write
  - default value: `4042260480`
  - default range: -9223372036854775808 - 9223372036854775807

`ChunkBlackLevel` : `float`  
  Returns the black level used to capture the image.
  - default access: not available
  - default range: -3.4028234663852886e+38 - 3.4028234663852886e+38

`ChunkCRC` : `int`  
  Returns the CRC of the image payload.
  - default access: not available
  - default range: 0 - 4294967295

`ChunkEnable` : `bool`  
  Enables the inclusion of the selected Chunk data in the payload of the image.
  - default access: read/write
  - default value: `False`

`ChunkExposureTime` : `float`  
  Returns the exposure time used to capture the image.
  - default access: not available
  - default range: -1.7976931348623157e+308 - 1.7976931348623157e+308

`ChunkFrameCounter` : `int`  
  Returns the image count.
  - default access: not available
  - default range: 0 - 4294967295

`ChunkGain` : `float`  
  Returns the gain used to capture the image.
  - default access: not available
  - default range: -3.4028234663852886e+38 - 3.4028234663852886e+38

`ChunkHeight` : `int`  
  Returns the height of the image.
  - default access: not available
  - default range: 0 - 4294967295

`ChunkModeActive` : `bool`  
  Activates the inclusion of Chunk data in the payload of the image
  - default access: read/write
  - default value: `False`

`ChunkOffsetX` : `int`  
  Returns the Offset X of the image included in the payload.
  - default access: not available
  - default range: 0 - 4294967295

`ChunkOffsetY` : `int`  
  Returns the Offset Y of the image included in the payload.
  - default access: not available
  - default range: 0 - 4294967295

`ChunkPixelDynamicRangeMax` : `int`  
  Returns the Maximum range of the pixel
  - default access: not available
  - default range: 0 - 4294967295

`ChunkPixelDynamicRangeMin` : `int`  
  Returns the Minimum range of the pixel
  - default access: not available
  - default range: 0 - 4294967295

`ChunkPixelFormat` : `enum`  
  This enumeration lists the pixel formats that can be indicated by the pixel format chunk.
  - default access: not available
  - possible values:
    - `'Mono8'`: This enumeration value indicates that the pixel data in the acquired image is in the Mono 8 format.
    - `'Mono10'`: This enumeration value indicates that the pixel data in the acquired image is in the Mono 10 format.
    - `'Mono12'`: This enumeration value indicates that the pixel data in the acquired image is in the Mono 12 format.
    - `'Mono12Packed'`: This enumeration value indicates that the pixel data in the acquired image is in the Mono 12 Packed format.
    - `'BayerGR8'`: This enumeration value indicates that the pixel data in the acquired image is in the Bayer GR 8 format.
    - `'BayerRG8'`: This enumeration value indicates that the pixel data in the acquired image is in the Bayer RG 8 format.
    - `'BayerGB8'`: This enumeration value indicates that the pixel data in the acquired image is in the Bayer GB 8 format.
    - `'BayerBG8'`: This enumeration value indicates that the pixel data in the acquired image is in the Bayer BG 8 format.
    - `'BayerGR12'`: This enumeration value indicates that the pixel data in the acquired image is in the Bayer GR 12 format.
    - `'BayerRG12'`: This enumeration value indicates that the pixel data in the acquired image is in the Bayer RG 12 format.
    - `'BayerGB12'`: This enumeration value indicates that the pixel data in the acquired image is in the Bayer GB 12 format.
    - `'BayerBG12'`: This enumeration value indicates that the pixel data in the acquired image is in the Bayer GB 12 format.
    - `'YUV422Packed'`: This enumeration value indicates that the pixel data in the acquired image is in the YUV 422 Packed format.
    - `'Packed'`: This enumeration value indicates that the pixel data in the acquired image is in the YUV 422 (YUYV) Packed format.
    - `'BayerGB12Packed'`: This enumeration value indicates that the pixel data in the acquired image is in the Bayer GB 12 Packed  format.
    - `'BayerGR12Packed'`: This enumeration value indicates that the pixel data in the acquired image is in the Bayer GR 12 Packed format.
    - `'BayerRG12Packed'`: This enumeration value indicates that the pixel data in the acquired image is in the Bayer RG 12 Packed format.
    - `'BayerBG12Packed'`: This enumeration value indicates that the pixel data in the acquired image is in the Bayer BG 12 Packed format.

`ChunkSelector` : `enum`  
  Selects which chunk data to enable or control
  - default access: read/write
  - default value: `'FrameCounter'`
  - possible values: `'Image'`, `'CRC'`, `'FrameCounter'`, `'OffsetX'`, `'OffsetY'`, `'Width'`, `'Height'`, `'ExposureTime'`, `'Gain'`, `'BlackLevel'`, `'PixelFormat'`, `'DynamicPixelRangeMin'`, `'DynamicPixelRangeMax'`, `'TransmitFrameCount'`, `'Timestamp'`

`ChunkTimestamp` : `int`  
  Returns the Timestamp of the image.
  - default access: not available
  - default range: 0 - 9223372036854775807

`ChunkWidth` : `int`  
  Returns the width of the image.
  - default access: not available
  - default range: 0 - 4294967295

`DataFlashBaseAddress` : `int`  
  
  - default access: read only
  - default value: `4043309056`
  - default range: -9223372036854775808 - 9223372036854775807

`DataFlashPageCount` : `int`  
  Number of the data flash pages.
  - default access: read only
  - default value: `2048`
  - default range: -9223372036854775808 - 9223372036854775807

`DataFlashPageSize` : `int`  
  Size of the data flash page.
  - default access: read only
  - default value: `256`
  - default range: -9223372036854775808 - 9223372036854775807

`DecimationHorizontal` : `int`  
  Horizontal sub-sampling of the image. This reduces the horizontal resolution (width) of the image by the specified horizontal decimation factor. A value of 1 indicates that the camera performs no horizontal decimation.
  - default access: read only
  - default value: `2`
  - default range: 1 - 2

`DecimationHorizontalLocked` : `int`  
  
  - default access: read/write
  - default value: `1`
  - default range: -9223372036854775808 - 9223372036854775807

`DecimationVertical` : `int`  
  Vertical sub-sampling of the image. This reduces the vertical resolution (height) of the image by the specified vertical decimation factor. A value of 1 indicates that the camera performs no vertical decimation.
  - default access: read/write
  - default value: `2`
  - default range: 1 - 2

`DeviceFirmwareVersion` : `string`  
  Version of the firmware in the device.
  - default access: read only
  - default value: `'1.61.3.00'`

`DeviceID` : `string`  
  Device identifier (serial number).
  - default access: read only
  - default value: `'18295818'`

`DeviceIndicatorMode` : `enum`  
  Controls the LED behaviour
  - default access: read/write
  - default value: `'Active'`
  - possible values: `'Inactive'`, `'Active'`

`DeviceLinkReserve` : `int`  
  Allocated the percentage of bandwidth reserved for asynchronous communication.
  - default access: read/write
  - default value: `10`
  - default range: 0 - 90

`DeviceLinkThroughputLimit` : `int`  
  Limits the maximum bandwidth of data that will be streamed out by the device.
  - default access: read/write
  - default value: `98952000`
  - default range: 8312000 - 125000000

`DeviceMaxThroughput` : `int`  
  Maximum bandwidth of the data  that can be streamed out of the device.
  - default access: read only
  - default value: `89296000`
  - default range: 800000 - 89296000

`DeviceModelName` : `string`  
  Model name of the device.
  - default access: read only
  - default value: `'Blackfly BFLY-PGE-31S4C'`

`DeviceSVNVersion` : `string`  
  SVN Version of the device.
  - default access: read only
  - default value: `'FW:292525 FPGA:288795'`

`DeviceScanType` : `enum`  
  Scan type of the sensor.
  - default access: read only
  - default value: `'Areascan'`
  - possible values: `'Areascan'`

`DeviceSerialNumber` : `string`  
  Device serial number. This string is a unique identifier of the device.
  - default access: read only
  - default value: `'18295818'`

`DeviceTemperature` : `float`  
  Device temperature in degrees Celcius (C).
  - default access: read only
  - default value: `56.85000000000002`
  - unit: °C
  - default range: -1.7976931348623157e+308 - 1.7976931348623157e+308

`DeviceUserID` : `string`  
  User Defined Name.
  - default access: read/write
  - default value: `''`

`DeviceVendorName` : `string`  
  Name of the manufacturer of the device.
  - default access: read only
  - default value: `'Point Grey Research'`

`DeviceVersion` : `string`  
  Version of the device.
  - default access: read only
  - default value: `'FW:v1.61.3.00 FPGA:v2.02'`

`EventAcquisitionEnd` : `int`  
  Returns the unique identifier of the AcquisitionEnd type of Event.
  - default access: not available
  - default range: 0 - 65535

`EventAcquisitionEndFrameID` : `int`  
  Returns the unique Identifier of the Frame (or image) that generated the AcquisitionEnd Event.
  - default access: not available
  - default range: 0 - 9223372036854775807

`EventAcquisitionEndTimestamp` : `int`  
  Returns the Timestamp of the AcquisitionEnd Event.
  - default access: not available
  - default range: 0 - 9223372036854775807

`EventAcquisitionStart` : `int`  
  Returns the unique identifier of the AcquisitionStart type of Event.
  - default access: not available
  - default range: 0 - 65535

`EventAcquisitionStartFrameID` : `int`  
  Returns the unique Identifier of the Frame (or image) that generated the AcquisitionStart Event.
  - default access: not available
  - default range: 0 - 9223372036854775807

`EventAcquisitionStartTimestamp` : `int`  
  Returns the Timestamp of the AcquisitionStart Event.
  - default access: not available
  - default range: 0 - 9223372036854775807

`EventExposureEnd` : `int`  
  Returns the unique identifier of the ExposureEnd type of Event.
  - default access: not available
  - default range: 0 - 65535

`EventExposureEndFrameID` : `int`  
  Returns the unique Identifier of the Frame (or image) that generated the ExposureEnd Event.
  - default access: not available
  - default range: 0 - 9223372036854775807

`EventExposureEndTimestamp` : `int`  
  Returns the Timestamp of the ExposureEnd Event.
  - default access: not available
  - default range: 0 - 9223372036854775807

`EventNotification` : `enum`  
  Enables/Disables event.
  - default access: read/write
  - default value: `'Off'`
  - possible values: `'On'`, `'Off'`

`EventSelector` : `enum`  
  Selects which Event to enable or control
  - default access: read/write
  - default value: `'AcquisitionStart'`
  - possible values: `'AcquisitionStart'`, `'AcquisitionEnd'`, `'ExposureStart'`, `'ExposureEnd'`

`ExposureAuto` : `enum`  
  Sets the automatic exposure mode when Exposure Mode is Timed.
  - default access: read/write
  - default value: `'Continuous'`
  - possible values:
    - `'Off'`: Exposure set to manual control.
    - `'Once'`: Exposure is automatically adjusted once, then returns to Off.
    - `'Continuous'`: Exposure is automatically adjusted by the camera

`ExposureMode` : `enum`  
  Sets the operation mode of the Exposure (or shutter).
  - default access: read/write
  - default value: `'Timed'`
  - possible values: `'Timed'`, `'TriggerWidth'`

`ExposureTime` : `float`  
  Exposure time in microseconds when Exposure Mode is Timed and ExposureAuto is Off.
  - default access: read only
  - default value: `10592.639446258545`
  - unit: us
  - default range: 13.172626495361328 - 10592.639446258545

`ExposureTimeAbs` : `float`  
  Exposure time in microseconds when Exposure Mode is Timed and ExposureAuto is Off.
  - default access: read only
  - default value: `10592.639446258545`
  - unit: us
  - default range: 13.172626495361328 - 10592.639446258545

`Fmt7RegBaseAddress` : `int`  
  
  - default access: read only
  - default value: `4042263040`
  - default range: -9223372036854775808 - 9223372036854775807

`GPIOCtrlPinRegBaseAddress` : `int`  
  
  - default access: read only
  - default value: `4042264848`
  - default range: -9223372036854775808 - 9223372036854775807

`Gain` : `float`  
  Gain applied to the image in dB.
  - default access: read only
  - default value: `29.696563720703125`
  - unit: dB
  - default range: 0.0 - 47.994266510009766

`GainAuto` : `enum`  
  Sets the automatic gain control (AGC) mode.
  - default access: read/write
  - default value: `'Continuous'`
  - possible values:
    - `'Off'`: Gain is User controlled using Gain.
    - `'Once'`: Gain is automatically adjusted once by the device. Once it has converged, it automatically returns to the Off state.
    - `'Continuous'`: Gain is constantly adjusted by the device.

`GainSelector` : `enum`  
  Selects which Gain is controlled by the various Gain features.
  - default access: read only
  - default value: `'All'`
  - possible values:
    - `'All'`: Gain will be applied to all channels or taps.

`Gamma` : `float`  
  Controls the gamma correction of pixel intensity.
  - default access: read only
  - default value: `1.0`
  - default range: 0.5 - 3.9990234375

`GammaEnabled` : `bool`  
  Enables/disables gamma correction.
  - default access: not available

`GevCCP` : `enum`  
  Controls the device access privilege of an application.
  - default access: read/write
  - default value: `'ControlAccess'`
  - possible values: `'OpenAccess'`, `'ExclusiveAccess'`, `'ControlAccess'`

`GevCurrentDefaultGateway` : `int`  
  Indicates the default gateway IP address to be used on the given network interface.
  - default access: read only
  - default value: `181116926`
  - default range: 0 - 4294967295

`GevCurrentIPAddress` : `int`  
  Current IP address for the given network interface.
  - default access: read only
  - default value: `181115997`
  - default range: 0 - 4294967295

`GevCurrentIPConfigurationDHCP` : `bool`  
  Indicates if DHCP IP configuration scheme is activated on the given network interface.
  - default access: read/write
  - default value: `True`

`GevCurrentIPConfigurationLLA` : `bool`  
  Indicates if Link Local Address IP configuration scheme is activated on the given network interface.
  - default access: read/write
  - default value: `True`

`GevCurrentIPConfigurationPersistentIP` : `bool`  
  Indicates if Persistent IP configuration scheme is activated on the given network interface.
  - default access: read/write
  - default value: `False`

`GevCurrentSubnetMask` : `int`  
  Current subnet mask of the given interface.
  - default access: read only
  - default value: `4294965248`
  - default range: 0 - 4294967295

`GevDeviceModeCharacterSet` : `enum`  
  Character set used by all the strings of the bootstrap registers.
  - default access: read only
  - default value: `'UTF8'`
  - possible values: `'UTF8'`

`GevDeviceModeIsBigEndian` : `bool`  
  Endianess of the device registers.
  - default access: read only
  - default value: `False`

`GevFirstURL` : `string`  
  The first choice of URL for the XML device description file.
  - default access: read only
  - default value: `'Local:GRS_GEV_v003_292525.zip;7F1D0040;8B46'`

`GevGVCPHeartbeatDisable` : `bool`  
  Disables the GVCP heartbeat.
  - default access: read/write
  - default value: `False`

`GevGVCPPendingAck` : `bool`  
  Enables the generation of PENDING_ACK.
  - default access: read/write
  - default value: `True`

`GevGVCPPendingTimeout` : `int`  
  Indicates the longest GVCP command execution time before the device returns a PENDING_ACK in milliseconds.
  - default access: read only
  - default value: `20`
  - default range: 0 - 4294967295

`GevHeartbeatTimeout` : `int`  
  Indicates the current heartbeat timeout in milliseconds.
  - default access: read/write
  - default value: `3000`
  - default range: 500 - 10000

`GevInterfaceSelector` : `int`  
  Selects which physical network interface to control.
  - default access: read only
  - default value: `0`
  - default range: 0 - 0

`GevLinkSpeed` : `int`  
  Indicates the speed of transmission negotiated by the given network interface in Mbps.
  - default access: read only
  - default value: `1000`
  - default range: 0 - 4294967295

`GevMACAddress` : `int`  
  MAC address of the network interface.
  - default access: read only
  - default value: `758549785610`
  - default range: -9223372036854775808 - 9223372036854775807

`GevMessageChannelCount` : `int`  
  Indicates the number of message channels supported by this device.
  - default access: read only
  - default value: `1`
  - default range: 0 - 4294967295

`GevNumberOfInterfaces` : `int`  
  Indicates the number of physical network interfaces supported by this device.
  - default access: read only
  - default value: `1`
  - default range: 0 - 4294967295

`GevPersistentDefaultGateway` : `int`  
  Indicates the persistent default gateway for this network interface. It is only used when the device boots with the Persistent IP configuration scheme.
  - default access: read/write
  - default value: `0`
  - default range: 0 - 4294967295

`GevPersistentIPAddress` : `int`  
  Indicates the persistent IP address for this network interface. It is only used when the device boots with the Persistent IP configuration scheme.
  - default access: read/write
  - default value: `0`
  - default range: 0 - 4294967295

`GevPersistentSubnetMask` : `int`  
  Indicates the persistent subnet mask associated with the persistent IP address on this network interface. It is only used when the device boots with the Persistent IP configuration scheme.
  - default access: read/write
  - default value: `0`
  - default range: 0 - 4294967295

`GevPrimaryApplicationIPAddress` : `int`  
  Indicates the address of the primary application.
  - default access: read only
  - default value: `181114924`
  - default range: 0 - 4294967295

`GevPrimaryApplicationSocket` : `int`  
  Indicates the UDP source port of the primary application.
  - default access: read only
  - default value: `48900`
  - default range: 0 - 4294967295

`GevSCDA` : `int`  
  Indicates the destination IP address for this stream channel.
  - default access: read/write
  - default value: `181114924`
  - default range: 0 - 4294967295

`GevSCPD` : `int`  
  Indicates the delay (in timestamp counter units) to insert between each packet for this stream channel. This can be used as a crude flow-control mechanism if the application or the network infrastructure cannot keep up with the packets coming from the device.
  - default access: read/write
  - default value: `400`
  - default range: 0 - 65535

`GevSCPDirection` : `enum`  
  Reports the direction of the stream channel.
  - default access: read only
  - default value: `'Transmitter'`
  - possible values:
    - `'Transmitter'`: Transmitter
    - `'Receiver'`: Receiver

`GevSCPHostPort` : `int`  
  Host port of the channel
  - default access: read/write
  - default value: `8881`
  - default range: 0 - 65535

`GevSCPInterfaceIndex` : `int`  
  Index of network interface to use.
  - default access: read/write
  - default value: `0`
  - default range: 0 - 0

`GevSCPSBigEndian` : `bool`  
  Endianess of multi-byte pixel data for this stream.
  - default access: read/write
  - default value: `False`

`GevSCPSDoNotFragment` : `bool`  
  The state of this feature is copied into the "do not fragment" bit of the IP header of each stream packet.
  - default access: read/write
  - default value: `False`

`GevSCPSFireTestPacket` : `bool`  
  Sends a test packet.
  - default access: read/write
  - default value: `False`

`GevSCPSPacketSize` : `int`  
  Specifies the stream packet size (in bytes) to send on this channel.
  - default access: read/write
  - default value: `1400`
  - default range: 220 - 9000

`GevSCSP` : `int`  
  Indicates the source UDP port address for this stream channel.
  - default access: read only
  - default value: `1047`
  - default range: 0 - 4294967295

`GevSecondURL` : `string`  
  The second choice of URL to the XML device description file.
  - default access: read only
  - default value: `'http://www.ptgrey.com/GRS_GEV_v003_292525.xml'`

`GevStreamChanRegOffset` : `int`  
  
  - default access: read only
  - default value: `3328`
  - default range: -9223372036854775808 - 9223372036854775807

`GevStreamChannelCount` : `int`  
  Indicates the number of stream channels supported by this device.
  - default access: read only
  - default value: `1`
  - default range: 0 - 4294967295

`GevStreamChannelSelector` : `int`  
  Selects the stream channel to control.
  - default access: read/write
  - default value: `0`
  - default range: 0 - 0

`GevSupportedOption` : `bool`  
  Indicates whether or not the selected GEV option is supported.
  - default access: read only
  - default value: `True`

`GevSupportedOptionSelector` : `enum`  
  Selects the GEV option to interrogate for existing support.
  - default access: read/write
  - default value: `'CommandsConcatenation'`
  - possible values: `'UserDefinedName'`, `'SerialNumber'`, `'HeartbeatDisable'`, `'LinkSpeed'`, `'CCPApplicationSocket'`, `'ManifestTable'`, `'TestData'`, `'DiscoveryAckDelay'`, `'DiscoveryAckDelayWritable'`, `'ExtendedStatusCodes'`, `'Action'`, `'PendingAck'`, `'EventData'`, `'Event'`, `'PacketResend'`, `'WriteMem'`, `'CommandsConcatenation'`, `'IPConfigurationLLA'`, `'IPConfigurationDHCP'`, `'IPConfigurationPersistentIP'`, `'StreamChannelSourceSocket'`

`GevTimestampTickFrequency` : `int`  
  Indicates the number of timestamp ticks in 1 second (frequency in Hz).
  - default access: read only
  - default value: `125000000`
  - default range: 0 - 4294967295

`GevTimestampTickFrequencyValue` : `int`  
  
  - default access: read only
  - default value: `125000000`
  - default range: -9223372036854775808 - 9223372036854775807

`GevTimestampValue` : `int`  
  This is a read only element. It indicates the latched value of the timestamp.  (The timestamp must first be latched using the Timestamp Control Latch command.)
  - default access: read only
  - default value: `0`
  - default range: -9223372036854775808 - 9223372036854775807

`GevVersionMajor` : `int`  
  Major version of the specification.
  - default access: read only
  - default value: `1`
  - default range: 0 - 65535

`GevVersionMinor` : `int`  
  Minor version of the specification.
  - default access: read only
  - default value: `2`
  - default range: 0 - 65535

`Height` : `int`  
  Height of the image provided by the device (in pixels).
  - default access: read/write
  - default value: `768`
  - default range: 2 - 768

`HeightMax` : `int`  
  Maximum height of the image (in pixels).
  - default access: read only
  - default value: `768`
  - default range: 0 - 65535

`Hue` : `float`  
  Hue of the image in degrees.
  - default access: read only
  - default value: `0.0`
  - unit: °
  - default range: -180.0 - 179.912109375

`HueEnabled` : `bool`  
  Enables/disables hue adjustment.
  - default access: not available

`LUTEnable` : `bool`  
  Activates the selected LUT.
  - default access: read/write
  - default value: `False`

`LUTIndex` : `int`  
  Control the index (offset) of the coefficient to access in the selected LUT.
  - default access: not available
  - default range: 0 - 511

`LUTRegBankBaseAddress` : `int`  
  
  - default access: read only
  - default value: `4042785792`
  - default range: -9223372036854775808 - 9223372036854775807

`LUTRegBaseAddress` : `int`  
  
  - default access: read only
  - default value: `4042784768`
  - default range: -9223372036854775808 - 9223372036854775807

`LUTRegChannelBaseAddress` : `int`  
  
  - default access: read only
  - default value: `4042785792`
  - default range: -9223372036854775808 - 9223372036854775807

`LUTSelector` : `enum`  
  This enumeration the lookup table (LUT) to configure. Once a LUT has been selected, all changes to the LUT settings will be applied to the selected LUT.
  - default access: read/write
  - default value: `'Luminance'`
  - possible values:
    - `'Luminance'`: Selects the Luminace LUT.
    - `'Red'`: Selects the Red LUT.
    - `'Green'`: Selects the Green LUT.
    - `'Blue'`: Selects the Blue LUT.

`LUTValue` : `int`  
  Returns the Value at entry LUTIndex of the LUT selected by LUTSelector.
  - default access: not available
  - default range: 0 - 511

`LineDebouncerTimeRaw` : `int`  
  Sets the raw value of the selected line debouncer time in microseconds.
  - default access: read/write
  - default value: `0`
  - default range: 0 - 1048575

`LineInverter` : `bool`  
  Controls the inversion of the signal of the selected input or output line.
  - default access: not available

`LineMode` : `enum`  
  Controls whether the physical Line is used to Input or Output a signal.
  - default access: read/write
  - default value: `'Input'`
  - possible values: `'Input'`, `'Output'`

`LineSelector` : `enum`  
  Selects the physical line (or pin) of the external device connector to configure.
  - default access: read/write
  - default value: `'Line0'`
  - possible values: `'Line0'`, `'Line1'`, `'Line2'`, `'Line3'`

`LineSource` : `enum`  
  Selects which internal acquisition or I/O source signal to output on the selected line.
  - default access: not available
  - possible values: `'ExposureActive'`, `'ExternalTriggerActive'`, `'UserOutput1'`, `'UserOutput2'`, `'UserOutput3'`

`LineStatus` : `bool`  
  Returns the current status of the selected input or output Line.
  - default access: read only
  - default value: `False`

`LineStatusAll` : `int`  
  Returns the current status of all available Line signals at time of polling in a single bitfield.
  - default access: read only
  - default value: `0`
  - default range: 0 - 15

`OffsetX` : `int`  
  Vertical offset from the origin to the AOI (in pixels).
  - default access: read/write
  - default value: `0`
  - default range: 0 - 0

`OffsetY` : `int`  
  Horizontal offset from the origin to the AOI (in pixels).
  - default access: read/write
  - default value: `0`
  - default range: 0 - 0

`ParameterSelector` : `enum`  
  Selects which parameter whose limit will be removed.
  - default access: read/write
  - default value: `'Gain'`
  - possible values: `'Gain'`

`PayloadSize` : `int`  
  Number of bytes transferred for each image or chunk on the stream channel.
  - default access: read only
  - default value: `786432`
  - default range: 0 - 4294967295

`PixelCoding` : `enum`  
  Coding of the pixels in the image.
  - default access: read only
  - default value: `'Raw'`
  - possible values: `'Mono'`, `'MonoSigned'`, `'RGBPacked'`, `'YUV411Packed'`, `'YUV422Packed'`, `'YUV444Packed'`, `'Raw'`

`PixelColorFilter` : `enum`  
  Type of color filter that is applied to the image.
  - default access: read only
  - default value: `'BayerRG'`
  - possible values: `'BayerRG'`, `'BayerGB'`, `'BayerGR'`, `'BayerBG'`, `'None'`

`PixelDefectCoordinateRegAddress` : `int`  
  
  - default access: read only
  - default value: `4042264064`
  - default range: -9223372036854775808 - 9223372036854775807

`PixelDynamicRangeMax` : `int`  
  Indicates the maximum pixel value transferred from the camera.
  - default access: read only
  - default value: `255`
  - default range: -9223372036854775808 - 9223372036854775807

`PixelDynamicRangeMin` : `int`  
  Indicates the minimum pixel value transferred from the camera.
  - default access: read only
  - default value: `0`
  - default range: -9223372036854775808 - 9223372036854775807

`PixelFormat` : `enum`  
  Format of the pixel data.
  - default access: read/write
  - default value: `'BayerRG8'`
  - possible values:
    - `'Mono8'`: Pixel format set to Mono 8.
    - `'Mono12Packed'`: Pixel format set Mono 12 Packed.
    - `'Mono16'`: Pixel format set to Mono 16.
    - `'BayerGR8'`: Pixel format set to Bayer GR 8.
    - `'BayerRG8'`: Pixel format set to Bayer RG 8.
    - `'BayerGB8'`: Pixel format set to Bayer GB 8.
    - `'BayerBG8'`: Pixel format set to Bayer BG 8.
    - `'BayerGR12Packed'`: Pixel format set BayerGR 12 Packed.
    - `'BayerRG12Packed'`: Pixel format set BayerRG 12 Packed.
    - `'BayerGB12Packed'`: Pixel format set BayerGB 12 Packed.
    - `'BayerBG12Packed'`: Pixel format set BayerBG 12 Packed.
    - `'BayerGR16'`: Pixel format set BayerGR 16.
    - `'BayerRG16'`: Pixel format set BayerRG 16.
    - `'BayerGB16'`: Pixel format set BayerGB 16.
    - `'BayerBG16'`: Pixel format set BayerBG 16.
    - `'YUV411Packed'`: Pixel format set YUV411 Packed.
    - `'YUV422Packed'`: Pixel format set to YUV 422 Packed.
    - `'YUV444Packed'`: Pixel format set to YUV 444 Packed.
    - `'RGB8Packed'`: Pixel format set RGB 8 Packed.

`PixelSize` : `enum`  
  Size of a pixel in bits.
  - default access: read only
  - default value: `'Bpp8'`
  - possible values: `'Bpp8'`, `'Bpp10'`, `'Bpp12'`, `'Bpp16'`, `'Bpp24'`, `'Bpp32'`

`RemoveLimits` : `bool`  
  Specifies whether or not the parameter limit is removed.
  - default access: read/write
  - default value: `False`

`ReverseX` : `bool`  
  Flip horizontally the image sent by the device. The AOI is applied after the flip.
  - default access: read/write
  - default value: `False`

`Saturation` : `float`  
  Saturation of the image in percent.
  - default access: read only
  - default value: `100.0`
  - unit: %
  - default range: 0.0 - 399.90234375

`SaturationAuto` : `enum`  
  Controls the mode for automatic saturation adjustment.
  - default access: not available
  - possible values: `'Off'`, `'Once'`, `'Continuous'`

`SaturationEnabled` : `bool`  
  Enables/disables saturation adjustment.
  - default access: not available

`SensorHeight` : `int`  
  Effective height of the sensor in pixels.
  - default access: read only
  - default value: `1536`
  - default range: 0 - 65535

`SensorWidth` : `int`  
  Effective width of the sensor in pixels.
  - default access: read only
  - default value: `2048`
  - default range: 0 - 65535

`Sharpness` : `int`  
  Sharpness of the image.
  - default access: read only
  - default value: `1024`
  - default range: 0 - 4095

`SharpnessAuto` : `enum`  
  Controls the mode for automatic sharpness adjustment.
  - default access: not available
  - possible values: `'Off'`, `'Once'`, `'Continuous'`

`SharpnessEnabled` : `bool`  
  Enables/disables sharpness adjustment.
  - default access: not available

`SingleFrameAcquisitionMode` : `enum`  
  Selects type of single acquisition mode
  - default access: read/write
  - default value: `'FreeRunning'`
  - possible values: `'FreeRunning'`, `'Triggered'`

`StrobeDelay` : `float`  
  Sets the duration (in microseconds) of the delay before starting the Strobe Signal.
  - default access: not available
  - unit: us
  - default range: 0.0 - 65535.0

`StrobeDuration` : `float`  
  Sets the duration (in microseconds) of the Strobe Signal.
  - default access: not available
  - unit: us
  - default range: 0.0 - 65535.0

`StrobeLineCnt16Address` : `int`  
  
  - default access: read only
  - default value: `4042266368`
  - default range: -9223372036854775808 - 9223372036854775807

`StrobeLineCntCtrlAddress` : `int`  
  
  - default access: read only
  - default value: `4042266112`
  - default range: -9223372036854775808 - 9223372036854775807

`StrobeRegBaseAddress` : `int`  
  
  - default access: read only
  - default value: `4042265344`
  - default range: -9223372036854775808 - 9223372036854775807

`TLParamsLocked` : `int`  
  
  - default access: read/write
  - default value: `0`
  - default range: 0 - 1

`TestImageSelector` : `enum`  
  Selects the type of test image that is sent by the camera.
  - default access: read/write
  - default value: `'Off'`
  - possible values: `'Off'`, `'TestImage1'`, `'TestImage2'`

`TestPattern` : `enum`  
  Selects the type of test pattern that is generated by the device as image source.
  - default access: read/write
  - default value: `'Off'`
  - possible values:
    - `'Off'`: Image is coming from the sensor.
    - `'TestImage1'`
    - `'TestImage2'`

`TransmitFailureCount` : `int`  
  Number of failed frame transmissions that have occurred since the last reset.
  - default access: read only
  - default value: `52`
  - default range: 0 - 2147483647

`TriggerActivation` : `enum`  
  Specifies the activation mode of the trigger.
  - default access: read/write
  - default value: `'FallingEdge'`
  - possible values: `'RisingEdge'`, `'FallingEdge'`

`TriggerDelay` : `float`  
  Specifies the delay (in microseconds) to apply after the trigger reception before activating it.
  - default access: read/write
  - default value: `0.0`
  - unit: us
  - default range: 0.0 - 10739.86291885376

`TriggerDelayEnabled` : `bool`  
  Specifies whether or not the Trigger Delay is enabled.
  - default access: read/write
  - default value: `False`

`TriggerMode` : `enum`  
  Controls whether or not the selected trigger is active.
  - default access: read/write
  - default value: `'Off'`
  - possible values: `'Off'`, `'On'`

`TriggerOverlap` : `enum`  
  Specifies the type trigger overlap permitted with the previous frame.
  - default access: not available
  - possible values:
    - `'Off'`: No trigger overlap is permitted.
    - `'ReadOut'`: Trigger is accepted immediately after the exposure period.

`TriggerSelector` : `enum`  
  Selects the type of trigger to configure.
  - default access: read/write
  - default value: `'FrameStart'`
  - possible values: `'FrameStart'`, `'ExposureActive'`

`TriggerSource` : `enum`  
  Specifies the internal signal or physical input line to use as the trigger source. The selected trigger must have its TriggerMode set to On.
  - default access: read/write
  - default value: `'Line0'`
  - possible values: `'Software'`, `'Line0'`, `'Line1'`, `'Line2'`, `'Line3'`

`UserDefinedValue` : `int`  
  User defined value.
  - default access: read/write
  - default value: `0`
  - default range: -2147483648 - 2147483647

`UserDefinedValueSelector` : `enum`  
  Used to select from a set of user defined values
  - default access: read/write
  - default value: `'Value1'`
  - possible values:
    - `'Value1'`: User defined value1.
    - `'Value2'`: User defined value2.
    - `'Value3'`: User defined value3.
    - `'Value4'`: User defined value4.
    - `'Value5'`: User defined value5.

`UserOutputPinRegBaseAddress` : `int`  
  
  - default access: read only
  - default value: `4042264864`
  - default range: -9223372036854775808 - 9223372036854775807

`UserOutputSelector` : `enum`  
  Selects the physical line (or pin) of the external device connector to configure.
  - default access: read/write
  - default value: `'UserOutputValue1'`
  - possible values: `'UserOutputValue1'`, `'UserOutputValue2'`, `'UserOutputValue3'`

`UserOutputStrobeCntCtrlAddress` : `int`  
  
  - default access: read only
  - default value: `4042266116`
  - default range: -9223372036854775808 - 9223372036854775807

`UserOutputValue` : `bool`  
  Sets the value of the bit to be output to the selected line.
  - default access: not available

`UserSetCurrent` : `int`  
  Indicates the user set that is currently in use.  At initialization time, the camera loads the most recently saved user set.
  - default access: read only
  - default value: `0`
  - default range: 0 - 15

`UserSetDefault` : `enum`  
  Selects the feature User Set to load and make active by default when the device is reset.
  - default access: read/write
  - default value: `'Default'`
  - possible values:
    - `'Default'`: Select the factory setting user set.
    - `'UserSet1'`: Select the user set 1.
    - `'UserSet2'`: Select the user set 2.

`UserSetDefaultSelector` : `enum`  
  Selects the feature User Set to load, save or configure.
  - default access: read/write
  - default value: `'Default'`
  - possible values:
    - `'Default'`: Factory default settings.
    - `'UserSet1'`
    - `'UserSet2'`

`UserSetSelector` : `enum`  
  Selects the feature User Set to load, save or configure.
  - default access: read/write
  - default value: `'Default'`
  - possible values:
    - `'Default'`: Factory default settings.
    - `'UserSet1'`: Selects the user set 1.
    - `'UserSet2'`: Selects the user set 2.

`VideoMode` : `enum`  
  Current video mode.
  - default access: read/write
  - default value: `'Mode2'`
  - possible values: `'Mode0'`, `'Mode1'`, `'Mode2'`, `'Mode3'`, `'Mode4'`, `'Mode5'`, `'Mode6'`, `'Mode7'`, `'Mode8'`, `'Mode9'`, `'Mode10'`, `'Mode11'`, `'Mode12'`, `'Mode13'`, `'Mode14'`, `'Mode15'`

`Width` : `int`  
  Width of the image provided by the device (in pixels).
  - default access: read/write
  - default value: `1024`
  - default range: 4 - 1024

`WidthMax` : `int`  
  Maximum width of the image (in pixels).
  - default access: read only
  - default value: `1024`
  - default range: 0 - 65535

`pGevTimestampTickFrequencyHighReg` : `int`  
  
  - default access: read only
  - default value: `0`
  - default range: 0 - 4294967295

`pGevTimestampTickFrequencyLowReg` : `int`  
  
  - default access: read only
  - default value: `125000000`
  - default range: 0 - 4294967295

`pgrAutoExposureCompensationLowerLimit` : `float`  
  Lower limit of the auto exposure compensation value(EV) parameter
  - default access: read/write
  - default value: `0.4150390625`
  - unit: EV
  - default range: -70.0849609375 - 2.0

`pgrAutoExposureCompensationUpperLimit` : `float`  
  Upper limit of the auto exposure compensation value(EV) parameter
  - default access: read/write
  - default value: `2.0`
  - unit: EV
  - default range: 0.4150390625 - 4.41473388671875

`pgrCurrentCorrectedPixelCount` : `int`  
  Current number of pixels that are being corrected.
  - default access: read only
  - default value: `35`
  - default range: 0 - 60

`pgrCurrentCorrectedPixelIndex` : `int`  
  Control the index of the defected pixels to be corrected.
  - default access: read only
  - default value: `0`
  - default range: 0 - 34

`pgrCurrentCorrectedPixelOffsetX` : `int`  
  Control the X offset of the defect pixel specified by the index.
  - default access: read only
  - default value: `547`
  - default range: 0 - 2047

`pgrCurrentCorrectedPixelOffsetY` : `int`  
  Control the Y offset of the defect pixel specified by the index.
  - default access: read only
  - default value: `42`
  - default range: 0 - 1535

`pgrDefectPixelCorrectionEnable` : `bool`  
  Enable or disable pixel correction.
  - default access: read only
  - default value: `True`

`pgrDefectPixelCorrectionTestMode` : `enum`  
  Controls whether or not the defect pixel correction test mode is active.
  - default access: read only
  - default value: `'Off'`
  - possible values: `'Off'`, `'On'`

`pgrDefectPixelCorrectionType` : `enum`  
  Specifies the current defect pixel correction type.
  - default access: read only
  - default value: `'FPGACorrection'`
  - possible values: `'FPGACorrection'`, `'SensorCorrection'`

`pgrDevicePowerSupplySelector` : `enum`  
  Selects the power supply or source to control or read.
  - default access: read/write
  - default value: `'External'`
  - possible values:
    - `'External'`: Power coming externally into the camera, often provided through the GPIO connector.
    - `'LinkPower'`: Power coming to the camera over the link (typically USB or PoE).
    - `'0'`: Internally generated 5V rail.
    - `'3'`: Internally generated 3.3V rail.
    - `'Vpos'`: Internally generated positive voltage.
    - `'Vneg'`: Internally generated negative voltage.
    - `'SensorPower'`: Internally generated sensor power.

`pgrDeviceUptime` : `int`  
  Time since the device was powered up.
  - default access: read only
  - default value: `2543`
  - default range: 0 - 4294967295

`pgrExposureCompensation` : `float`  
  The measured or target image plane illuminance in EV.
  - default access: read only
  - default value: `1.02978515625`
  - unit: EV
  - default range: -7.5849609375 - 2.41363525390625

`pgrExposureCompensationAuto` : `enum`  
  Sets the automatic exposure compensation value mode.
  - default access: read/write
  - default value: `'Continuous'`
  - possible values: `'Off'`, `'Once'`, `'Continuous'`

`pgrHDRImageSelector` : `enum`  
  Selects the HDR image.
  - default access: not available
  - possible values:
    - `'Image1'`: HDR image 1.
    - `'Image2'`: HDR image 2.
    - `'Image3'`: HDR image 3.
    - `'Image4'`: HDR image 4.

`pgrHDRModeEnabled` : `bool`  
  Specifies whether or not the High Dynamic Range mode is enabled.
  - default access: read/write
  - default value: `False`

`pgrPixelBigEndian` : `bool`  
  Set pixel endianess for pixel format Mono16.
  - default access: read/write
  - default value: `False`

`pgrPowerSourcePresent` : `enum`  
  Indicates if the selected power source currently has power present.
  - default access: not available
  - possible values: `'False'`, `'True'`

`pgrPowerSupplyCurrent` : `float`  
  Indicates the output current of the selected power supply.
  - default access: read only
  - default value: `0.19299983978271484`
  - unit: A
  - default range: -3.4028234663852886e+38 - 3.4028234663852886e+38

`pgrPowerSupplyEnable` : `enum`  
  Enables or disables the selected power supply.
  - default access: not available
  - possible values: `'False'`, `'True'`

`pgrPowerSupplyVoltage` : `float`  
  Indicates the current voltage of the selected power supply.
  - default access: read only
  - default value: `9.902999877929688`
  - unit: V
  - default range: -3.4028234663852886e+38 - 3.4028234663852886e+38

`pgrSensorDescription` : `string`  
  Description of the sensor of the device.
  - default access: read only
  - default value: `'Sony IMX265 (1/1.8" Color CMOS)'`

Commands
--------

**Note: the camera recording should be started/stopped using the `start` and `stop` methods, not any of the functions below (see simple_pyspin documentation).**

`AcquisitionStart()`:  
  Starts the Acquisition of the device.
  - default access: write only

`AcquisitionStop()`:  
  Stops the acquisition of the device at the end of the current frame.
  - default access: write only

`ActivePageSave()`:  
  Save the data in the active page to the data flash.
  - default access: write only

`DeviceReset()`:  
  This is a command that immediately resets and reboots the device.
  - default access: write only

`GevTimestampControlLatch()`:  
  This command latches the current timestamp value of the device.
  - default access: write only

`GevTimestampControlReset()`:  
  This command resets the timestamp value for the device.
  - default access: write only

`TransmitFailureCountReset()`:  
  Reset the transmit failure count.
  - default access: write only

`TriggerSoftware()`:  
  Generates an internal trigger if Trigger Source is set to Software.
  - default access: not available

`UserSetLoad()`:  
  Loads the User Set specified by UserSetSelector to the device and makes it active.
  - default access: write only

`UserSetSave()`:  
  Save the User Set specified by UserSetSelector to the non-volatile memory of the device.
  - default access: not available

`pgrCurrentCorrectedPixelSave()`:  
  Save the Current Corrected Pixels to the non-volatile memory of the device.
  - default access: not available
