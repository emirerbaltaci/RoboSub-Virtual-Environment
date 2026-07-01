function rosmsgOut = Image(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.header = bus_conv_fcns.ros2.busToMsg.std_msgs.Header(slBusIn.header,rosmsgOut.header(1));
    rosmsgOut.height = uint32(slBusIn.height);
    rosmsgOut.width = uint32(slBusIn.width);
    rosmsgOut.encoding = char(slBusIn.encoding);
    if slBusIn.encoding_SL_Info.CurrentLength < numel(slBusIn.encoding)
    rosmsgOut.encoding(slBusIn.encoding_SL_Info.CurrentLength+1:numel(slBusIn.encoding)) = [];
    end
    rosmsgOut.is_bigendian = uint8(slBusIn.is_bigendian);
    rosmsgOut.step = uint32(slBusIn.step);
    rosmsgOut.data = uint8(slBusIn.data(1:slBusIn.data_SL_Info.CurrentLength));
end
