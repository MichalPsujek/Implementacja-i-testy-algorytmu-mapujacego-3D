function [K,depths,colors,depthScale,numerek] = strimek (filename)
    mustBeText(filename);

    bag=rosbag(filename);

    depths=select(bag,"Topic","/device_0/sensor_0/Depth_0/image/data");
    colors=select(bag,"Topic","/device_0/sensor_1/Color_0/image/data");

    numerek=rosbag(filename);
    numerek=min(numerek.AvailableTopics.NumMessages(2),numerek.AvailableTopics.NumMessages(49));

    cfg = realsense.config();
    validateattributes(filename, {'char','string'}, {'scalartext', 'nonempty'}, '', 'filename', 1);
    % Tell pipeline to stream from the given rosbag file
    cfg.enable_device_from_file(filename);
    % Make Pipeline object to manage streaming
    pipe = realsense.pipeline();

    % Start streaming on an arbitrary camera with default settings
    profile = pipe.start(cfg);

    % Get streaming device's name
    dev = profile.get_device();
    streams=profile.get_streams();
    stream_profile=streams{1,1};
    stream_profile=stream_profile.as('video_stream_profile');
    raw_intrinsics=stream_profile.get_intrinsics();
    focalLength=[raw_intrinsics.fx raw_intrinsics.fy];
    principalPoint=[raw_intrinsics.ppx raw_intrinsics.ppy];
    imageSize=single([raw_intrinsics.height raw_intrinsics.width]);
    intrinsics=cameraIntrinsics(focalLength,principalPoint,imageSize);

    depthSensor = dev.first('depth_sensor');
    depthScale = depthSensor.get_depth_scale();

    K = [intrinsics.FocalLength(1,1) 0 intrinsics.PrincipalPoint(1,1); 0 intrinsics.FocalLength(1,2) intrinsics.PrincipalPoint(1,2); 0 0 1];

    pipe.stop();
end