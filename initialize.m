function [intrinsics,name,depthScale,cfg,pipe] = initialize (filename)
    mustBeText(filename);
    clear
    clc
    cfg = realsense.config();
    validateattributes(filename, {'char','string'}, {'scalartext', 'nonempty'}, '', 'filename', 1);
    % Tell pipeline to stream from the given rosbag file
    cfg.enable_device_from_file(filename);
    % Make Pipeline object to manage streaming
    pipe = realsense.pipeline();
    % Make Colorizer object to prettify depth output
    %colorizer = realsense.colorizer();

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

    name = dev.get_info(realsense.camera_info.name);
    depthSensor = dev.first('depth_sensor');
    depthScale = depthSensor.get_depth_scale();
    pipe.stop();
end