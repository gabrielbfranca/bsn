void positive() {
    // ruleid: isolation.cpp.file-stream-extra
    std::fstream stream;
    // ruleid: isolation.cpp.file-stream-extra
    fp.open(path);
    // ruleid: isolation.cpp.high-resolution-clock
    std::chrono::high_resolution_clock::now();
    // ruleid: isolation.cpp.ros-parameters
    handle.getParam("frequency", f);
    // ruleid: isolation.cpp.ros-rate-sleep
    loop_rate.sleep();
    // ruleid: isolation.cpp.signal-handler
    signal(SIGINT, cb);
    // ruleid: isolation.cpp.ros-package-path
    ros::package::getPath("repository");
    // ruleid: isolation.cpp.directory-io
    mkdir(path, mode);
}
void negative() {
    // ok: isolation.cpp.file-stream-extra
    ordinary.open(path);
    // ok: isolation.cpp.high-resolution-clock
    fake_clock.now();
    // ok: isolation.cpp.ros-parameters
    config.compute();
    // ok: isolation.cpp.ros-rate-sleep
    fake.sleep();
    // ok: isolation.cpp.signal-handler
    emitSignal();
    // ok: isolation.cpp.ros-package-path
    paths.getPath();
    // ok: isolation.cpp.directory-io
    make_value();
}
