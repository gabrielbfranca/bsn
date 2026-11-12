# Development container

Open the repository in VS Code and run **Dev Containers: Rebuild and Reopen in
Container** after changing `Dockerfile.dev`.

The project uses ROS Melodic on Ubuntu 18.04. Modern VS Code Server requires a
newer glibc, so the Dockerfile provides an isolated x86-64 runtime from Ubuntu
22.04 under `/opt/vscode-sysroot`, plus statically linked patchelf 0.18.0.
The `VSCODE_SERVER_CUSTOM_GLIBC_*` variables select it for the server. The ROS
environment continues using the original system libraries; do not export this
runtime through `LD_LIBRARY_PATH`.

This follows the [VS Code custom sysroot workaround](https://code.visualstudio.com/docs/remote/faq#_can-i-run-vs-code-server-on-older-linux-distributions),
which Microsoft documents as an unsupported compatibility workaround. Extensions
with their own native binaries can have additional Linux requirements.

On Windows, if startup fails mounting WSLg's `wayland-0`, disable **Dev Containers:
Mount Wayland Socket** (`dev.containers.mountWaylandSocket`) in VS Code's **User**
settings, reload the window, and reopen the container. This setting applies to all
devcontainers and disables their automatic Wayland socket sharing.
