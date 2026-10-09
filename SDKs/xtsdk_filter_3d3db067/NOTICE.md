# XT host-filter runtime bundle

These two unchanged vendor binaries are included to reproduce the project's pinned filter runtime. They are vendor software, not project-authored configuration or an independently implemented filter.

- Upstream: https://github.com/XT-Toffuture/xtsdk_cpp
- Commit: `3d3db067ae9bdc0528202c3087bc10fd3b706638`
- Original paths: `xtsdk/lib/linux/{aarch64,x86_64}/libxtsdk_shared.so`
- Local paths omit the upstream `xtsdk/` prefix.
- aarch64 SHA-256: `3336a76590b5447efd7c037929e61c287523fcd79e8125589a3adb35eee83411`
- x86_64 SHA-256: `69d80f0d64f1b7a65c1cf76aa57d7ba84fb0ff34a03014e6d04d52092e683821`

The pinned upstream tree contains no LICENSE, COPYING or NOTICE file. Its [README_EN.md](https://github.com/XT-Toffuture/xtsdk_cpp/blob/3d3db067ae9bdc0528202c3087bc10fd3b706638/README_EN.md) states: “This code is confidential.” Vendor ownership and applicable terms remain with the vendor; this repository does not relicense these binaries or assert an open-source grant for them.

The project's configuration, integration patch and identity checks are maintained separately in `config/` and `vendor_patches/`. Both binaries are required by the pinned manifest; CMake checks their original bytes. Do not replace the original SDK's other binary with this bundle. See [deployment](../../docs/deployment.md) for the clone and build workflow.
